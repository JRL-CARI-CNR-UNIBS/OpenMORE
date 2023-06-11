#include "replanners_lib/replanners/MARSHA.h"

namespace pathplan
{
MARSHA::MARSHA(const Eigen::VectorXd& current_configuration, const PathPtr& current_path,
               const double& max_time, const TreeSolverPtr &solver):
  MARS(current_configuration,current_path,max_time,solver)
{
  init(nullptr);
}

MARSHA::MARSHA(const Eigen::VectorXd& current_configuration, const PathPtr& current_path,
               const double& max_time, const TreeSolverPtr &solver, const std::vector<PathPtr> &other_paths):
  MARS(current_configuration,current_path,max_time,solver,other_paths)
{
  init(nullptr);
}

MARSHA::MARSHA(const Eigen::VectorXd& current_configuration, const PathPtr& current_path,
               const double& max_time, const TreeSolverPtr &solver, const std::vector<PathPtr> &other_paths,
               const LengthPenaltyMetricsPtr& ha_metrics):
  MARS(current_configuration,current_path,max_time,solver,other_paths)
{
  init(ha_metrics);
}

void MARSHA::init(const LengthPenaltyMetricsPtr& ha_metrics)
{
  if(ha_metrics == nullptr)
  {
    metrics_    = nullptr;
    ha_metrics_ = nullptr;
    ssm_ = nullptr;
  }
  else
    setMetricsHA(ha_metrics);

  full_net_search_   = false;
  euclidean_metrics_ = std::make_shared<Metrics>();
  cost_updated_flag_ = Connection::getReservedFlagsNumber(); //the first free position in Connection::flags_ vector where we can store our new custom flag

  //Lambda function definition for Net -> it determines when cost of a connection should be re-evaluated
  cost_evaluation_condition_ =
      std::make_shared<std::function<bool (const ConnectionPtr& connection)>>([&](const ConnectionPtr& connection)->bool{
    if(connection->getFlag(cost_updated_flag_,false) ||
       (connection->isRecentlyChecked() && connection->getCost() == std::numeric_limits<double>::infinity()))
    {
      return false;
    }
    else
    {
      connection->setFlag(cost_updated_flag_,true);  //connection's cost will be re-evaluated by net using the metrics, set its flag to true
      flagged_connections_.push_back(connection);

      return true;
    }
  });

  net_->setCostEvaluationCondition(cost_evaluation_condition_);
}

bool MARSHA::setObstaclesPosition(const Eigen::Matrix<double,3,Eigen::Dynamic>& obstacles_positions)
{
  if(ssm_ != nullptr)
  {
    ssm_->setObstaclesPositions(obstacles_positions);
    return true;
  }
  return false;
}

bool MARSHA::addObstaclePosition(const Eigen::Vector3d& obstacle_position)
{
  if(ssm_ != nullptr)
  {
    ssm_->addObstaclePosition(obstacle_position);
    return true;
  }
  return false;
}

void MARSHA::setSSM()
{
  if(std::dynamic_pointer_cast<ssm15066_estimator::SSM15066Estimator>(ha_metrics_->getPenalizer()) == nullptr)
    throw std::runtime_error("Cost penalizer should be of SSM15066Estimator type");
  else
    ssm_ = std::static_pointer_cast<ssm15066_estimator::SSM15066Estimator>(ha_metrics_->getPenalizer());
}

void MARSHA::setMetricsHA(const LengthPenaltyMetricsPtr& ha_metrics)
{
  ha_metrics_ = ha_metrics;
  metrics_    = ha_metrics;

  net_->setMetrics(ha_metrics_);
  tree_->setMetrics(ha_metrics_);
  solver_->setMetrics(ha_metrics_);
  current_path_->setMetrics(ha_metrics_);

  for(const PathPtr& p:other_paths_)
    p->setMetrics(ha_metrics_);

  setSSM();
}

void MARSHA::initFlaggedConnections()
{
  clearFlaggedConnections();

  flagged_connections_ = current_path_->getConnections();
  for(const PathPtr& p:other_paths_)
    flagged_connections_.insert(flagged_connections_.end(),p->getConnectionsConst().begin(),p->getConnectionsConst().end());

  std::for_each(flagged_connections_.begin(),flagged_connections_.end(),
                [&](const ConnectionPtr& flagged_conn) ->void{
    flagged_conn->setRecentlyChecked(true);
    flagged_conn->setFlag(cost_updated_flag_,true);
  });
}

void MARSHA::clearFlaggedConnections()
{
  std::for_each(flagged_connections_.begin(),flagged_connections_.end(),
                [&](const ConnectionPtr& flagged_conn) ->void{
    flagged_conn->setRecentlyChecked(false);
    flagged_conn->setFlag(cost_updated_flag_,false);
  });

  flagged_connections_.clear();
}

std::vector<NodePtr> MARSHA::startNodes(const std::vector<ConnectionPtr>& subpath1_conn)
{
  /* Unlike MARS, avoid nodes for which the subpath to reach them is expensive */

  std::vector<NodePtr> start_node_vector;
  if(subpath1_conn.front()->getCost() >= expensive_cost_)
  {
    NodePtr current_node = subpath1_conn.front()->getParent();

    assert(current_node->getParentConnectionsSize() == 1);

    if(not current_node->getFlag(examined_flag_,false))
      start_node_vector.push_back(current_node);
  }
  else
  {
    /* If the current connection is not too expensive, all the nodes between the current child to the parent
     * of the connection expensive are considered as starting points for the replanning */

    for(const ConnectionPtr& conn:subpath1_conn)
    {
      if(conn == subpath1_conn.front())
        continue;
      else if(conn == subpath1_conn.back())
      {
        /* If the path is free, you can consider all the nodes but it is useless to consider
         * the last one before the goal (it is already connected to the goal with a straight line) */

        if(conn->getCost() >= expensive_cost_ && (not conn->getParent()->getFlag(examined_flag_,false)))
          start_node_vector.push_back(conn->getParent());
      }
      else
      {
        if(not conn->getParent()->getFlag(examined_flag_,false))
          start_node_vector.push_back(conn->getParent());

        if(conn->getCost() >= expensive_cost_)
          break;
      }
    }
  }

  if(reverse_start_nodes_)
    std::reverse(start_node_vector.begin(),start_node_vector.end());

  if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
    ROS_GREEN_STREAM("NEW J: "<<(int) (start_node_vector.size()-1));

  return start_node_vector;
}

std::vector<ps_goal_ptr> MARSHA::sortNodes(const NodePtr& start_node)
{
  /* Sort nodes based on the metrics utopia.
   * Prioritize nodes with low cost subpath to goal
   */

  PathPtr tmp_path;
  std::vector<NodePtr> nodes;
  ps_goal_ptr pathswitch_goal;
  std::vector<ps_goal_ptr> goals;
  std::vector<NodePtr> considered_nodes;
  std::multimap<double,ps_goal_ptr> ps_goals_map;

  double utopia;
  bool start_node_belongs_to_p;
  bool goal_node_considered = false;

  for(const PathPtr& p:admissible_other_paths_)
  {
    nodes = p->getNodes();
    if(goal_node_considered)
    {
      assert(nodes.back() == goal_node_);
      nodes.pop_back();
    }

    if(std::find(nodes.begin(),nodes.end(),start_node)<nodes.end())
      start_node_belongs_to_p = true;
    else
      start_node_belongs_to_p = false;

    for(const NodePtr& n:nodes)
    {
      if(std::find(considered_nodes.begin(),considered_nodes.end(),n)<considered_nodes.end())
        continue;

      utopia = metrics_->utopia(start_node->getConfiguration(),n->getConfiguration());

      if(utopia<TOLERANCE)
        continue;

      if(start_node_belongs_to_p)
      {
        // Do not connect nodes which are on the same path and already connected by a straight connection (only if there are no obstacles in between)
        tmp_path = p->getSubpathFromNode(start_node);
        tmp_path = tmp_path->getSubpathToNode(n);
        if(tmp_path->cost()<std::numeric_limits<double>::infinity())
        {
          if(std::abs(tmp_path->computeEuclideanNorm()-utopia)<TOLERANCE)
          {
            if(n == goal_node_)
              goal_node_considered = true;

            if(pathSwitch_verbose_)
              ROS_RED_STREAM("node removed from Q2 list: "<<n);
            continue;
          }
        }
      }

      pathswitch_goal = std::make_shared<ps_goal>();
      pathswitch_goal->node = n;
      pathswitch_goal->utopia = utopia;

      if(n != goal_node_)
      {
        pathswitch_goal->subpath = p->getSubpathFromNode(n);
        pathswitch_goal->subpath_cost = pathswitch_goal->subpath->cost();
      }
      else
      {
        goal_node_considered = true;
        pathswitch_goal->subpath = nullptr;
        pathswitch_goal->subpath_cost = 0.0;
      }

      considered_nodes.push_back(n);

      if(pathswitch_goal->subpath_cost < expensive_cost_)
        ps_goals_map.insert(std::pair<double,ps_goal_ptr>(utopia,pathswitch_goal));
    }
  }

  for(const std::pair<double,ps_goal_ptr> &p: ps_goals_map)
    goals.push_back(p.second);

  return goals;
}

bool MARSHA::computeConnectingPath(const NodePtr& path1_node, const NodePtr& path2_node, const double& diff_subpath_cost,
                                   const PathPtr& current_solution, const ros::WallTime& tic, const ros::WallTime& tic_cycle,
                                   PathPtr& connecting_path, bool& quickly_solved)
{
  connecting_path = nullptr;

  /* Create a subtree rooted at path1_node. It will be used to build the connecting path between path1_node and path2_node */
  std::vector<NodePtr> black_list;
  std::vector<PathPtr> paths = other_paths_;
  paths.push_back(current_path_);
  paths.push_back(current_solution);

  for(const PathPtr& p:paths)
  {
    std::vector<NodePtr> nodes = p->getNodes();
    black_list.insert(black_list.end(),nodes.begin(),nodes.end());
  }
  assert(not black_list.empty());

  SubtreePtr subtree = pathplan::Subtree::createSubtree(tree_,path1_node,
                                                        path1_node->getConfiguration(),
                                                        path2_node->getConfiguration(),
                                                        diff_subpath_cost,
                                                        black_list,true); //collision check before adding a node
  subtree->hideInvalidBranches(subtree->getRoot());

  assert([&]() ->bool{
           std::vector<NodePtr> leaves;
           subtree->getLeaves(leaves);

           for(const NodePtr& n:leaves)
           {
             std::vector<ConnectionPtr> conns = subtree->getConnectionToNode(n);
             for(const ConnectionPtr& c:conns)
             {
               if(c->getCost() == std::numeric_limits<double>::infinity())
               return false;
             }
           }
           return true;
         }());

  if(pathSwitch_disp_)
  {
    disp_->changeConnectionSize({0.025,0.025,0.025});
    int subtree_id = disp_->displaySubtree(subtree,"pathplan",{0.0,0.0,0.0,1.0});
    disp_->nextButton("Displaying subtree..");
    disp_->clearMarker(subtree_id);
    disp_->defaultConnectionSize();
  }

  /* Search for an already existing solution between path1_node and path2_node */
  if(pathSwitch_verbose_)
    ROS_YELLOW_STREAM("Searching for an already existing solution in the subtree..");

  ros::WallTime tic_search = ros::WallTime::now();

  bool search_in_subtree = true;
  double net_time = maxSolverTime(tic,tic_cycle);

  NetPtr net = std::make_shared<Net>(subtree);
  net->setCostEvaluationCondition(cost_evaluation_condition_);
  std::multimap<double,std::vector<ConnectionPtr>> already_existing_solutions_map = net->getConnectionBetweenNodes(path1_node,path2_node,diff_subpath_cost,
                                                                                                                   black_list,net_time,search_in_subtree);
  double time_search = (ros::WallTime::now()-tic_search).toSec();

  if(pathSwitch_verbose_)
    ROS_YELLOW_STREAM("In the subtree exist "<< already_existing_solutions_map.size() <<" paths to path2_node (time to search "<<time_search<<" seconds)");

  tic_search = ros::WallTime::now();
  unsigned int number_of_candidates = 0;
  double already_existing_solution_cost;
  std::vector<ConnectionPtr> already_existing_solution_conn;
  if(findValidSolution(already_existing_solutions_map,diff_subpath_cost,already_existing_solution_conn,
                       already_existing_solution_cost,number_of_candidates,false))
  {
    assert(ha_metrics_ == metrics_);
    connecting_path = std::make_shared<Path>(already_existing_solution_conn,metrics_,checker_);
    connecting_path->setTree(tree_);
    quickly_solved = true;

    assert([&]() ->bool{
             for(const ConnectionPtr& c:already_existing_solution_conn)
             {
               if(c->getFlag(cost_updated_flag_,false) != true)
               {
                 return false;
               }
             }
             return true;
           }());
    assert(already_existing_solution_cost == connecting_path->cost());

    if(pathSwitch_verbose_)
      ROS_BOLDYELLOW_STREAM("A solution with cost "<< already_existing_solution_cost<<" has been found in the subtree in "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds! Making it a solution of the subtree..");

    convertToSubtreeSolution(connecting_path,black_list);

    if(pathSwitch_disp_)
    {
      disp_->changeConnectionSize({0.02,0.02,0.02});
      int connecting_path_id = disp_->displayPath(connecting_path,"pathplan",{1.0,0.4,0.0,1.0});
      disp_->nextButton("Displaying connecting_path..");
      disp_->clearMarker(connecting_path_id);
      disp_->defaultConnectionSize();
    }

    return true;
  }
  else
  {
    if(pathSwitch_verbose_)
    {
      if(number_of_candidates>0)
        ROS_YELLOW_STREAM(number_of_candidates<< " candidate solutions found in the subtree but no one was free (check time "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds)");
      else
        ROS_YELLOW_STREAM("No candidate solutions found in the subtree (search time "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds)");
    }

    //Remove the invalid branches from the subtree
    for(const std::pair<double,std::vector<ConnectionPtr>>& p:already_existing_solutions_map)
    {
      for(const ConnectionPtr& c:p.second)
      {
        if(c->getCost() == std::numeric_limits<double>::infinity())
          subtree->hideFromSubtree(c->getChild());
      }
    }
  }

  /* If no solutions already exist, search for a new one. The ellipsoide determined
   * by diff_subpath_cost is used to sample the space. Outside of this ellipsoid,
   * the nodes would create an inconvenient connecting_path */

  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node->getConfiguration(),
                                                         path2_node->getConfiguration(),
                                                         lb_, ub_,diff_subpath_cost);

  std::vector<NodePtr> subtree_nodes;
  NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

  bool solver_has_solved = false;
  bool valid_connecting_path_found = false;

  double solver_time = maxSolverTime(tic,tic_cycle);
  double available_search_time = solver_time;
  ros::WallTime tic_before_search = ros::WallTime::now();

  while(available_search_time>0)
  {
    solver_->resetProblem();
    solver_->setSampler(sampler);
    solver_->addStart(path1_node);
    solver_->setStartTree(subtree);

    solver_->setMetrics(euclidean_metrics_);
    subtree->setMetrics(euclidean_metrics_);

    connecting_path = nullptr;
    subtree_nodes = subtree->getNodesConst(); //nodes already in the subtree

    available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();
    if(pathSwitch_verbose_)
      ROS_YELLOW_STREAM("Searching for a connecting path...max time: "<<available_search_time);

    ros::WallTime tic_solver = ros::WallTime::now();
    solver_->addGoal(path2_node_fake,available_search_time*0.85);
    ros::WallTime toc_solver = ros::WallTime::now();

    quickly_solved = solver_->solved();

    if(quickly_solved)
    {
      solver_has_solved = true;
      connecting_path = solver_->getSolution();
    }
    else
    {
      available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();

      if(pathSwitch_verbose_)
      {
        ROS_YELLOW_STREAM("Direct connection NOT found (time "<<(toc_solver-tic_solver).toSec()<<" s)");
        ROS_YELLOW_STREAM("Solving...max time: "<<available_search_time);
      }

      tic_solver = ros::WallTime::now();
      solver_has_solved = solver_->solve(connecting_path,10000,available_search_time*0.85);
      toc_solver = ros::WallTime::now();
    }

    if(solver_has_solved)
    {
      if(pathSwitch_verbose_)
        ROS_YELLOW_STREAM("Solved in "<<(toc_solver-tic_solver).toSec()<<" s (direct connection to goal: "<<quickly_solved<<")");

      bool subtree_valid = true;
      ConnectionPtr obstructed_connection = nullptr;

      for(const ConnectionPtr& c:connecting_path->getConnections())
      {
        available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();
        if(available_search_time<=0.0)
        {
          subtree_valid = false;
          quickly_solved = false;
          break;
        }

        std::vector<NodePtr>::iterator it = std::find(subtree_nodes.begin(),subtree_nodes.end(),c->getChild());

        //lazy collision check of the connections that already were in the subtree
        if(it<subtree_nodes.end())
        {
          if(not subtree_valid)
            continue;
          if(c->isRecentlyChecked())
          {
            if(c->getCost() == std::numeric_limits<double>::infinity())
            {
              ROS_INFO_STREAM(*c);
              assert(0);
            }
            else
              continue;
          }

          if(not checker_->checkConnection(c))
          {
            invalid_connection_ptr invalid_conn = std::make_shared<invalid_connection>();
            invalid_conn->connection = c;
            invalid_conn->cost = c->getCost();
            invalid_connections_.push_back(invalid_conn);

            c->setCost(std::numeric_limits<double>::infinity());

            obstructed_connection = c;
            subtree_valid = false;
            quickly_solved = false;
          }

          if(not c->isRecentlyChecked())
          {
            c->setRecentlyChecked(true);
            flagged_connections_.push_back(c);
          }
        }
        else //do not re-check connections just added to the subtree
        {
          assert(not c->isRecentlyChecked());
          c->setRecentlyChecked(true);
          flagged_connections_.push_back(c);
        }
      }

      if(subtree_valid)
      {
        assert([&]() ->bool{
                 PathPtr clone = connecting_path->clone();
                 if(connecting_path->isValid())
                 return true;

                 ROS_BOLDYELLOW_STREAM("clone "<<*clone);
                 ROS_BOLDRED_STREAM("connecting path "<<*connecting_path);
                 for(const NodePtr& n:subtree_nodes)
                 ROS_BOLDWHITE_STREAM("subtree node "<<n->getConfiguration().transpose()<<" ("<<n<<")");

                 return false;
               }());

        valid_connecting_path_found = true;
        break;
      }
      else
      {
        subtree->purgeFromHere(path2_node_fake);

        if(obstructed_connection != nullptr)
          subtree->hideFromSubtree(obstructed_connection->getChild());

        if(pathSwitch_verbose_)
          ROS_YELLOW_STREAM("Lazy check detected that subtree was not valid, compute again..");
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        ROS_YELLOW_STREAM("Not solved, time: "<<(toc_solver-tic_solver).toSec());

      break;
    }

    available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();
  }

  subtree->setMetrics(ha_metrics_);
  solver_->setMetrics(ha_metrics_);

  //If a collision free solution exists, search for the best route using net, which takes care also of updating the connections cost
  if(valid_connecting_path_found)
  {
    /* Search for the best solution in the subtree which connects path1_node to path2_node_fake */
    assert(path2_node_fake->getParentConnectionsSize   () == 1);
    assert(path2_node_fake->getNetChildConnectionsSize () == 0);
    assert(path2_node_fake->getNetParentConnectionsSize() == 0);

    net_time = maxSolverTime(tic,tic_cycle);

    number_of_candidates = 0;
    double connecting_path_cost;
    std::vector<ConnectionPtr> connecting_path_conn;

    ros::WallTime tic_net = ros::WallTime::now();
    net->setCostEvaluationCondition(cost_evaluation_condition_);
    std::multimap<double,std::vector<ConnectionPtr>> connecting_paths_map = net->getConnectionBetweenNodes(path1_node,path2_node_fake,diff_subpath_cost,
                                                                                                           black_list,net_time,search_in_subtree);

    if(pathSwitch_verbose_)
      ROS_YELLOW_STREAM("Net search in the subtree found "<<connecting_paths_map.size()<<" solutions in "<<(ros::WallTime::now()-tic_net).toSec()<<" s");

    /*To be sure map contains the connecting path computed*/
    double cost = 0.0;
    std::vector<ConnectionPtr> conns = subtree->getConnectionToNode(path2_node_fake);
    for(const ConnectionPtr& c: conns)
    {
      if(not c->getFlag(cost_updated_flag_,false))
      {
        c->setCost(ha_metrics_->cost(c->getParent()->getConfiguration(),c->getChild()->getConfiguration()));
        c->setFlag(cost_updated_flag_,true);
        flagged_connections_.push_back(c);
      }

      cost += c->getCost();
    }

    if(cost<diff_subpath_cost)
    {
      std::pair<double,std::vector<ConnectionPtr>> p;
      p.first = cost;
      p.second = conns;

      connecting_paths_map.insert(p);
    }

    if(not connecting_paths_map.empty())
    {
      if(findValidSolution(connecting_paths_map,diff_subpath_cost,connecting_path_conn,connecting_path_cost,number_of_candidates))
      {
        ConnectionPtr last_conn = connecting_path_conn.back();

        assert(last_conn != nullptr);
        assert(last_conn->getChild() == path2_node_fake);

        ConnectionPtr new_conn= std::make_shared<Connection>(last_conn->getParent(),path2_node,(path2_node->getParentConnectionsSize()>0));
        new_conn->setCost(last_conn->getCost());
        new_conn->setTimeCostUpdate(last_conn->getTimeCostUpdate());
        new_conn->add();

        assert(path2_node->getParentConnectionsSize() == 1);

        connecting_path_conn.back() = new_conn;
        connecting_path = std::make_shared<Path>(connecting_path_conn,ha_metrics_,checker_);
        connecting_path->setTree(tree_);

        assert(last_conn->isRecentlyChecked());
        assert(last_conn->getFlag(cost_updated_flag_,false));
        new_conn->setRecentlyChecked(last_conn->isRecentlyChecked());
        new_conn->setFlag(cost_updated_flag_,last_conn->getFlag(cost_updated_flag_,false));

        std::vector<ConnectionPtr>::iterator it = std::find(flagged_connections_.begin(),flagged_connections_.end(),last_conn);
        assert(it<flagged_connections_.end());
        *it = new_conn;

        last_conn->remove();

        assert(connecting_path->cost() == std::numeric_limits<double>::infinity() || connecting_path->cost()<diff_subpath_cost);

        if(pathSwitch_disp_)
        {
          disp_->changeConnectionSize({0.02,0.02,0.02});
          int connecting_path_id = disp_->displayPath(connecting_path,"pathplan",{1.0,0.4,0.0,1.0});
          disp_->nextButton("Displaying connecting_path..");
          disp_->clearMarker(connecting_path_id);
          disp_->defaultConnectionSize();
        }

        if(not((path2_node != tree_->getRoot() && path2_node->getParentConnectionsSize() == 1) ||
               (path2_node == tree_->getRoot() && path2_node->getParentConnectionsSize() == 0)))
        {
          ROS_INFO_STREAM("path2_node "<<path2_node);
          ROS_INFO_STREAM(*path2_node);

          ROS_INFO_STREAM("root "<<tree_->getRoot());
          ROS_INFO_STREAM(*tree_->getRoot());

          ROS_INFO_STREAM("fake root "<<paths_start_);
          ROS_INFO_STREAM(*paths_start_);

          throw std::exception();
        }

        subtree->purgeFromHere(path2_node_fake); //disconnect and remove the fake node
        assert(not tree_->isInTree(path2_node_fake));

        assert([&]() ->bool{
                 if(connecting_path->isValid())
                 {
                   return true;
                 }
                 else
                 {
                   ROS_BOLDRED_STREAM("connecting path "<<*connecting_path);
                   return false;
                 }
               }());

        return true;
      }
      else
      {
        if(pathSwitch_verbose_)
          ROS_YELLOW_STREAM("No free solutions found in the subtree");
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        ROS_YELLOW_STREAM("No solutions with cost less than diff_subpath_cost found in the subtree");
    }
  }

  /* If a solution was not found or the found solution was not free */
  connecting_path = nullptr;

  subtree->purgeFromHere(path2_node_fake); //disconnect and remove the fake node
  assert(not tree_->isInTree(path2_node_fake));

  return false;
}

bool MARSHA::findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double &cost2beat, std::vector<ConnectionPtr> &solution, double& cost, unsigned int &number_of_candidates, bool verbose)
{
  solution.clear();
  number_of_candidates = 0;

  if(not map.empty())
  {
    if(verbose)
      ROS_CYAN_STREAM("Map not empty, size "<<map.size());

    bool free;
    int i,size;
    double updated_cost;

    for(const std::pair<double,std::vector<ConnectionPtr>> &solution_pair:map)
    {
      if(solution_pair.first == std::numeric_limits<double>::infinity())
      {
        updated_cost = std::numeric_limits<double>::infinity();

        if(verbose)
          ROS_CYAN_STREAM("solution cost inf -> updated cost inf");

        assert([&]() ->bool{
                 i=0;
                 double updated_cost_check = 0.0;
                 size = solution_pair.second.size();

                 while(updated_cost_check<std::numeric_limits<double>::infinity() && i<size)
                 {
                   updated_cost_check += solution_pair.second.at(i)->getCost();
                   if(solution_pair.second.at(i)->getCost() == std::numeric_limits<double>::infinity() && not solution_pair.second.at(i)->isRecentlyChecked())
                   {
                     ROS_INFO("no recently checked");
                     return false;
                   }
                   i++;
                 }

                 if(updated_cost_check == updated_cost)
                 return true;
                 else
                 return false;
               }());
      }
      else //some connections are shared between solutions and during checking some of them can be set to infinity cost -> update cost
      {
        if(verbose)
          ROS_CYAN_STREAM("solution cost not inf, updating the cost..");

        i=0;
        updated_cost = 0.0;
        size = solution_pair.second.size();

        while(updated_cost<std::numeric_limits<double>::infinity() && i<size)
        {
          if(verbose)
            ROS_CYAN_STREAM("connection: "<<solution_pair.second.at(i)<<" cost: "<<solution_pair.second.at(i)->getCost());

          updated_cost += solution_pair.second.at(i)->getCost();
          assert([&]() ->bool{
                   if(solution_pair.second.at(i)->getCost() == std::numeric_limits<double>::infinity())
                   {
                     if(solution_pair.second.at(i)->isRecentlyChecked())
                     {
                       return true;
                     }
                     else
                     {
                       ROS_INFO("cost inf but not recently checked");
                       return false;
                     }
                   }
                   else
                   {
                     if(solution_pair.second.at(i)->getFlag(cost_updated_flag_,false))
                     {
                       if(std::find(flagged_connections_.begin(),flagged_connections_.end(),solution_pair.second.at(i))>=flagged_connections_.end())
                       {
                         ROS_BOLDRED_STREAM("connection "<<*solution_pair.second.at(i));
                         return false;
                       }

                       return true;
                     }
                     else
                     {
                       ROS_BOLDRED_STREAM("connection "<<*solution_pair.second.at(i));
                       return false;
                     }
                   }
                 }());
          i++;
        }
      }

      if(verbose)
        ROS_CYAN_STREAM("updated cost: "<<updated_cost<<" cost2beat: "<<cost2beat);

      if(updated_cost<cost2beat)
      {
        if(verbose)
          ROS_CYAN_STREAM("new candidate solution");

        number_of_candidates++;

        free = true;
        for(const ConnectionPtr& conn: solution_pair.second)
        {
          if(not conn->isRecentlyChecked())
          {
            conn->setRecentlyChecked(true);
            flagged_connections_.push_back(conn);

            if(verbose)
              ROS_CYAN_STREAM("conn "<<conn<<" not recently checked");

            assert(conn->getCost() != std::numeric_limits<double>::infinity());

            if(not checker_->checkConnection(conn))
            {
              free = false;

              /* Save the invalid connection */
              invalid_connection_ptr invalid_conn = std::make_shared<invalid_connection>();
              invalid_conn->connection = conn;
              invalid_conn->cost = conn->getCost();
              invalid_connections_.push_back(invalid_conn);

              /* Set the cost equal to infinity */
              conn->setCost(std::numeric_limits<double>::infinity());

              if(verbose)
                ROS_INFO_STREAM("conn "<<conn<<" obstructed!");

              break;
            }
          }
          else
          {
            if(verbose)
              ROS_CYAN_STREAM("conn "<<conn<<" already checked, cost: "<<conn->getCost());

            assert(std::find(flagged_connections_.begin(),flagged_connections_.end(),conn)<flagged_connections_.end());

            if(conn->getCost() == std::numeric_limits<double>::infinity()) //it should not happen..
            {
              assert(0);
              free = false;
              break;
            }
          }
        }

        if(free)
        {
          if(verbose)
            ROS_CYAN_STREAM("Solution free, cost: "<<updated_cost);

          solution = solution_pair.second;
          cost = updated_cost;

          //updated_cost != solution_pair.first can happen only with updated_cost == infinity and solution_pair.first not
          assert(cost < std::numeric_limits<double>::infinity());
          assert([&]() ->bool{
                   if(std::abs(updated_cost-solution_pair.first)>1.04)
                   {
                     ROS_INFO_STREAM("updated cost "<<updated_cost<<" solution_pair.first "<<solution_pair.first <<" NET_ERROR_TOLERANCE "<<NET_ERROR_TOLERANCE<< " err "<<std::abs(solution_pair.first-updated_cost));
                     return false;
                   }
                   return true;
                 }());

          return true;
        }
      }
      else
      {
        if(verbose)
          ROS_CYAN_STREAM("not a candidate solution");

        if(updated_cost<std::numeric_limits<double>::infinity()) //solutions ordered by cost in the map, so, if this solution is not obstructed and it is worst than cost2beat, no better solutions exist (subsequent solutions will have higher cost or cost infinite)
        {
          if(verbose)
            ROS_CYAN_STREAM("update cost not infinite, no better solutions available -> exit");

          return false;
        }
      }

      if(verbose)
        ROS_CYAN_STREAM("-------------------------");
    }
  }
  return false;
}

void MARSHA::setCurrentPath(const PathPtr& path)
{
  MARS::setCurrentPath(path);
  setMetricsHA(ha_metrics_);
}
void MARSHA::setOtherPaths(const std::vector<PathPtr> &other_paths, const bool merge_tree)
{
  MARS::setOtherPaths(other_paths,merge_tree);
  setMetricsHA(ha_metrics_);
}

bool MARSHA::replan()
{
  replanned_path_ = nullptr;

  assert(metrics_ == ha_metrics_);
  if(metrics_ == nullptr)
  {
    ROS_ERROR("metrics are not defined!");
    return false;
  }

  return MARS::replan();
}
}
