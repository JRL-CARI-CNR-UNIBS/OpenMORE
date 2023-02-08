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
    metrics_   = nullptr;
    ha_metrics_ = nullptr;
  }
  else
    setMetricsHA(ha_metrics);

  full_net_search_ = false;
}

bool MARSHA::setObstaclesPosition(const Eigen::Matrix<double,3,Eigen::Dynamic>& obstacles_positions)
{
  if(ha_metrics_ != nullptr)
  {
    ha_metrics_->setObstaclesPosition(obstacles_positions);
    return true;
  }
  return false;
}

bool MARSHA::addObstaclePosition(const Eigen::Vector3d& obstacle_position)
{
  if(ha_metrics_ != nullptr)
  {
    ha_metrics_->addObstaclePosition(obstacle_position);
    return true;
  }
  return false;
}

void MARSHA::setMetricsHA(const LengthPenaltyMetricsPtr& ha_metrics)
{
  ha_metrics_ = ha_metrics ;
  metrics_    = ha_metrics_;

  net_->setMetrics(ha_metrics_);
  tree_->setMetrics(ha_metrics_);
  solver_->setMetrics(ha_metrics_);

  assert(current_path_->getMetrics() == ha_metrics_);
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
    ROS_BLUE_STREAM("Searching for an already existing solution in the subtree..");

  ros::WallTime tic_search = ros::WallTime::now();

  bool search_in_subtree = true;
  double net_time = maxSolverTime(tic,tic_cycle);

  NetPtr net = std::make_shared<Net>(subtree);
  std::multimap<double,std::vector<ConnectionPtr>> already_existing_solutions_map = net->getConnectionBetweenNodes(path1_node,path2_node,diff_subpath_cost,
                                                                                                                   black_list,net_time,search_in_subtree);
  double time_search = (ros::WallTime::now()-tic_search).toSec();

  if(pathSwitch_verbose_)
    ROS_BLUE_STREAM("In the subtree exist "<< already_existing_solutions_map.size() <<" paths to path2_node (time to search "<<time_search<<" seconds)");

  tic_search = ros::WallTime::now();
  unsigned int number_of_candidates = 0;
  double already_existing_solution_cost;
  std::vector<ConnectionPtr> already_existing_solution_conn;
  if(findValidSolution(already_existing_solutions_map,diff_subpath_cost,already_existing_solution_conn,
                       already_existing_solution_cost,number_of_candidates,false))
  {
    connecting_path = std::make_shared<Path>(already_existing_solution_conn,metrics_,checker_);
    connecting_path->setTree(tree_);
    quickly_solved = true;

    assert([&]() ->bool{
             for(const ConnectionPtr& c:already_existing_solution_conn)
             {
               if(c->getTimeCostUpdate()<starting_time_)
               {
                 return false;
               }
             }
             return true;
           }());
    assert(already_existing_solution_cost == connecting_path->cost());

    if(pathSwitch_verbose_)
      ROS_BLUE_STREAM("A solution with cost "<< already_existing_solution_cost<<" has been found in the subtree in "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds! Making it a solution of the subtree..");

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
        ROS_BLUE_STREAM(number_of_candidates<< " candidate solutions found in the subtree but no one was free (check time "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds)");
      else
        ROS_BLUE_STREAM("No candidate solutions found in the subtree (search time "<<(ros::WallTime::now()-tic_search).toSec()<<" seconds)");
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

    connecting_path = nullptr;
    subtree_nodes = subtree->getNodesConst(); //nodes already in the subtree

    available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();
    if(pathSwitch_verbose_)
      ROS_BLUE_STREAM("Searching for a connecting path...max time: "<<available_search_time);

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
        ROS_BLUE_STREAM("Direct connection NOT found (time "<<(toc_solver-tic_solver).toSec()<<" s)");
        ROS_BLUE_STREAM("Solving...max time: "<<available_search_time);
      }

      tic_solver = ros::WallTime::now();
      solver_has_solved = solver_->solve(connecting_path,10000,available_search_time*0.85);
      toc_solver = ros::WallTime::now();
    }

    if(solver_has_solved)
    {
      if(pathSwitch_verbose_)
        ROS_BLUE_STREAM("Solved in "<<(toc_solver-tic_solver).toSec()<<" s (direct connection to goal: "<<quickly_solved<<")");

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
          if(not subtree_valid || c->isRecentlyChecked())
            continue;

          if(not checker_->checkConnection(c))
          {
            invalid_connection invalid_conn;
            invalid_conn.connection = c;
            invalid_conn.cost = c->getCost();
            invalid_connections_.push_back(invalid_conn);

            c->setCost(std::numeric_limits<double>::infinity());

            obstructed_connection = c;
            subtree_valid = false;
            quickly_solved = false;
          }
          else
            c->setCost(metrics_->cost(c->getParent(),c->getChild()));

          c->setRecentlyChecked(true);
          checked_connections_.push_back(c);
        }
        else //do not re-check connections just added to the subtree
        {
          c->setRecentlyChecked(true);
          checked_connections_.push_back(c);

          assert([&]() ->bool{
                   if(c->getTimeCostUpdate()<starting_time_)
                   {
                     return false
                   }
                   return true;
                 }());
        }
      }

      if(subtree_valid)
      {
        valid_connecting_path_found = true;
        break;
      }
      else
      {
        subtree->purgeFromHere(path2_node_fake);

        if(obstructed_connection != nullptr)
          subtree->hideFromSubtree(obstructed_connection->getChild());

        if(pathSwitch_verbose_)
          ROS_BLUE_STREAM("Lazy check detected that subtree was not valid, compute again..");
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        ROS_BLUE_STREAM("Not solved, time: "<<(toc_solver-tic_solver).toSec());

      break;
    }

    available_search_time = solver_time-(ros::WallTime::now()-tic_before_search).toSec();
  }

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
    std::multimap<double,std::vector<ConnectionPtr>> connecting_paths_map = net->getConnectionBetweenNodes(path1_node,path2_node_fake,diff_subpath_cost,
                                                                                                           black_list,net_time,search_in_subtree);

    if(pathSwitch_verbose_)
      ROS_BLUE_STREAM("Net search in the subtree found "<<connecting_paths_map.size()<<" solutions in "<<(ros::WallTime::now()-tic_net).toSec()<<" s");

    /*To be sure map contains the connecting path computed*/
    double cost = 0.0;
    std::vector<ConnectionPtr> conns = subtree->getConnectionToNode(path2_node_fake);
    for(const ConnectionPtr& c: conns)
      cost += c->getCost();

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
        new_conn->add();

        assert([&]() ->bool{
                 if(last_conn->getTimeCostUpdate()<starting_time_)
                 {
                   return false;
                 }

                 return true;
               }());

        assert(path2_node->getParentConnectionsSize() == 1);

        connecting_path_conn.back() = new_conn;
        connecting_path = std::make_shared<Path>(connecting_path_conn,metrics_,checker_);
        connecting_path->setTree(tree_);

        assert([&]() ->bool{
                 for(const ConnectionPtr& c:connecting_path_conn)
                 {
                   if(c->getTimeCostUpdate()<starting_time_)
                   {
                     return false;
                   }
                 }
                 return true;
               }());

        last_conn->remove();

        std::vector<ConnectionPtr>::iterator it = std::find(checked_connections_.begin(),checked_connections_.end(),last_conn);
        assert(it<checked_connections_.end());
        *it = new_conn;

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

        return true;
      }
      else
      {
        if(pathSwitch_verbose_)
          ROS_BLUE_STREAM("No free solutions found in the subtree");
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        ROS_BLUE_STREAM("No solutions with cost less than diff_subpath_cost found in the subtree");
    }
  }

  /* If a solution was not found or the found solution was not free */
  connecting_path = nullptr;

  subtree->purgeFromHere(path2_node_fake); //disconnect and remove the fake node
  assert(not tree_->isInTree(path2_node_fake));

  return false;
}

bool MARSHA::replan()
{
  starting_time_ = ros::WallTime::now().toSec();
  net_->reEvaluateCostOlderThan(starting_time_);

  //Update the internal time of paths' connections to avoid to recompute their costs during algorithm execution
  std::vector<PathPtr> paths = other_paths_;
  paths.push_back(current_path_);

  for(const PathPtr& p: paths)
  {
    for(ConnectionPtr& c:p->getConnections())
      c->setCost(c->getCost()); //setCost updates the internal time
  }

  assert(metrics_ == ha_metrics_);
  if(metrics_ == nullptr)
  {
    ROS_ERROR("metrics are not defined!");
    return false;
  }

  bool path_changed = MARS::replan();

  assert([&]() ->bool{
           if(path_changed)
           {
             for(const ConnectionPtr& c:replanned_path_->getConnectionsConst())
             {
               if(c->getTimeCostUpdate()<starting_time_)
               return false;
             }
           }
           return true;
         }());
}
}
