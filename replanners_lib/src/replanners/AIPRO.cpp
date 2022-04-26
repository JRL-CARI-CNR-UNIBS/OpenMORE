#include "replanners_lib/replanners/AIPRO.h"

namespace pathplan
{
AIPRO::AIPRO(const Eigen::VectorXd& current_configuration,
             const PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  tree_ = current_path_->getTree();
  net_ = std::make_shared<Net>(tree_);

  copyTreeRoot();

  available_time_ =  std::numeric_limits<double>::infinity();
  pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  time_percentage_variability_ = TIME_PERCENTAGE_VARIABILITY;

  an_obstacle_ = false;

  informedOnlineReplanning_disp_ = false;
  pathSwitch_disp_ = false;

  informedOnlineReplanning_verbose_ = false;
  pathSwitch_verbose_ = false;
}

AIPRO::AIPRO(const Eigen::VectorXd& current_configuration,
             const PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver,
             std::vector<PathPtr> &other_paths): AIPRO(current_configuration,current_path,max_time,solver)
{
  setOtherPaths(other_paths);
}

void AIPRO::copyTreeRoot()
{
  // Net stops working when encounters the tree root, so doesn't allow to find a replanned path which pass through the start
  //So, a copy of the root (=start) is created and all the paths will start from this node and not from the root.
  paths_start_ = tree_->getRoot();
  assert(tree_->getRoot() == current_path_->getConnections().front()->getParent());
  NodePtr new_tree_root =std::make_shared<Node>(paths_start_->getConfiguration());

  ConnectionPtr conn = std::make_shared<Connection>(paths_start_,new_tree_root,false);
  conn->setCost(0.0);
  conn->add();

  tree_->addNode(new_tree_root);
  if(not tree_->changeRoot(new_tree_root))
  {
    ROS_ERROR("The root can be moved to its copy");
    assert(0);
  }
}


bool AIPRO::mergePathToTree(PathPtr &path)
{
  TreePtr path_tree = path->getTree();
  NodePtr path_goal = path->getConnections().back()->getChild();

  for(const ConnectionPtr& conn:path->getConnections())
    assert(not conn->isNet());

  if(tree_ == path_tree)
    return true;

  assert(goal_node_->parent_connections_.size() == 1);

  //Merging the root
  if(not tree_)
  {
    if(path_tree)
    {
      tree_ = path_tree;
      copyTreeRoot();
    }
    else
    {
      std::vector<ConnectionPtr> conns;
      conns = current_path_->getConnections();
      conns.insert(conns.end(),path->getConnectionsConst().begin(),path->getConnectionsConst().end());

      double max_dist = 0;
      for(const ConnectionPtr& conn:conns)
      {
        if(conn->norm()>max_dist)
          max_dist = conn->norm();
      }

      tree_ = std::make_shared<Tree>(path->getNodes().front(),max_dist,checker_,metrics_);
      tree_->addBranch(path->getConnections());
      copyTreeRoot();
    }

    NodePtr current_path_start = current_path_->getConnections().front()->getParent();
    if(paths_start_->getConfiguration() == current_path_start->getConfiguration())
    {
      if(paths_start_ != current_path_start)
      {
        ConnectionPtr conn_child = current_path_->getConnections().front();
        assert(not conn_child->isNet());

        ConnectionPtr conn = std::make_shared<Connection>(paths_start_,conn_child->getChild());
        conn->setCost(conn_child->getCost());
        conn->add();

        std::vector<ConnectionPtr> connections = current_path_->getConnections();
        connections.front() = conn;
        current_path_->setConnections(connections);

        conn_child->remove();
      }

      tree_->addBranch(current_path_->getConnections());

      current_path_->setTree(tree_);
      path->setTree(tree_);
    }
    else
    {
      ROS_ERROR("Path has a starting node different from the tree root (!tree_)");
      tree_.reset();
      net_->getTree().reset();

      return false;
    }
  }
  else
  {
    NodePtr path_start = path->getConnections().front()->getParent();
    if(paths_start_->getConfiguration() == path_start->getConfiguration())
    {
      if(paths_start_ != path_start)
      {
        std::vector<ConnectionPtr> connections;
        if(path_tree)
        {
          assert(path_start == path_tree->getRoot());

          path_tree->addNode(paths_start_);
          path_tree->changeRoot(path_goal);

          NodePtr root = tree_->getRoot();
          tree_->changeRoot(paths_start_);

          ConnectionPtr conn;
          NodePtr child, parent;
          std::vector<ConnectionPtr> child_connections = path_start->child_connections_;
          for(const ConnectionPtr& child_conn:child_connections)
          {
            assert(not child_conn->isNet());

            child = child_conn->getChild();
            conn = std::make_shared<Connection>(paths_start_,child);
            conn->setCost(child_conn->getCost());
            conn->add();
          }

          std::vector<ConnectionPtr> parent_connections = path_start->parent_connections_;
          for(const ConnectionPtr& parent_conn:parent_connections)
          {
            assert(not parent_conn->isNet());

            parent = parent_conn->getParent();
            conn = std::make_shared<Connection>(parent,paths_start_);
            conn->setCost(parent_conn->getCost());
            conn->add();
          }

          if(not path_tree->changeRoot(paths_start_))
            assert(0);

          tree_->changeRoot(root);
          path_tree->removeNode(path_start);

          connections = path_tree->getConnectionToNode(path_goal);
        }
        else
        {
          ConnectionPtr conn_child = path->getConnections().front();
          assert(not conn_child->isNet());

          ConnectionPtr conn = std::make_shared<Connection>(paths_start_,conn_child->getChild());
          conn->setCost(conn_child->getCost());
          conn->add();

          conn_child->remove();

          connections = path->getConnections();
          connections.front() = conn;
        }

        path->setConnections(connections);
      }

      if(path_tree)
      {
        tree_->addTree(path_tree);
        path_tree = tree_;
      }
      else
        tree_->addBranch(path->getConnections());

      current_path_->setTree(tree_);
      path->setTree(tree_);
    }
    else
    {
      ROS_ERROR("Path has a starting node different from the tree root (tree_)");
      return false;
    }
  }

  //Merging the goal
  std::vector<ConnectionPtr> path_conns = path->getConnections();
  ConnectionPtr conn2delete = path->getConnections().back();

  assert(goal_node_->parent_connections_.size() == 1);

  ConnectionPtr new_goal_conn;
  (goal_node_->parent_connections_.empty())?
        (new_goal_conn = std::make_shared<Connection>(conn2delete->getParent(),goal_node_,false)):
        (new_goal_conn= std::make_shared<Connection>(conn2delete->getParent(),goal_node_,true));

  new_goal_conn->setCost(conn2delete->getCost());
  new_goal_conn->add();

  conn2delete->remove();

  path_conns.back() = new_goal_conn;
  path->setConnections(path_conns);

  tree_->removeNode(path_goal);
  net_->setTree(tree_);

  assert(goal_node_->parent_connections_.size() == 1);

  return true;
}

void AIPRO::clearInvalidConnections()
{
  /*Set the cost of subtree connections equal to their default value. In previous call to AIPRO replanner
    the cost of some subtrees connections has been set equal to infinite. These connections usually are
    not checked because not part of a path. So, reset their cost. The connections of the paths are excluded
    because their cost is updated by an external collision checker*/

  std::vector<ConnectionPtr> connections = current_path_->getConnections();
  for(const PathPtr& p:other_paths_)
    connections.insert(connections.end(),p->getConnectionsConst().begin(),p->getConnectionsConst().end());

  for(invalid_connection& invalid_conn:invalid_connections_)
  {
    if(std::find(connections.begin(),connections.end(),invalid_conn.connection)>=connections.end())
      invalid_conn.connection->setCost(invalid_conn.cost);
  }

  invalid_connections_.clear();
}

std::vector<PathPtr> AIPRO::addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path)
{
  std::vector<PathPtr> reset_other_paths;
  admissible_current_path = nullptr;

  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    if(current_path_->getConnections().back()->getCost() == std::numeric_limits<double>::infinity())
      return other_paths_;
    else
    {
      int z = current_path_->getConnectionsSize()-2;  //penultimate connection (last connection is at end-1)
      ConnectionPtr conn;

      while(z>=idx_current_conn) //to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
      {
        conn = current_path_->getConnections().at(z);
        if(conn->getCost() == std::numeric_limits<double>::infinity())
        {
          admissible_current_path = current_path_->getSubpathFromNode(conn->getChild());
          break;
        }
        z -= 1;
      }
    }

    // Adding the savable subpath of the current_path to the set of available paths
    if(admissible_current_path)
      reset_other_paths.push_back(admissible_current_path);

    reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());

    return reset_other_paths;
  }
  else
  {
    if(idx_current_conn<current_path_->getConnectionsSize()-1)
    {
      admissible_current_path = current_path_->getSubpathFromNode(current_path_->getConnections().at(idx_current_conn)->getChild());
      reset_other_paths.push_back(admissible_current_path);
      reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
    }
    else
      reset_other_paths = other_paths_;

    return reset_other_paths;
  }
}

std::vector<node_and_path> AIPRO::sortNodesOnDistance(const NodePtr& start_node)
{
  std::multimap<double,node_and_path> node_and_path_map;
  std::vector<node_and_path> goals, ordered_goals;

  std::vector<NodePtr> nodes2check; //elimina

  bool goal_added = false;
  for(const PathPtr& p:admissible_other_paths_)
  {
    std::vector<NodePtr> nodes = p->getNodes();
    if(goal_added)
      nodes.pop_back();

    for(const NodePtr& n:nodes)
    {
      if(n->getConfiguration() == start_node->getConfiguration())
        continue;

      node_and_path n_p;
      n_p.node = n;
      n_p.path = p;

      //elimina
      if(std::find(nodes2check.begin(),nodes2check.end(),n)<nodes2check.end() && not (n->getConfiguration() == paths_start_->getConfiguration()))
      {
        ROS_INFO_STREAM("nodo multiplo: "<<*n);

        for(unsigned int i=0;i<admissible_other_paths_.size();i++)
        {
          if(admissible_other_paths_.at(i) == p)
          {
            ROS_INFO_STREAM("il nodo appartiene al path numero: "<<i);
            break;
          }
        }

        for(unsigned int j=0;j<admissible_other_paths_.size();j++)
        {
          ROS_INFO_STREAM("PATH NUMBER: "<<j);
          for(const NodePtr nn:admissible_other_paths_.at(j)->getNodes())
            ROS_INFO_STREAM("n: "<<nn->getConfiguration().transpose()<<" "<<nn);
        }
        //assert(0);
      }
      else
        nodes2check.push_back(n);
      //

      goals.push_back(n_p);
    }

    goal_added = true;
  }

  double distance;
  for(const node_and_path& n_p:goals)
  {
    distance = metrics_->utopia(start_node->getConfiguration(),n_p.node->getConfiguration());
    node_and_path_map.insert(std::pair<double,node_and_path>(distance,n_p));
  }

  for(const std::pair<double,node_and_path> &p: node_and_path_map)
    ordered_goals.push_back(p.second);

  return ordered_goals;
}

std::vector<NodePtr> AIPRO::startNodes(const std::vector<ConnectionPtr>& subpath1_conn)
{
  for(const ConnectionPtr &conn:subpath1_conn) //elimina
  {
    if(conn->getParent()->getConfiguration() == conn->getChild()->getConfiguration())
    {
      for(const ConnectionPtr &conn:subpath1_conn)
        ROS_INFO_STREAM(*conn);

      assert(0);
    }
  }

  std::vector<NodePtr> start_node_vector;

  if((subpath1_conn.front()->getCost() == std::numeric_limits<double>::infinity()))
  {
    /*if the current conf is obstructed the replanning will start from the current node*/

    NodePtr current_node = subpath1_conn.front()->getParent();

    if(not current_node->getAnalyzed())
      start_node_vector.push_back(current_node);
  }
  else
  {
    /*if the current connection is free, all the nodes between the current child to the parent
     *of the connection obstructed are considered as starting points for the replanning*/

    for(const ConnectionPtr& conn:subpath1_conn)
    {
      if(conn == subpath1_conn.front())
        continue;
      else if(conn == subpath1_conn.back())
      {
        /*if the path is free, you can consider all the nodes but it is useless to consider
         *the last one before the goal (it is already connected to the goal with a straight line) */

        if(conn->getCost() ==  std::numeric_limits<double>::infinity() && (not conn->getParent()->getAnalyzed()))
          start_node_vector.push_back(conn->getParent());
      }
      else
      {
        if(not conn->getParent()->getAnalyzed())
          start_node_vector.push_back(conn->getParent());

        if(conn->getCost() ==  std::numeric_limits<double>::infinity())
          break;
      }
    }
  }

  if(start_node_vector.size()>1) //elimina
  {
    for(unsigned int i=0;i<start_node_vector.size()-1;i++)
    {
      if(start_node_vector.at(i)->getConfiguration() == start_node_vector.at(i+1)->getConfiguration())
      {
        for(const NodePtr& n:start_node_vector)
          ROS_INFO_STREAM("NODE PS: "<<*n);
        assert(0);
      }
    }
  }

  return start_node_vector;
}

bool AIPRO::findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double &cost2beat, std::vector<ConnectionPtr> &solution, double& cost)
{
  unsigned int number_of_candidates = 0;
  return findValidSolution(map,cost2beat,solution,cost,number_of_candidates);
}

bool AIPRO::findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double &cost2beat, std::vector<ConnectionPtr> &solution, double& cost, unsigned int &number_of_candidates)
{
  solution.clear();
  number_of_candidates = 0;

  if(not map.empty())
  {
    bool free;
    for(const std::pair<double,std::vector<ConnectionPtr>> &solution_pair:map)
    {
      if(solution_pair.first<cost2beat)
      {
        number_of_candidates++;

        free = true;
        for(const ConnectionPtr& conn: solution_pair.second)
        {
          if(not conn->isRecentlyChecked())
          {
            conn->setRecentlyChecked(true);
            checked_connections_.push_back(conn);

            if(not checker_->checkConnection(conn))
            {
              free = false;

              //Save the invalid connection
              invalid_connection invalid_conn;
              invalid_conn.connection = conn;
              invalid_conn.cost = conn->getCost();
              invalid_connections_.push_back(invalid_conn);

              //Set the cost equal to infinity
              conn->setCost(std::numeric_limits<double>::infinity());

              break;
            }
          }
          else
          {
            if(conn->getCost() == std::numeric_limits<double>::infinity())
            {
              free = false;
              break;
            }
          }
        }

        if(free)
        {
          solution = solution_pair.second;
          cost = solution_pair.first;

          return true;
        }
      }
    }
  }

  return false;
}


PathPtr AIPRO::bestExistingSolution(const PathPtr& current_solution)
{
  std::multimap<double,std::vector<ConnectionPtr>> tmp_map;
  PathPtr solution;

  NodePtr current_node = current_solution->getConnections().front()->getParent();
  double best_cost = current_solution->cost();

  tmp_map = net_->getConnectionBetweenNodes(current_node,goal_node_);

  if(informedOnlineReplanning_verbose_)
    ROS_INFO_STREAM(tmp_map.size()<<" solutions already exist!");

  if(informedOnlineReplanning_disp_)
  {
    disp_->changeNodeSize({0.025,0.025,0.025});
    disp_->changeConnectionSize({0.025,0.025,0.025});

    int id;
    int n_sol = 0;
    for(const std::pair<double,std::vector<ConnectionPtr>> &solution_pair:tmp_map)
    {
      PathPtr disp_path = std::make_shared<Path>(solution_pair.second,metrics_,checker_);

      n_sol++;
      id = disp_->displayPath(disp_path);
      disp_->nextButton("Displaying the "+std::to_string(n_sol)+"° existing solution (cost " + std::to_string(disp_path->cost())+")");
      disp_->clearMarker(id);
    }
    disp_->defaultNodeSize();
    disp_->defaultConnectionSize();
  }

  double new_cost;
  std::vector<ConnectionPtr> solution_conns;
  findValidSolution(tmp_map,best_cost,solution_conns,new_cost)?
        (solution = std::make_shared<Path>(solution_conns,metrics_,checker_)):
        (solution = current_solution);

  solution->setTree(tree_);
  return solution;
}

void AIPRO::simplifyAdmissibleOtherPaths(const PathPtr& current_solution_path, const NodePtr& start_node, const std::vector<PathPtr>& reset_other_paths)
{
  PathPtr path_connected_to = nullptr;
  int number_path_connected_to = -1;

  std::vector<ConnectionPtr> current_solution_conn = current_solution_path->getConnections();
  ConnectionPtr last_conn_current_path = current_solution_conn.back();
  ConnectionPtr last_conn;

  for(unsigned int i=0;i<admissible_other_paths_.size();i++)
  {
    last_conn = admissible_other_paths_.at(i)->getConnections().back();
    if(last_conn == last_conn_current_path)
    {
      path_connected_to = admissible_other_paths_.at(i);
      number_path_connected_to = i;

      break;
    }
  }

  admissible_other_paths_.clear();
  admissible_other_paths_ = reset_other_paths;

  if(path_connected_to)
  {
    std::vector<NodePtr> path2_nodes = path_connected_to->getNodes();
    assert(start_node != path2_nodes.back());
    path2_nodes.pop_back();  // removing the goal from the vector

    std::vector<NodePtr>::iterator it = std::find(path2_nodes.begin(),path2_nodes.end(),start_node);
    if(it < path2_nodes.end())
      admissible_other_paths_.at(number_path_connected_to) = path_connected_to->getSubpathFromNode(*it);
  }
}

double AIPRO::maxSolverTime(const ros::WallTime& tic, const ros::WallTime& tic_cycle)
{
  double time;
  ros::WallTime toc = ros::WallTime::now();

  if(pathSwitch_disp_)
    time = std::numeric_limits<double>::infinity();
  else if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    time = pathSwitch_max_time_-(toc-tic).toSec(); //when there is an obstacle or when the cycle time mean has not been defined yets
  else
    time = (2-time_percentage_variability_)*pathSwitch_cycle_time_mean_-(toc-tic_cycle).toSec();

  return time;
}

void AIPRO::optimizePath(PathPtr& path, const double& max_time)
{
  ros::WallTime tic_opt = ros::WallTime::now();
  path->warp(0.1,max_time);
  ros::WallTime toc_opt = ros::WallTime::now();

  if(pathSwitch_verbose_)
    ROS_INFO_STREAM("max opt time: "<<max_time<<" used time: "<<(toc_opt-tic_opt).toSec());
}

bool AIPRO::simplifyReplannedPath(const double& distance)
{
  bool simplify1 = false;
  bool simplify2 = false;
  bool simplified = false;

  do
  {
    simplify1 = replanned_path_->removeNodes();
    simplify2 = replanned_path_->simplify(distance);

    if(simplify1 || simplify2)
      simplified = true;
  }
  while(simplify1 || simplify2);

  return simplified;
}

void AIPRO::convertToSubtreeSolution(const PathPtr& net_solution, const std::vector<NodePtr>& black_nodes)
{
  std::vector<ConnectionPtr> connections = net_solution->getConnections();
  connections.pop_back(); //the last connection must remain a net connection

  NodePtr node;
  for(ConnectionPtr& conn:connections)
  {
    if(conn->isNet())
    {
      node = conn->getChild();
      assert(std::find(black_nodes.begin(),black_nodes.end(),node)>=black_nodes.end());

      if(not node->switchParentConnection(conn))
        assert(0);
    }
  }
}

bool AIPRO::computeConnectingPath(const NodePtr& path1_node, const NodePtr& path2_node, const double& diff_subpath_cost, const PathPtr& current_solution, const ros::WallTime& tic, const ros::WallTime& tic_cycle, PathPtr& connecting_path, bool& quickly_solved)
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

  SubtreePtr subtree = pathplan::Subtree::createSubtree(tree_,path1_node,
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

  //elimina
  std::vector<NodePtr> nn;
  if(replanned_path_ != current_path_ && replanned_path_ != nullptr)
  {
    nn = replanned_path_->getNodes();
    black_list.insert(black_list.end(),nn.begin(),nn.end());
  }
  for(const NodePtr& n:black_list)
  {
    if(n != subtree->getRoot())
    {
      if(subtree->isInTree(n))
        assert(0);
    }
  }
  //

  /* Search for an already existing solution between path1_node and path2_node */
  if(pathSwitch_verbose_)
    ROS_INFO("Searching for an already existing solution in the subtree..");

  NetPtr net = std::make_shared<Net>(subtree);
  std::multimap<double,std::vector<ConnectionPtr>> already_existing_solutions_map = net->getConnectionBetweenNodes(path1_node,path2_node,black_list);

  if(pathSwitch_verbose_)
    ROS_INFO("In the subtree exist %u paths to path2_node",already_existing_solutions_map.size());

  unsigned int number_of_candidates = 0;
  double already_existing_solution_cost;
  std::vector<ConnectionPtr> already_existing_solution_conn;
  if(findValidSolution(already_existing_solutions_map,diff_subpath_cost,already_existing_solution_conn,already_existing_solution_cost,number_of_candidates))
  {
    connecting_path = std::make_shared<Path>(already_existing_solution_conn,metrics_,checker_);
    connecting_path->setTree(tree_);
    quickly_solved = true;

    assert(already_existing_solution_cost == connecting_path->cost());

    if(pathSwitch_verbose_)
      ROS_INFO("A solution with cost %f has been found in the subtree! Making it a solution of the subtree..",already_existing_solution_cost);

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
        ROS_INFO("%u candidate solutions found in the subtree but no one was free",number_of_candidates);
      else
        ROS_INFO("No candidate solutions found in the subtree");
    }
  }

  //elimina
  std::vector<NodePtr> init_nodes = subtree->getNodes();
  int count = 0;
  for(const NodePtr& n:init_nodes)
  {
    if(n->getConfiguration()==subtree->getRoot()->getConfiguration())
      count++;
  }
  if(count!=1)
  {
    for(const NodePtr& n:init_nodes)
      ROS_INFO_STREAM("n: "<<n->getConfiguration().transpose());

    assert(0);
  }
  // ///


  /* If no solutions already exist, search for a new one. The ellipsoide determined
   * by diff_subpath_cost is used to sample the space. Outside of this ellipsoid,
   * the nodes create an inconvenient connecting_path */

  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node->getConfiguration(),
                                                         path2_node->getConfiguration(),
                                                         lb_, ub_,diff_subpath_cost);
  solver_->setSampler(sampler);
  solver_->resetProblem();
  solver_->addStart(path1_node);
  solver_->setStartTree(subtree);

  double solver_time = maxSolverTime(tic,tic_cycle);

  if(pathSwitch_verbose_)
    ROS_INFO_STREAM("Searching for a direct connection...max time: "<<solver_time);

  NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());
  ROS_INFO_STREAM("PATH2_NODE_FAKE: "<<path2_node_fake->getConfiguration().transpose()<<" "<<path2_node_fake); //elimina
  ros::WallTime tic_directConnection = ros::WallTime::now();
  solver_->addGoal(path2_node_fake,solver_time);
  ros::WallTime toc_directConnection = ros::WallTime::now();

  quickly_solved = solver_->solved();

  if(pathSwitch_verbose_)
  {
    if(quickly_solved)
      ROS_INFO("Quickly solved->direct connection found");
    else
      ROS_INFO("Direct connection NOT found, not quickly solved");

    ROS_INFO_STREAM("Time to directly connect: "<<(toc_directConnection-tic_directConnection).toSec());
  }

  bool solver_has_solved = false;
  ros::WallTime tic_solver;
  ros::WallTime toc_solver;

  assert(quickly_solved == solver_->solved());

  if(solver_->solved())
  {
    connecting_path = solver_->getSolution();
    solver_has_solved = true;
  }
  else
  {
    solver_time = maxSolverTime(tic,tic_cycle);

    if(pathSwitch_verbose_)
      ROS_INFO_STREAM("Solving...max time: "<<solver_time);

    tic_solver = ros::WallTime::now();
    solver_has_solved = solver_->solve(connecting_path,10000,solver_time);
    toc_solver = ros::WallTime::now();

    if(pathSwitch_verbose_)
    {
      if(solver_has_solved)
        ROS_INFO_STREAM("Solved in time: "<<(toc_solver-tic_solver).toSec());
      else
        ROS_INFO_STREAM("Not solved, time: "<<(toc_solver-tic_solver).toSec());
    }
  }

  if(solver_has_solved)
  {
    /* Search for the best solution in the subtree which connects path1_node to path2_node_fake */
    number_of_candidates = 0;
    double connecting_path_cost;
    std::vector<ConnectionPtr> connecting_path_conn;
    std::multimap<double,std::vector<ConnectionPtr>> connecting_paths_map = net->getConnectionBetweenNodes(path1_node,path2_node_fake,black_list);

    assert(connecting_paths_map.size()>0);

    if(findValidSolution(connecting_paths_map,diff_subpath_cost,connecting_path_conn,connecting_path_cost,number_of_candidates))
    {
      connecting_path = std::make_shared<Path>(connecting_path_conn,metrics_,checker_);
      connecting_path->setTree(tree_);

      ConnectionPtr last_conn = connecting_path_conn.back();
      assert(last_conn != nullptr);

      //elimina
      std::vector<ConnectionPtr> conns = path2_node->net_parent_connections_;
      conns.insert(conns.end(),path2_node->parent_connections_.begin(),path2_node->parent_connections_.end());
      for(const ConnectionPtr& conn: conns)
      {
        if(not (conn->getParent() != last_conn->getParent()))
        {
          ROS_INFO_STREAM("conn\n"<<*conn);
          ROS_INFO_STREAM("last conn\n"<<*last_conn);
          ROS_INFO_STREAM("CONN PARENT PTR"<<conn->getParent());
          ROS_INFO_STREAM("LAST CONN PARENT PTR"<<last_conn->getParent());

          assert(0);
        }
      }
      //

      ConnectionPtr new_conn= std::make_shared<Connection>(last_conn->getParent(),path2_node,true);
      new_conn->setCost(last_conn->getCost());
      new_conn->add();

      connecting_path_conn.back() = new_conn;
      connecting_path->setConnections(connecting_path_conn);

      ROS_INFO_STREAM("last conn child: "<<last_conn->getChild()->getConfiguration().transpose()<<" "<<last_conn->getChild()); //elimina
      ROS_INFO_STREAM("last conn parent: "<<last_conn->getParent()->getConfiguration().transpose()<<" "<<last_conn->getParent()); //elimina

      last_conn->remove();

      if(pathSwitch_disp_)
      {
        disp_->changeConnectionSize({0.02,0.02,0.02});
        int connecting_path_id = disp_->displayPath(connecting_path,"pathplan",{1.0,0.4,0.0,1.0});
        disp_->nextButton("Displaying connecting_path..");
        disp_->clearMarker(connecting_path_id);
        disp_->defaultConnectionSize();
      }

      if(not((path2_node != tree_->getRoot() && path2_node->parent_connections_.size() == 1) ||
             (path2_node == tree_->getRoot() && path2_node->parent_connections_.size() == 0)))
      {
        ROS_INFO_STREAM("path2_node "<<path2_node);
        ROS_INFO_STREAM(*path2_node);

        ROS_INFO_STREAM("root "<<tree_->getRoot());
        ROS_INFO_STREAM(*tree_->getRoot());

        ROS_INFO_STREAM("fake root "<<paths_start_);
        ROS_INFO_STREAM(*paths_start_);

        assert(0);
      }

      subtree->removeNode(path2_node_fake); //disconnect and remove the fake node
      assert(not tree_->isInTree(path2_node_fake));

      return true;
    }
    else
    {
      if(pathSwitch_verbose_)
        ROS_INFO("No free solutions found in the subtree");
    }
  }

  /* If a solution was not found or the found solution was not free */
  solver_has_solved = false;
  connecting_path = nullptr;

  subtree->removeNode(path2_node_fake); //disconnect and remove the fake node
  assert(not tree_->isInTree(path2_node_fake));

  return false;
}

bool AIPRO::pathSwitch(const PathPtr &current_path,
                       const NodePtr &path1_node,
                       PathPtr &new_path)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;

  (pathSwitch_disp_ == true)?
        (pathSwitch_max_time_ = std::numeric_limits<double>::infinity()):
        (pathSwitch_max_time_ = available_time_);

  double time = pathSwitch_max_time_;
  std::vector<double> time_vector;

  if(pathSwitch_verbose_)
    ROS_INFO_STREAM("PathSwitch cycle time mean: "<<pathSwitch_cycle_time_mean_);

  if(pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
    time_vector.push_back(pathSwitch_cycle_time_mean_);

  if(not pathSwitch_disp_)
  {
    if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    {
      if(time<=0.0)
        return false;
    }
    else
    {
      if(time<time_percentage_variability_*pathSwitch_cycle_time_mean_)
        return false;
    }
  }

  int new_node_id;
  std::vector<int> node_id_vector;
  std::vector<double> marker_scale_sphere(3,0.025);
  std::vector<double> marker_color_sphere = {0.5,0.5,0.5,1.0};

  std::vector<double> marker_color = {1.0,0.5,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);

  bool success = false;
  NodePtr path1_node_of_sol, path2_node_of_sol;

  /* Identifying the subpath of current_path starting from node. Iit should be on the best path
   * from node to goal because current path is the result of bestExistingSolution */

  PathPtr path1_subpath;
  path1_subpath = current_path->getSubpathFromNode(path1_node);

  double path1_subpath_cost = path1_subpath->cost();
  double candidate_solution_cost = path1_subpath_cost;

  std::vector<node_and_path> ordered_goals_and_paths = sortNodesOnDistance(path1_node);

  for(const node_and_path& n_p:ordered_goals_and_paths)
  {
    tic_cycle = ros::WallTime::now();

    NodePtr path2_node = n_p.node;
    PathPtr path2 = n_p.path;

    PathPtr path2_subpath = nullptr;
    std::vector<ConnectionPtr> path2_subpath_conn;
    double path2_subpath_cost = 0.0;

    /* Search for a better path2_subpath from path2_node*/
    if(path2_node != goal_node_)
    {
      path2_subpath      = path2->getSubpathFromNode(path2_node);
      path2_subpath_conn = path2_subpath->getConnections();
      path2_subpath_cost = path2_subpath->cost();

      ROS_INFO_STREAM("path1_node: "<<path1_node->getConfiguration().transpose()<<" "<<path1_node<<" path2_node: "<<path2_node->getConfiguration().transpose()<<" "<<path2_node);//elimina

      int cc = 0;
      for(const PathPtr&p:admissible_other_paths_)
      {
        cc++;
        ROS_INFO_STREAM("path "<<cc);
        for(const NodePtr&n:p->getNodes()) //elimina
          ROS_INFO_STREAM("pn: "<<n->getConfiguration().transpose()<<" "<<n);
      }

      double better_path2_subpath_cost;
      std::vector<ConnectionPtr> better_path2_subpath_conn;
      std::multimap<double,std::vector<ConnectionPtr>> path2_subpath_map = net_->getConnectionBetweenNodes(path2_node,goal_node_);
      if(findValidSolution(path2_subpath_map,path2_subpath_cost,better_path2_subpath_conn,better_path2_subpath_cost))
      {
        path2_subpath_conn = better_path2_subpath_conn;
        path2_subpath = std::make_shared<Path>(path2_subpath_conn,metrics_,checker_);
        path2_subpath_cost = better_path2_subpath_cost;

        assert(better_path2_subpath_cost == path2_subpath->cost());

        if(pathSwitch_verbose_)
          ROS_INFO_STREAM("A better path2_subpath has been found");
      }
    }

    double diff_subpath_cost = candidate_solution_cost - path2_subpath_cost; //it is the maximum cost to make the connecting_path convenient
    double utopia = metrics_->utopia(path1_node->getConfiguration(),path2_node->getConfiguration()); //the Euclidean distance is the minimum cost that the connecting_path can have

    if(pathSwitch_disp_ || pathSwitch_verbose_)
    {
      ROS_INFO_STREAM("candidate_solution_cost: "<<candidate_solution_cost<<" subpath2_cost: "<<path2_subpath_cost);
      ROS_INFO_STREAM("diff_subpath_cost: "<< diff_subpath_cost<<" utopia: " << utopia);
    }

    /* The Euclidean distance between the two nodes must be
     * less than the maximum cost allowed for the connecting_path */
    if(utopia < 0.999*diff_subpath_cost)
    {
      ROS_INFO_STREAM("path1_node: "<<path1_node->getConfiguration().transpose()<<" path2_node: "<<path2_node->getConfiguration().transpose()<<" diff_subpath_cost: "<< diff_subpath_cost<<" utopia: " << utopia); //elimina

      PathPtr connecting_path;
      bool quickly_solved = false;
      bool solver_has_solved = computeConnectingPath(path1_node, path2_node, diff_subpath_cost, current_path, tic, tic_cycle, connecting_path, quickly_solved);

      if(solver_has_solved)
      {
        assert(connecting_path->isValid()); //elimina

        bool straight_path = true;

        if(connecting_path->getConnectionsSize() > 1)
        {
          ConnectionPtr conn1,conn2;
          for(unsigned int i=0;i<connecting_path->getConnectionsSize()-1;i++)
          {
            conn1 = connecting_path->getConnections().at(i);
            conn2 = connecting_path->getConnections().at(i+1);

            if(not conn1->isParallel(conn2))
            {
              straight_path = false;
              break;
            }
          }
        }

        if(not straight_path)
        {
          //double opt_time = maxSolverTime(tic,tic_cycle);
          //optimizePath(connecting_path,opt_time);
        }

        double new_solution_cost = path2_subpath_cost + connecting_path->cost();

        ROS_INFO_STREAM("solution cost: "<<new_solution_cost); //elimina

        if(pathSwitch_verbose_ || pathSwitch_disp_)
          ROS_INFO_STREAM("solution cost: "<<new_solution_cost);

        if(new_solution_cost<candidate_solution_cost)
        {
          std::vector<ConnectionPtr> new_path_conn = connecting_path->getConnections();

          if(not path2_subpath_conn.empty())
            new_path_conn.insert(new_path_conn.end(),path2_subpath_conn.begin(),path2_subpath_conn.end());

          new_path = std::make_shared<Path>(new_path_conn, metrics_, checker_);
          new_path->setTree(tree_);

          std::vector<Eigen::VectorXd> wp = new_path->getWaypoints();
          for(unsigned int z=0;z<wp.size()-1;z++)
          {
            if(wp.at(z) == wp.at(z+1)) //elimina
            {
              for(const Eigen::VectorXd& w:wp)
                ROS_INFO_STREAM("wp: "<<w.transpose());
              assert(0);
            }
          }

          candidate_solution_cost = new_path->cost();

          path1_node_of_sol = path1_node;
          path2_node_of_sol = path2_node;

          success = true;
          an_obstacle_ = false;

          if(pathSwitch_disp_)
          {
            disp_->clearMarker(pathSwitch_path_id_);
            disp_->changeConnectionSize(marker_scale);
            pathSwitch_path_id_ = disp_->displayPath(new_path,"pathplan",marker_color);
            disp_->defaultConnectionSize();
          }
        }
        else
        {
          if(pathSwitch_verbose_ || pathSwitch_disp_)
            ROS_INFO_STREAM("It is not a better solution");

          if(pathSwitch_disp_)
          {
            disp_->changeNodeSize(marker_scale_sphere);
            new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
            disp_->defaultNodeSize();

            node_id_vector.push_back(new_node_id);
          }
        }

        toc_cycle = ros::WallTime::now();
        if(pathSwitch_verbose_)
          ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc_cycle-tic_cycle).toSec());

        if(not quickly_solved)  //not directly connected, usually it is very fast and it would alterate the mean value
        {
          time_vector.push_back((toc_cycle-tic_cycle).toSec());
          pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());

          if(pathSwitch_verbose_)
            ROS_INFO_STREAM("cycle time mean updated: "<<pathSwitch_cycle_time_mean_);
        }
        else
        {
          if(pathSwitch_verbose_)
            ROS_INFO_STREAM("cycle time mean not updated");
        }

        if(pathSwitch_disp_)
          disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
      }
      else
      {
        if(!an_obstacle_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
        {
          pathSwitch_cycle_time_mean_ = 1.2*pathSwitch_cycle_time_mean_;

          if(pathSwitch_verbose_)
            ROS_INFO_STREAM("cycle time mean increased of 20%: "<<pathSwitch_cycle_time_mean_);
        }

        if(pathSwitch_disp_)
        {
          ROS_INFO("Not solved");

          disp_->changeNodeSize(marker_scale_sphere);
          new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
          disp_->defaultNodeSize();

          node_id_vector.push_back(new_node_id);

          disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
        }
      }
    }
    else
    {
      if(pathSwitch_verbose_ || pathSwitch_disp_)
        ROS_INFO_STREAM("It would not be a better solution");

      if(pathSwitch_disp_)
      {
        disp_->changeNodeSize(marker_scale_sphere);
        new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
        disp_->defaultNodeSize();

        node_id_vector.push_back(new_node_id);

        disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
      }
    }

    if(pathSwitch_verbose_)
    {
      toc=ros::WallTime::now();
      time = pathSwitch_max_time_ - (toc-tic).toSec();
      ROS_INFO_STREAM("cycle time mean: "<<pathSwitch_cycle_time_mean_<<" -> available time: "<< time);
    }

    toc=ros::WallTime::now();
    time = pathSwitch_max_time_ - (toc-tic).toSec();
    if((!an_obstacle_ && time<time_percentage_variability_*pathSwitch_cycle_time_mean_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()) || time<=0.0)  //if there is an obstacle, you should use the entire available time to find a feasible solution
    {
      if(pathSwitch_verbose_)
        ROS_INFO_STREAM("TIME OUT! max time: "<<pathSwitch_max_time_<<", time_available: "<<time<<", time needed for a new cycle: "<<time_percentage_variability_*pathSwitch_cycle_time_mean_);

      break;
    }
  }

  if(pathSwitch_verbose_ || pathSwitch_disp_)
  {
    if(pathSwitch_verbose_)
      ROS_INFO_STREAM("PathSwitch duration: "<<(ros::WallTime::now()-tic).toSec());

    if(pathSwitch_disp_)
    {
      for(const int& id_to_delete:node_id_vector)
        disp_->clearMarker(id_to_delete);
    }

    if(success)
      ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost()<<". Path1_node conf: "<<path1_node_of_sol->getConfiguration().transpose()<<" path2_node conf: "<<path2_node_of_sol->getConfiguration().transpose());
    else
      ROS_INFO_STREAM("PathSwitch has NOT found a solution");
  }

  return success;
}

bool AIPRO::stealSubtree(const NodePtr& node)
{
  /*Steal subtree from a close node and attch it to node*/

  std::vector<NodePtr> nodes = current_path_->getNodes();

  NodePtr successor   = nullptr;
  NodePtr predecessor = nullptr;

  unsigned int idx = -1;
  for(unsigned int i=0;i<nodes.size();i++)
  {
    if(node == nodes.at(i))
      idx = i;
  }

  if(idx<0)
  {
    ROS_ERROR("Node not member of current path");
    return false;
  }
  else if(idx == 0)
  {
    successor = nodes.at(idx+1);
  }
  else if(idx == (nodes.size()-1))
  {
    predecessor = nodes.at(idx-1);
    assert(0);
  }
  else
  {
    successor = nodes.at(idx+1);
    predecessor = nodes.at(idx-1);
  }

  if((successor == nullptr) && (predecessor == nullptr))
  {
    ROS_ERROR("Successor and predecessor not found");
    return false;
  }
  else
  {
    SubtreePtr successor_subtree   = nullptr;
    SubtreePtr predecessor_subtree = nullptr;
    unsigned int successor_subtree_dim   = 0;
    unsigned int predecessor_subtree_dim = 0;

    if(successor)
    {
      successor_subtree = pathplan::Subtree::createSubtree(tree_,successor,nodes);
      successor_subtree_dim = successor_subtree->getNumberOfNodes();
    }

    if(predecessor)
    {
      predecessor_subtree = pathplan::Subtree::createSubtree(tree_,successor,nodes);
      predecessor_subtree_dim = predecessor_subtree->getNumberOfNodes();
    }

    if((successor_subtree_dim == 0) && (predecessor_subtree_dim == 0))
    {
      ROS_ERROR("Subtree not available");
      return false;
    }

    std::vector<NodePtr> selected_nodes;
    if(successor_subtree_dim>predecessor_subtree_dim)
    {
      selected_nodes.push_back(successor);
      if(predecessor_subtree_dim>0)
        selected_nodes.push_back(predecessor);
    }
    else
    {
      selected_nodes.push_back(predecessor);
      if(successor_subtree_dim>0)
        selected_nodes.push_back(successor);
    }

    bool success = false;
    for(const NodePtr& selected_node:selected_nodes)
    {
      std::vector<ConnectionPtr> child_connections = selected_node->child_connections_;
      for(const ConnectionPtr& child_conn:child_connections)
      {
        if(std::find(nodes.begin(),nodes.end(),child_conn->getChild())<nodes.end())
          continue;
        else
        {
          if((node->getConfiguration()-child_conn->getChild()->getConfiguration()).norm()<tree_->getMaximumDistance())
          {
            if(checker_->checkPath(node->getConfiguration(),child_conn->getChild()->getConfiguration()))
            {
              ConnectionPtr conn = std::make_shared<Connection>(node,child_conn->getChild());
              conn->setCost(metrics_->cost(node->getConfiguration(),child_conn->getChild()->getConfiguration()));
              conn->add();

              conn->setRecentlyChecked(true);
              checked_connections_.push_back(conn);

              child_conn->remove();
              success = true;
            }
            else
            {
              ROS_ERROR("CHECK FALLITO");
            }
          }
          else
          {
            ROS_ERROR("TROPPO DISTANTI");
          }
        }
      }

      if(success)
        return true;
    }
  }

  return false;
}

PathPtr AIPRO::getSubpath1(const ConnectionPtr& current_conn, NodePtr& current_node)
{
  PathPtr subpath1;

  //If the current configuration matches a node of the current_path_
  std::vector<NodePtr> current_path_nodes = current_path_->getNodes();
  if(current_configuration_ == current_path_nodes.back()->getConfiguration())
  {
    ROS_WARN("The current node is the goal!");
    current_node = current_path_nodes.back();
    subpath1 = nullptr;

    return subpath1;
  }

  for(unsigned int i=0;i<current_path_nodes.size()-1;i++)
  {
    if(current_configuration_ == current_path_nodes.at(i)->getConfiguration())
    {
      current_node = current_path_nodes.at(i);
      subpath1 = current_path_->getSubpathFromNode(current_node);

      return subpath1;
    }
  }

  //If the current configuration doesn't match any node of the current_path_
  PathPtr subpath_from_child;
  std::vector<ConnectionPtr> subpath_from_child_conn;

  NodePtr child = current_conn->getChild();
  current_node = std::make_shared<Node>(current_configuration_);

  if(current_conn != current_path_->getConnections().back())
  {
    subpath_from_child = current_path_->getSubpathFromNode(child);
    subpath_from_child_conn = subpath_from_child->getConnections();
  }

  std::vector<ConnectionPtr> subpath1_conn;
  ConnectionPtr current2child_conn = nullptr;
  double current2child_conn_cost = 0;

  current2child_conn = std::make_shared<Connection>(current_node,child,true);

  if(current_conn->getCost() == std::numeric_limits<double>::infinity())
  {
    checker_->checkConnection(current2child_conn)?
          (current2child_conn_cost = metrics_->cost(current_node,child)):
          (current2child_conn_cost = std::numeric_limits<double>::infinity());

    current2child_conn->setRecentlyChecked(true);
    checked_connections_.push_back(current2child_conn);
  }
  else
    current2child_conn_cost = metrics_->cost(current_node,child);

  current2child_conn->setCost(current2child_conn_cost);
  current2child_conn->add();

  subpath1_conn.push_back(current2child_conn);

  if(not subpath_from_child_conn.empty())
    subpath1_conn.insert(subpath1_conn.end(),subpath_from_child_conn.begin(),subpath_from_child_conn.end());

  subpath1 = std::make_shared<Path>(subpath1_conn,metrics_,checker_);

  return subpath1;
}

void AIPRO::initCheckedConnections()
{
  if(not  checked_connections_.empty())
  {
    for(ConnectionPtr& checked_conn:checked_connections_)
      checked_conn->setRecentlyChecked(false);
  }

  checked_connections_.clear();

  checked_connections_ = current_path_->getConnections();
  for(const PathPtr& p:other_paths_)
    checked_connections_.insert(checked_connections_.end(),p->getConnectionsConst().begin(),p->getConnectionsConst().end());

  for(const ConnectionPtr& checked_conn:checked_connections_)
    checked_conn->setRecentlyChecked(true);
}

bool AIPRO::informedOnlineReplanning(const double &max_time)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;

  double MAX_TIME;
  if(informedOnlineReplanning_disp_)
    MAX_TIME = std::numeric_limits<double>::infinity();
  else
    MAX_TIME = max_time;

  available_time_ = MAX_TIME;
  const double TIME_LIMIT = 0.85*MAX_TIME; //seconds
  const int CONT_LIMIT = 5;

  if(not informedOnlineReplanning_disp_ && available_time_<=0.0)
    return false;

  /*//////////////For display//////////////////*/
  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere_analizing = {1.0,0.5,0.0,1.0};
  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);
  int replanned_path_id;
  /*/////////////////////////////////////////*/

  std::vector<PathPtr> replanned_paths_vector, reset_other_paths;
  std::vector<NodePtr> examined_nodes;
  PathPtr new_path, replanned_path;
  PathPtr admissible_current_path = nullptr;
  bool exit = false;
  bool solved = false;
  bool first_sol = true;
  unsigned int cont = 0;
  double previous_cost;
  double replanned_path_cost = std::numeric_limits<double>::infinity();

  success_ = false;
  an_obstacle_ = false;

  /*Clear the vector of connections set invalid during a previous call to the replanner
   * (MUST BE HERE AND NOT AT THE END)*/
  clearInvalidConnections();
  assert(invalid_connections_.empty());

  /*Set the connections of the available paths to recently checked.
   * They don't need a collision check by the replanner because they are checked externally.*/
  initCheckedConnections();

  /*Add the valid portion of the current path to the set of available paths*/
  int current_conn_idx;
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,current_conn_idx);

  admissible_other_paths_.clear();
  reset_other_paths = addAdmissibleCurrentPath(current_conn_idx,admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  /*Compute the subpath1*/
  NodePtr current_node;
  PathPtr subpath1 = getSubpath1(current_conn,current_node); //nullptr if subpath1 does not exist (current_node = goal)
  if(subpath1)
  {
    if(subpath1->cost() == std::numeric_limits<double>::infinity())
      an_obstacle_ = true;
  }
  else
  {
    ROS_WARN("The current configuration matches with the goal");

    for(const ConnectionPtr& checked_conn:checked_connections_)
      checked_conn->setRecentlyChecked(true);

    success_ = false;
    return false;
  }

  /*Searching for an already existing solution*/
  if(informedOnlineReplanning_verbose_)
    ROS_INFO("Searching for an already existing solution..");

  replanned_path = bestExistingSolution(subpath1);  //if a solution is not found, replanned_path = subpath1
  replanned_path_cost = replanned_path->cost();

  if(replanned_path != subpath1)  //if an already existing solution has been found (different from subpath1), success = true
  {
    success_ = true;
    assert(replanned_path->cost()<std::numeric_limits<double>::infinity());

    if(informedOnlineReplanning_verbose_)
      ROS_INFO_STREAM("A ready-to-use solution has been found, cost: "<<replanned_path_cost);
  }
  else
  {
    if(informedOnlineReplanning_verbose_)
      ROS_INFO_STREAM("A solution better than subpath1 has not been found, cost: "<<replanned_path_cost);
  }

  assert(replanned_path->getTree() == tree_);

  if(informedOnlineReplanning_disp_)
  {
    disp_->changeConnectionSize(marker_scale);
    replanned_path_id = disp_->displayPath(replanned_path,"pathplan",marker_color);
    disp_->defaultConnectionSize();
    disp_->nextButton("Press Next to start searching for a better solution");
  }

  std::vector<NodePtr> start_node_vector = startNodes(replanned_path->getConnectionsConst());

  /*If the replan starts from the current node, try to steal a
   * subtree from one of the closest node to have more chances of success*/

  //  if((start_node_vector.size() == 1) && (start_node_vector.front() == current_node))
  //  {
  //    bool subtree_stolen = stealSubtree(current_node);

  //    if(informedOnlineReplanning_verbose_)
  //    {
  //      if(subtree_stolen)
  //        ROS_WARN("Subtree stolen from a near node and attached to the current node");
  //      else
  //        ROS_WARN("Subtree not stolen from a near node ");
  //    }
  //  }

  int j = start_node_vector.size()-1;
  NodePtr start_node_for_pathSwitch;

  while(j>=0)
  {
    tic_cycle = ros::WallTime::now();

    start_node_for_pathSwitch = start_node_vector.at(j);
    simplifyAdmissibleOtherPaths(replanned_path,start_node_for_pathSwitch,reset_other_paths);

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO_STREAM("j: "<<j);

    if(informedOnlineReplanning_disp_)
    {
      disp_->changeNodeSize(marker_scale_sphere);
      disp_->displayNode(start_node_for_pathSwitch,"pathplan",marker_color_sphere_analizing);
      disp_->defaultNodeSize();
    }

    if(pathSwitch_cycle_time_mean_ >= 0.8*max_time)
      pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();  //reset

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME - (toc-tic).toSec();

    double min_time_to_launch_pathSwitch;
    if(informedOnlineReplanning_disp_)
      min_time_to_launch_pathSwitch = std::numeric_limits<double>::infinity();
    else if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity())
      min_time_to_launch_pathSwitch = 0.0;
    else
      min_time_to_launch_pathSwitch = time_percentage_variability_*pathSwitch_cycle_time_mean_;

    if(informedOnlineReplanning_verbose_)
      ROS_INFO_STREAM("available time: "<<available_time_<<", min required time to call PathSwitch: "<<min_time_to_launch_pathSwitch);

    if(available_time_>=min_time_to_launch_pathSwitch)
    {
      if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
        ROS_INFO_STREAM("Launching PathSwitch...");

      solved = pathSwitch(replanned_path,start_node_for_pathSwitch,new_path);

      start_node_for_pathSwitch->setAnalyzed(true);
      examined_nodes.push_back(start_node_for_pathSwitch);

      assert((solved && new_path->getTree() != nullptr) || (not solved));
    }
    else
    {
      solved = false;
      exit = true;

      if(informedOnlineReplanning_verbose_)
        ROS_INFO_STREAM("Not enough time to call PathSwitch, available time: "<<available_time_<<" min time to call PathSwitch: "<<min_time_to_launch_pathSwitch);
    }

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO_STREAM("Solved: "<<solved);

    if(solved)
    {
      PathPtr candidate_solution;
      std::vector<ConnectionPtr> candidate_solution_conn;

      if(start_node_for_pathSwitch != current_node)
      {
        try
        {
          candidate_solution_conn = replanned_path->getSubpathToNode(start_node_for_pathSwitch)->getConnections();
        }
        catch(...)
        {
          ROS_INFO_STREAM("ps start node: "<<start_node_for_pathSwitch<<" "<<*start_node_for_pathSwitch);
          ROS_INFO_STREAM("current node: "<<current_node<<" "<<*current_node);
          for(const NodePtr& n:start_node_vector)
            ROS_INFO_STREAM("node ps: "<<n<<*n);

          disp_->changeConnectionSize({0.03,0.03,0.03});
          disp_->displayPath(replanned_path,"pathplan",{0.0,0.0,1.0,1.0});

          for(const NodePtr& n:replanned_path->getNodes())
            ROS_INFO_STREAM("node rep: "<<n<<*n);

          assert(0);
        }
        candidate_solution_conn.insert(candidate_solution_conn.end(),new_path->getConnectionsConst().begin(),new_path->getConnectionsConst().end());

        candidate_solution = std::make_shared<Path>(candidate_solution_conn,metrics_,checker_);
        candidate_solution->setTree(tree_);
      }
      else
      {
        candidate_solution = new_path;
        assert(tree_ == new_path->getTree());
      }

      assert((candidate_solution->getTree() == new_path->getTree()) && (candidate_solution->getTree() == tree_));

      if(candidate_solution->cost()<replanned_path_cost)
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          ROS_INFO_STREAM("new path found, cost: " << candidate_solution->cost() <<" previous cost: " << replanned_path_cost);

        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          time_replanning_ = time_first_sol_;
          first_sol = false;  //false when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = candidate_solution;
        replanned_path_cost = candidate_solution->cost();

        assert(replanned_path->getTree() == tree_);
        assert(replanned_path_cost < std::numeric_limits<double>::infinity());

        replanned_paths_vector.push_back(replanned_path);

        success_ = true;
        an_obstacle_ = false;

        if(informedOnlineReplanning_disp_)
        {
          disp_->clearMarker(pathSwitch_path_id_);
          disp_->clearMarker(replanned_path_id);
          disp_->changeConnectionSize(marker_scale);
          replanned_path_id = disp_->displayPath(replanned_path,"pathplan",marker_color);
          disp_->defaultConnectionSize();
        }

        toc = ros::WallTime::now();
        if((toc-tic).toSec()>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
          break;
        }
        else
          ((previous_cost-replanned_path_cost)<(0.05*previous_cost))? cont++:
                                                                      cont = 0;
      }
      else
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          ROS_INFO_STREAM("NO better path found, cost: " << candidate_solution->cost() <<" previous cost: " << replanned_path_cost);
      }

      toc_cycle = ros::WallTime::now();
      if(informedOnlineReplanning_verbose_)
        ROS_INFO_STREAM("Solution with cost "<<replanned_path_cost<<" found!->Informed cycle duration: "<<(toc_cycle-tic_cycle).toSec());
    }

    if(success_ && j == 0)
    {
      if(replanned_path->getConnectionsSize()>1)
      {
        subpath1 = replanned_path;
        start_node_vector = startNodes(replanned_path->getConnectionsConst());
        j = start_node_vector.size();

        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          ROS_INFO_STREAM("NEW J: "<<j-1);
      }
    }

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME-(toc-tic).toSec();

    if(exit || j==0)
    {
      if(informedOnlineReplanning_verbose_ && exit)
        ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<min_time_to_launch_pathSwitch);

      if(informedOnlineReplanning_disp_)
      {
        ROS_INFO("Optimizing...");
        disp_->nextButton();
      }

      // double cost_pre_opt = replanned_path->cost();
      // ros::WallTime tic_warp = ros::WallTime::now();
      // if(success)
      //   optimizePath(replanned_path,available_time_*0.95);
      // ros::WallTime toc_warp = ros::WallTime::now();
      // double cost_opt = replanned_path->cost();
      //
      // if(informedOnlineReplanning_verbose_)
      //   ROS_INFO_STREAM("Path optimization, max time: "<<available_time_<<" time used: "<<(toc_warp-tic_warp).toSec()<<" previous cost: "<<cost_pre_opt<<" new cost: "<<cost_opt);

      j = -1;
      break;
    }

    j -= 1;

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO("------------------------------------------");
    if(informedOnlineReplanning_disp_)
      disp_->nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");
  }

  for(NodePtr& examined_node:examined_nodes)
    examined_node->setAnalyzed(false);

  for(ConnectionPtr& checked_conn:checked_connections_)
    checked_conn->setRecentlyChecked(false);

  if(success_)
  {
    replanned_path_ = replanned_path;

    assert(replanned_path_->getTree() == tree_);

    if(replanned_paths_vector.size()>10)
    {
      std::vector<PathPtr> best_10_paths(replanned_paths_vector.end()-10, replanned_paths_vector.end());
      replanned_paths_vector_ = best_10_paths;
    }
    else
      replanned_paths_vector_ = replanned_paths_vector;

    std::reverse(replanned_paths_vector_.begin(),replanned_paths_vector_.end());  //ordered with growing cost

    toc = ros::WallTime::now();
    time_replanning_ = (toc - tic).toSec();

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
    {
      ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost() << " in "<< time_replanning_ << "seconds. Number of new sol: " << replanned_paths_vector.size());

      for(const ConnectionPtr & conn:replanned_path_->getConnections()) //elimina
        ROS_INFO_STREAM("rep conn: "<<*conn<< " "<<conn);

      bool equal = true;
      std::multimap<double,std::vector<ConnectionPtr>> new_conns_map = net_->getConnectionBetweenNodes(current_node,goal_node_);
      for(const ConnectionPtr & conn:new_conns_map.begin()->second) //elimina
      {
        ROS_INFO_STREAM("net conn: "<<*conn<<" "<<conn);

        if(not (std::find(replanned_path_->getConnectionsConst().begin(),replanned_path_->getConnectionsConst().end(),conn)<replanned_path_->getConnectionsConst().end()))
          equal = false;
      }

      if(not equal)
      {
        PathPtr p = std::make_shared<Path>(new_conns_map.begin()->second,metrics_,checker_);
        ROS_WARN_STREAM("cost rep: "<<replanned_path_->cost()<<" net path cost: "<<p->cost());

        //printa tutte le soluzioni
        for(const std::pair<double,std::vector<ConnectionPtr>> &sol:new_conns_map)
        {
          PathPtr p = std::make_shared<Path>(sol.second,metrics_,checker_);
          ROS_INFO_STREAM("sol cost: "<<p->cost());
        }

        disp_->displayPath(p);
        disp_->displayPathAndWaypoints(replanned_path_,"pathplan",{0.0,0.0,1.0,1.0});
        assert(0);
      }
    }
  }
  else
  {
    success_ = false;
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  available_time_ = MAX_TIME-(toc-tic).toSec();

  return success_;
}

bool AIPRO::replan()
{
  ros::WallTime tic = ros::WallTime::now();
  success_ = false;

  std::vector<NodePtr> nodes = current_path_->getNodes();

  //ELIMINA
  ROS_INFO("PRIMA DI AGGIUNGERE REPL NODE");
  for(const Eigen::VectorXd& wp:current_path_->getWaypoints())
    ROS_INFO_STREAM(wp.transpose());

  ConnectionPtr conn = current_path_->findConnection(current_configuration_);
  NodePtr current_node = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true,is_a_new_node_);

  //ELIMINA
  ROS_INFO("DOPO AVER AGGIUNTO REPL NODE");
  for(const Eigen::VectorXd& wp:current_path_->getWaypoints())
    ROS_INFO_STREAM(wp.transpose());

  if(verbose_)
  {
    ROS_INFO_STREAM("Starting node for replanning: \n"<< *current_node<<current_node<<"\nis a new node: "<<is_a_new_node_);
    ROS_INFO_STREAM("Cost from here: "<<current_path_->getCostFromConf(current_configuration_));
  }

  double max_time = max_time_-(tic-ros::WallTime::now()).toSec();
  success_ = informedOnlineReplanning(max_time);

  bool path_changed = true;  //curent_node added
  if(not success_ && is_a_new_node_)
  {
    if(current_path_->removeNode(current_node,nodes))
      path_changed = false;
  }
  return path_changed;
}
}
