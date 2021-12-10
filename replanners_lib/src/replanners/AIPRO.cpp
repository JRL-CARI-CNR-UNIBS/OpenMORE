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


bool AIPRO::mergePathToTree(PathPtr &path)
{
  TreePtr path_tree = path->getTree();

  if(!tree_)
  {
    if(path_tree)
      tree_ = path_tree;
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

      tree_ = std::make_shared<Tree>(path->getNodes().front(),Direction::Forward,max_dist,checker_,metrics_);
      tree_->addBranch(path->getConnections());
    }

    NodePtr path_start = current_path_->getConnections().front()->getParent();
    if(tree_->getRoot()->getConfiguration() == path_start->getConfiguration())
    {
      if(tree_->getRoot() != path_start)
      {
        ConnectionPtr conn_child = current_path_->getConnections().front();
        ConnectionPtr conn = std::make_shared<Connection>(tree_->getRoot(),conn_child->getChild());
        conn->setCost(conn_child->getCost());
        conn->add();

        conn_child->remove();

        std::vector<ConnectionPtr> connections = current_path_->getConnections();
        connections.at(0) == conn;
        current_path_->setConnections(connections);
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
    if(tree_->getRoot()->getConfiguration() == path_start->getConfiguration())
    {
      if(tree_->getRoot() != path_start)
      {
        std::vector<ConnectionPtr> connections;
        if(path_tree)
        {
          if(path_start != path_tree->getRoot())
            assert(0);

          NodePtr path_goal = path->getConnections().back()->getChild();

          path_tree->addNode(tree_->getRoot());
          for(const ConnectionPtr& conn_child:path_start->child_connections_)
          {
            ConnectionPtr conn = std::make_shared<Connection>(tree_->getRoot(),conn_child->getChild());
            conn->setCost(conn_child->getCost());
            conn->add();

            conn_child->remove();
          }
          path_tree->removeNode(path_start);

          connections = path_tree->getConnectionToNode(path_goal);
        }
        else
        {
          ConnectionPtr conn_child = path->getConnections().front();
          ConnectionPtr conn = std::make_shared<Connection>(tree_->getRoot(),conn_child->getChild());
          conn->setCost(conn_child->getCost());
          conn->add();

          conn_child->remove();

          connections = path->getConnections();
          connections.at(0) == conn;
        }

        path->setConnections(connections);
      }

      if(path_tree)
        tree_->addTree(path_tree);
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

  net_->setTree(tree_);

  return true;
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
      int z = current_path_->getConnections().size()-2;  //penultimate connection (last connection is at end-1)
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
    if(idx_current_conn<current_path_->getConnections().size()-1)
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
  std::vector<node_and_path> unconnected_goals, ordered_unconnected_goals;

  for(const unconnected_nodes& this_struct : unconnected_nodes_)
  {
    if(this_struct.start_node == start_node)
    {
      unconnected_goals = this_struct.goals_and_paths;
      break;
    }
  }

  double distance;
  for(const node_and_path& goal_and_path:unconnected_goals)
  {
    distance = (start_node->getConfiguration()-goal_and_path.node->getConfiguration()).norm();
    node_and_path_map.insert(std::pair<double,node_and_path>(distance,goal_and_path));
  }

  for(const std::pair<double,node_and_path>& p: node_and_path_map)
    ordered_unconnected_goals.push_back(p.second);

  return ordered_unconnected_goals;
}

std::vector<NodePtr> AIPRO::startingNodesForPathSwitch(const std::vector<ConnectionPtr>& subpath1_conn, const NodePtr& current_node, const double& current2child_conn_cost, const int& idx,  bool& available_nodes)
{
  std::vector<NodePtr> path1_node_vector;

  if(current2child_conn_cost == std::numeric_limits<double>::infinity() || idx == current_path_->getConnections().size()-1) //if the obstacle is obstructing the current connection, the replanning must start from the current configuration
  {
    available_nodes = 0;
    path1_node_vector.push_back(current_node);
  }
  else      //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
  {
    int subpath1_size =  subpath1_conn.size();
    for(unsigned int i=0; i<subpath1_size; i++)
    {
      if(i==subpath1_size-1)
      {
        if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())      // if the path is free, you can consider all the nodes but it is useless to consider the last one before the goal (it is already connected to the goal with a straight line)
          path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
      }
      else
      {
        path1_node_vector.push_back(subpath1_conn.at(i)->getParent());

        if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())
          break;
      }
    }
    available_nodes = 1;
  }

  return path1_node_vector;
}

PathPtr AIPRO::existingSolutions(const NodePtr& start_node)
{
  struct connecting_path_struct
  {
    std::vector<ConnectionPtr> connecting_conns;
    node_and_path goal_and_path;
  };

  std::multimap<double,connecting_path_struct> existing_solutions;
  std::multimap<double,std::vector<ConnectionPtr>> tmp_map;
  std::vector<node_and_path> unconnected_goals_and_paths;
  std::vector<NodePtr> path_nodes;
  double best_cost, subpath_cost;
  PathPtr subpath, solution;
  bool custom_sol_exists;

  NodePtr goal = current_path_->getConnections().back()->getChild();

  solution = current_path_->getSubpathFromNode(start_node);
  best_cost = solution->cost();

  unconnected_nodes nodes_unconnected_to_start;
  nodes_unconnected_to_start.start_node = start_node;

  for(const PathPtr& path:admissible_other_paths_)
  {
    path_nodes = path->getNodes();

    for(const NodePtr& path_node:path_nodes)
    {
      subpath_cost = 0;
      subpath = nullptr;

      if(path_node != goal)
      {
        subpath = path->getSubpathFromNode(path_node);
        subpath_cost = subpath->cost();
      }

      tmp_map = net_->getNetConnectionBetweenNodes(start_node,path_node);

      custom_sol_exists = false;
      if(!tmp_map.empty())
      {
        for(const std::pair<double,std::vector<ConnectionPtr>>& tmp_pair:tmp_map)
        {
          if(tmp_pair.second.front() != solution->getConnections().front())
          {
            node_and_path tmp_goal_and_path;
            tmp_goal_and_path.node = path_node;
            tmp_goal_and_path.path = path;

            std::pair<double,connecting_path_struct> solution_pair;
            solution_pair.first = tmp_pair.first + subpath_cost;
            solution_pair.second.connecting_conns = tmp_pair.second;
            solution_pair.second.goal_and_path = tmp_goal_and_path;

            existing_solutions.insert(solution_pair);

            custom_sol_exists = true;
          }
        }

        if(!custom_sol_exists)
        {
          node_and_path tmp_goal_and_path;
          tmp_goal_and_path.node = path_node;
          tmp_goal_and_path.path = path;

          unconnected_goals_and_paths.push_back(tmp_goal_and_path);
        }
      }
      else
      {
        node_and_path tmp_goal_and_path;
        tmp_goal_and_path.node = path_node;
        tmp_goal_and_path.path = path;

        unconnected_goals_and_paths.push_back(tmp_goal_and_path);
      }
    }
  }

  std::vector<ConnectionPtr> solution_conns;
  SubtreePtr subtree;
  ConnectionPtr conn;
  NodePtr child;
  bool free;

  for(const std::pair<double,connecting_path_struct>& solution_pair:existing_solutions)
  {
    if(solution_pair.first<best_cost)
    {
      free = true;
      for(unsigned int i=0; i<solution_pair.second.connecting_conns.size();i++)
      {
        conn = solution_pair.second.connecting_conns.at(i);
        if(i<solution_pair.second.connecting_conns.size()-1)
        {
          if(!checker_->checkConnection(conn))
          {
            unconnected_goals_and_paths.push_back(solution_pair.second.goal_and_path);

            subtree = std::make_shared<Subtree>(tree_,start_node);
            child = conn->getChild();
            subtree->purgeFromHere(child);  //destroy the subtree starting from the obstructed conn
            free = false;

            break;
          }
        }
        else
        {
          if(!checker_->checkConnection(conn))
          {
            unconnected_goals_and_paths.push_back(solution_pair.second.goal_and_path);

            conn->remove();
            free = false;
          }
        }
      }

      if(free)
      {
        solution_conns = solution_pair.second.connecting_conns;

        if(solution_pair.second.goal_and_path.node != goal)
        {
          subpath = solution_pair.second.goal_and_path.path->getSubpathFromNode(solution_pair.second.goal_and_path.node);
          solution_conns.insert(solution_conns.end(),subpath->getConnectionsConst().begin(), subpath->getConnectionsConst().end());
        }

        break;  //the solutions are ordered by cost, so if the solution is free it is the less costly one
      }
    }
  }

  nodes_unconnected_to_start.goals_and_paths = unconnected_goals_and_paths;
  unconnected_nodes_.push_back(nodes_unconnected_to_start);

  if(!solution_conns.empty())
    solution = std::make_shared<Path>(solution_conns,metrics_,checker_);

  return solution;
}

void AIPRO::simplifyAdmissibleOtherPaths(const bool& no_available_paths, const PathPtr& confirmed_subpath_from_path2, const int& confirmed_connected2path_number, const NodePtr& starting_node_of_pathSwitch, const std::vector<PathPtr>& reset_other_paths)
{
  if(confirmed_subpath_from_path2 && !no_available_paths)
  {
    std::vector<NodePtr> node_vector = confirmed_subpath_from_path2->getNodes();
    node_vector.pop_back();  // removing the goal from the vector

    int pos = -1;
    for (unsigned int k=0; k<node_vector.size(); k++)
    {
      if(starting_node_of_pathSwitch == node_vector.at(k))
      {
        pos = k;
        break;
      }
    }

    if(pos>=0)
    {
      if(confirmed_connected2path_number<admissible_other_paths_.size()-1)
      {
        admissible_other_paths_.clear();

        admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
        admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
        admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());
      }
      else
      {
        admissible_other_paths_.clear();

        admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
        admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
      }
    }
    else
    {
      admissible_other_paths_ = reset_other_paths;
    }
  }
  else
  {
    admissible_other_paths_ = reset_other_paths;
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
  int count = 0;

  bool simplify1 = false;
  bool simplify2 = false;
  bool simplified = false;

  do
  {
    simplify1 = replanned_path_->removeNodes();
    simplify2 = replanned_path_->simplify(distance);
    if(simplify1 || simplify2)
    {
      count +=1;
      simplified = true;
    }
  }
  while(simplify1 || simplify2);

  return simplified;
}

bool AIPRO::computeConnectingPath(const NodePtr& path1_node, const NodePtr& path2_node, const double& diff_subpath_cost, const ros::WallTime& tic, const ros::WallTime& tic_cycle, PathPtr& connecting_path, bool& quickly_solved)
{
  NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node->getConfiguration(), path2_node->getConfiguration(), lb_, ub_,diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
  SubtreePtr subtree = pathplan::Subtree::createSubtree(tree_,path1_node,path1_node->getConfiguration(),path2_node->getConfiguration(),diff_subpath_cost);

  solver_->setSampler(sampler);
  solver_->resetProblem();
  solver_->addStart(path1_node);

  if(subtree->getNodes().size()>1)
    solver_->setStartTree(subtree);

  double solver_time = maxSolverTime(tic,tic_cycle);

  if(pathSwitch_verbose_)
    ROS_INFO_STREAM("Searching for a direct connection...max time: "<<solver_time);

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

  bool solver_has_solved;
  ros::WallTime tic_solver;
  ros::WallTime toc_solver;

  if(solver_->solved())
  {
    connecting_path = solver_->getSolution();
    solver_has_solved = true;
  }
  else
  {
    solver_time = maxSolverTime(tic,tic_cycle);

    if(pathSwitch_verbose_)
      ROS_INFO_STREAM("solving...max time: "<<solver_time);

    tic_solver = ros::WallTime::now();
    solver_has_solved = solver_->solve(connecting_path,10000,solver_time);
    toc_solver = ros::WallTime::now();

    if(pathSwitch_verbose_)
    {
      if(solver_has_solved)
        ROS_INFO_STREAM("solved in time: "<<(toc_solver-tic_solver).toSec());
      else
        ROS_INFO_STREAM("not solved, time: "<<(toc_solver-tic_solver).toSec());
    }
  }

  if(solver_has_solved) //check the path found and then change the connection to the path2_node_fake with  net connection to path2_node
  {
    std::vector<ConnectionPtr> connections = connecting_path->getConnections();
    for(const ConnectionPtr& conn:connections)
    {
      if(!checker_->checkConnection(conn))
      {
        NodePtr child = conn->getChild();
        subtree->purgeFromHere(child);

        solver_has_solved = false;
        connecting_path = nullptr;

        if(pathSwitch_verbose_)
          ROS_INFO("Solution found not free, subtree purged");

        return solver_has_solved;
      }
    }

    ConnectionPtr last_conn = connections.back();

    NetConnectionPtr net_conn = std::make_shared<NetConnection>(last_conn->getParent(),path2_node);
    net_conn->setCost(last_conn->getCost());
    net_conn->add();

    connections.at(connections.size()-1) = net_conn;
    connecting_path->setConnections(connections);

    std::vector<NodePtr>::iterator it = std::find(subtree->getNodes().begin(), subtree->getNodes().end(), path2_node_fake);
    subtree->removeNode(it);
    last_conn->remove();
  }

  return solver_has_solved;
}

PathPtr AIPRO::concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn,
                                               const std::vector<ConnectionPtr>& path2_subpath_conn)
{
  std::vector<ConnectionPtr> new_connecting_path_conn = connecting_path_conn;

  if(!path2_subpath_conn.empty())
    new_connecting_path_conn.insert(new_connecting_path_conn.end(),path2_subpath_conn.begin(),path2_subpath_conn.end());

  PathPtr new_path = std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
  new_path->setTree(tree_);

  return new_path;
}

bool AIPRO::pathSwitch(const PathPtr &current_path,
                       const NodePtr &path1_node,
                       PathPtr &new_path,
                       PathPtr &subpath_from_path2,
                       int &connected2path_number)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;

  if(pathSwitch_disp_)
    pathSwitch_max_time_ = std::numeric_limits<double>::infinity();
  else
    pathSwitch_max_time_ = available_time_;

  double time = pathSwitch_max_time_;
  std::vector<double> time_vector;

  if(pathSwitch_verbose_)
    ROS_INFO_STREAM("PathSwitch cycle starting mean: "<<pathSwitch_cycle_time_mean_);

  if(pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
    time_vector.push_back(pathSwitch_cycle_time_mean_);

  if(!pathSwitch_disp_)
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
  NodePtr goal = current_path->getConnections().back()->getChild();

  // Identifying the subpath of current_path starting from node
  PathPtr path1_subpath = current_path->getSubpathFromNode(path1_node);
  double path1_subpath_cost = path1_subpath->cost();
  double candidate_solution_cost = path1_subpath_cost;

  std::vector<node_and_path> ordered_goals_and_paths = sortNodesOnDistance(path1_node);
  for(unsigned int i=0; i<ordered_goals_and_paths.size();i++)
  {
    tic_cycle = ros::WallTime::now();

    NodePtr path2_node = ordered_goals_and_paths.at(i).node;
    PathPtr path2 = ordered_goals_and_paths.at(i).path;

    PathPtr path2_subpath = nullptr;
    std::vector<ConnectionPtr> path2_subpath_conn;
    double path2_subpath_cost = 0.0;

    if(path2 != nullptr && path2_node != goal)
    {
      path2_subpath = path2->getSubpathFromNode(path2_node);
      path2_subpath_conn = path2_subpath->getConnections();
      path2_subpath_cost = path2_subpath->cost();
    }

    double diff_subpath_cost = candidate_solution_cost - path2_subpath_cost; //it is the maximum cost to make the connecting_path convenient
    double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

    if(pathSwitch_disp_)
    {
      ROS_INFO_STREAM("candidate_solution_cost: "<<candidate_solution_cost<<" subpath2_cost: "<<path2_subpath_cost);
      ROS_INFO_STREAM("diff_subpath_cost: "<< diff_subpath_cost<<" distance: " << distance_path_node);
    }

    if(diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it is useless to calculate a connecting_path because it surely will not be convenient
    {
      PathPtr connecting_path;
      bool quickly_solved = false;
      bool solver_has_solved = computeConnectingPath(path1_node, path2_node, diff_subpath_cost, tic, tic_cycle, connecting_path, quickly_solved);

      if(solver_has_solved)
      {
        bool straight_path;

        if(connecting_path->getConnections().size() == 1)
          straight_path = true;
        else
        {
          straight_path = true;

          ConnectionPtr conn1,conn2;
          for(unsigned int i=0;i<connecting_path->getConnections().size()-1;i++)
          {
            conn1 = connecting_path->getConnections().at(i);
            conn2 = connecting_path->getConnections().at(i+1);

            if(!conn1->isParallel(conn2))
            {
              straight_path = false;
              break;
            }
          }
        }

        if(!straight_path)
        {
          //double opt_time = maxSolverTime(tic,tic_cycle);
          //optimizePath(connecting_path,opt_time);
        }

        double new_solution_cost = path2_subpath_cost + connecting_path->cost();
        if(pathSwitch_verbose_ || pathSwitch_disp_)
          ROS_INFO_STREAM("solution cost: "<<new_solution_cost);

        if(new_solution_cost<candidate_solution_cost)
        {
          std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();
          new_path = concatConnectingPathAndSubpath2(connecting_path_conn,path2_subpath_conn);
          candidate_solution_cost = new_path->cost();

          success = true;
          an_obstacle_ = false;
          subpath_from_path2 = path2_subpath;
          for(unsigned int z=0; z<admissible_other_paths_.size();z++)
          {
            if(path2 == admissible_other_paths_.at(z))
              connected2path_number = z;
          }

          /*//////////Visualization//////////////////*/
          if(pathSwitch_disp_)
          {
            disp_->clearMarker(pathSwitch_path_id_);
            disp_->changeConnectionSize(marker_scale);
            pathSwitch_path_id_ = disp_->displayPath(new_path,"pathplan",marker_color);
            disp_->defaultConnectionSize();
          }
          /*//////////////////////////////////////*/
        }
        else
        {
          if(pathSwitch_verbose_ || pathSwitch_disp_)
            ROS_INFO_STREAM("It is not a better solution");

          /*//////////Visualization//////////////////*/
          if(pathSwitch_disp_)
          {
            disp_->changeNodeSize(marker_scale_sphere);
            new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
            disp_->defaultNodeSize();

            node_id_vector.push_back(new_node_id);
          }
          /*//////////////////////////////////////*/
        }

        toc_cycle = ros::WallTime::now();
        if(pathSwitch_verbose_)
          ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc_cycle-tic_cycle).toSec());

        if(!quickly_solved)  //not directly connected, usually it is very fast and it would alterate the mean value
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
          pathSwitch_cycle_time_mean_ = (2-time_percentage_variability_)*pathSwitch_cycle_time_mean_;

          if(pathSwitch_verbose_)
            ROS_INFO_STREAM("cycle time mean increased of 20%: "<<pathSwitch_cycle_time_mean_);
        }

        /*//////////Visualization//////////////////*/
        if(pathSwitch_disp_)
        {
          ROS_INFO("Not solved");

          disp_->changeNodeSize(marker_scale_sphere);
          new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
          disp_->defaultNodeSize();

          node_id_vector.push_back(new_node_id);

          disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
        }
        /*//////////////////////////////////////*/
      }
    }
    else
    {
      if(pathSwitch_verbose_ || pathSwitch_disp_)
        ROS_INFO_STREAM("It would not be a better solution");

      /*//////////Visualization//////////////////*/
      if(pathSwitch_disp_)
      {
        disp_->changeNodeSize(marker_scale_sphere);
        new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
        disp_->defaultNodeSize();

        node_id_vector.push_back(new_node_id);

        disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
      }
      /*//////////////////////////////////////*/
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
      ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
    else
      ROS_INFO_STREAM("PathSwitch has NOT found a solution");
  }

  return success;
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
  std::vector<double> time_vector;
  const double TIME_LIMIT = 0.85*MAX_TIME; //seconds
  const int CONT_LIMIT = 5;

  if(!informedOnlineReplanning_disp_ && available_time_<=0.0)
    return false;

  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere = {0.0,0.0,0.0,1.0};
  std::vector<double> marker_color_sphere_analizing = {1.0,0.5,0.0,1.0};

  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);
  int replanned_path_id;
  int node_id;

  PathPtr new_path;
  PathPtr replanned_path;
  PathPtr subpath_from_path2;
  PathPtr confirmed_subpath_from_path2 = nullptr;
  PathPtr admissible_current_path = nullptr;
  PathPtr subpath1;
  std::vector<PathPtr> replanned_path_vector;
  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> path1_node_vector;
  std::vector<ConnectionPtr> subpath1_conn;
  bool exit = false;
  bool success = false;
  bool solved = false;
  bool first_sol = true;
  bool available_nodes;
  bool no_available_paths = true;
  int confirmed_connected2path_number;
  int connected2path_number;
  unsigned int cont = 0;  //to count the number of replanning without significant improvement in the final solution
  double replanned_path_cost = std::numeric_limits<double>::infinity();
  double previous_cost = current_path_->getCostFromConf(current_configuration_);

  examined_nodes_.clear();
  success_ = false;
  an_obstacle_ = false;

  int current_conn_idx; //to save the index of the connection on which the current configuration is
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,current_conn_idx);

  admissible_other_paths_.clear();
  reset_other_paths = addAdmissibleCurrentPath(current_conn_idx, admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  for(const PathPtr& path: admissible_other_paths_)
  {
    if(path->getConnections().back()->getCost() != std::numeric_limits<double>::infinity())
      no_available_paths = false;
  }

  NodePtr parent = current_conn->getParent();
  NodePtr current_node = std::make_shared<Node>(current_configuration_);
  NodePtr child = current_conn->getChild();

  ConnectionPtr current2child_conn = nullptr;
  double current2child_conn_cost = 0;

  std::vector<ConnectionPtr> replanned_path_conn;

  if(current_conn_idx<current_path_->getConnections().size()-1)
  {
    subpath1 = current_path_->getSubpathFromNode(child);
    subpath1_conn =  subpath1->getConnections();
    if(subpath1->cost() == std::numeric_limits<double>::infinity())
      an_obstacle_ = true;
  }

  if(current_configuration_ != child->getConfiguration() && current_configuration_ != parent->getConfiguration())
  {
    current2child_conn = std::make_shared<Connection>(current_node,child);

    if(current_conn->getCost() == std::numeric_limits<double>::infinity())
    {
      if(!checker_->checkConnection(current2child_conn))
      {
        current2child_conn_cost = std::numeric_limits<double>::infinity();
        an_obstacle_ = true;
      }
      else
        current2child_conn_cost = metrics_->cost(current_node,child);
    }
    else
      current2child_conn_cost = metrics_->cost(current_node,child);

    current2child_conn->setCost(current2child_conn_cost);
    //current2child_conn->add(); not add the connection!
    replanned_path_conn.push_back(current2child_conn);

    if(!subpath1_conn.empty())
      replanned_path_conn.insert(replanned_path_conn.end(),subpath1_conn.begin(),subpath1_conn.end());
  }
  else if(current_configuration_ == parent->getConfiguration())
  {
    current2child_conn = current_conn;
    replanned_path_conn.push_back(current2child_conn);

    if(!subpath1_conn.empty())
      replanned_path_conn.insert(replanned_path_conn.end(),subpath1_conn.begin(),subpath1_conn.end());
  }
  else
  {
    if(!subpath1_conn.empty())
      replanned_path_conn = subpath1_conn;
    else
    {
      success_ = false;
      return success_;
    }
  }

  replanned_path = std::make_shared<Path>(replanned_path_conn,metrics_,checker_); //at the start, the replanned path is initialized with the subpath of the current path from the current config to GOAL
  replanned_path_cost = replanned_path->cost();

  path1_node_vector = startingNodesForPathSwitch(subpath1_conn,current_node,current2child_conn_cost,current_conn_idx,available_nodes);
  int j = path1_node_vector.size()-1;

  while(j>=0)
  {
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO_STREAM("j: "<<j);

    if(informedOnlineReplanning_disp_)
    {
      /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
      disp_->changeNodeSize(marker_scale_sphere);
      node_id = disp_->displayNode(path1_node_vector.at(j),"pathplan",marker_color_sphere_analizing);
      disp_->defaultNodeSize();
      /*/////////////////////////////////////////////////////////////////////////////////////////////*/
    }

    tic_cycle = ros::WallTime::now();

    simplifyAdmissibleOtherPaths(no_available_paths,confirmed_subpath_from_path2,confirmed_connected2path_number,path1_node_vector.at(j),reset_other_paths);

    if(pathSwitch_cycle_time_mean_ >= 0.8*max_time) pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();  //reset

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME - (toc-tic).toSec();
    double min_time_pathSwitch;
    if(informedOnlineReplanning_disp_) min_time_pathSwitch = std::numeric_limits<double>::infinity();
    else if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity()) min_time_pathSwitch = 0.0;
    else min_time_pathSwitch = time_percentage_variability_*pathSwitch_cycle_time_mean_;

    if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("available time: "<<available_time_<<", min required time to call PathSwitch: "<<min_time_pathSwitch);

    if(available_time_>=min_time_pathSwitch)
    {
      if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
        ROS_INFO_STREAM("Launching PathSwitch...");

      solved = pathSwitch(replanned_path, path1_node_vector.at(j), new_path, subpath_from_path2, connected2path_number);
    }
    else
    {
      solved = false;
      exit = true;

      if(informedOnlineReplanning_verbose_)
        ROS_INFO_STREAM("Not eanough time to call PathSwitch or Connect2Goal, available time: "<<available_time_<<" min time to call ps: "<<min_time_pathSwitch);
    }

    path1_node_vector.at(j)->setAnalyzed(true);  //to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
    examined_nodes_.push_back(path1_node_vector.at(j));  //to save the analyzed nodes

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      ROS_INFO_STREAM("Solved: "<<solved);

    if(informedOnlineReplanning_disp_)
    {
      /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
      disp_->clearMarker(node_id);
      disp_->changeNodeSize(marker_scale_sphere);
      node_id = disp_->displayNode(examined_nodes_.back(),"pathplan",marker_color_sphere);
      disp_->defaultNodeSize();
      /*/////////////////////////////////////////////////////////////////////////////////////////////*/
    }

    if(solved)
    {
      PathPtr path;
      PathPtr subpath;
      std::vector<ConnectionPtr> path_conn;

      if(available_nodes) //calculating the cost of the replanned path found
      {
        if(path1_node_vector.at(j)->getConfiguration() != child->getConfiguration() && path1_node_vector.at(j)->getConfiguration() != parent->getConfiguration())
        {
          if(current2child_conn != nullptr)
            path_conn.push_back(current2child_conn);  //connection between the current config and the child of the current conn

          try
          {
            subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));  //path between the current connection child and the node analyzed now
          }
          catch(std::invalid_argument)
          {
            ROS_INFO_STREAM("curr conf: "<<current_configuration_.transpose()<<" child: "<<child->getConfiguration().transpose()<<" parent: "<<parent->getConfiguration().transpose());
          }

          path_conn.insert(path_conn.end(),subpath->getConnectionsConst().begin(),subpath->getConnectionsConst().end());
          path_conn.insert(path_conn.end(),new_path->getConnectionsConst().begin(),new_path->getConnectionsConst().end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
        else if(path1_node_vector.at(j)->getConfiguration() == parent->getConfiguration())
        {
          path = new_path;

          if(current_configuration_ != parent->getConfiguration())
          {
            throw std::invalid_argument("curr conf dovrebbe essere = al parent");
          }
        }
        else    //the node analyzed is the child of the current connection
        {
          if(current2child_conn != nullptr)
            path_conn.push_back(current2child_conn);

          path_conn.insert(path_conn.end(),new_path->getConnectionsConst().begin(),new_path->getConnectionsConst().end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
      }
      else
        path = new_path;

      if(path->cost()<replanned_path_cost)

        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          ROS_INFO_STREAM("new path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);

        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          time_replanning_ = time_first_sol_;
          first_sol = false;  //false when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = path;
        replanned_path_cost = path->cost();

        if(!no_available_paths)
        {
          confirmed_connected2path_number = connected2path_number;  //to remember the vector index of the path to which the algoithm has created a connection
          confirmed_subpath_from_path2 = subpath_from_path2;        //to save the subpath of the path to which the algoritm has created a connection
        }

        success = true;
        an_obstacle_ = false;

        if(replanned_path->cost() == std::numeric_limits<double>::infinity())
          throw std::invalid_argument("the cost of the path found should be finite");

        if(replanned_path_vector.size()<10) //the algorithm gives as output the vector of the best 10 solutions found
          replanned_path_vector.push_back(replanned_path);

        else
        {
          std::vector<PathPtr> support_vector;
          support_vector.insert(support_vector.begin(),replanned_path_vector.begin()+1,replanned_path_vector.end());
          support_vector.push_back(replanned_path);

          replanned_path_vector = support_vector;
        }

        if(available_nodes == 0 && replanned_path->getConnections().size()>1) // when actual conn is obstructed and a path has been found -> PathSwitch will be called from the nodes of the new path found
        {
          current2child_conn = replanned_path->getConnections().at(0);
          child = current2child_conn->getChild();
          available_nodes = 1;
        }

        if(informedOnlineReplanning_disp_)
        {
          /*//////////////////////////Visualization////////////////////////////////////*/
          disp_->clearMarker(replanned_path_id);
          disp_->changeConnectionSize(marker_scale);
          replanned_path_id = disp_->displayPath(replanned_path,"pathplan",marker_color);
          disp_->defaultConnectionSize();
          /*/////////////////////////////////////////////////////////////////////////*/
        }

        toc = ros::WallTime::now();
        if((toc-tic).toSec()>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
          break;
        }
        else
        {
          if((previous_cost-replanned_path_cost)<0.05*previous_cost) cont = cont+1;
          else cont = 0;
        }
      }
      else
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          ROS_INFO_STREAM("NO better path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);
      }

      toc_cycle = ros::WallTime::now();
      //time_vector.push_back((toc_cycle-tic_cycle).toSec());
      //informedOnlineReplanning_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Solution with cost "<<replanned_path_cost<<" found!->Informed cycle duration: "<<(toc_cycle-tic_cycle).toSec());
    }

    if(success && j == 0)
    {
      if(child->getConfiguration() != current_path_->getWaypoints().back())
      {
        subpath1 = replanned_path->getSubpathFromNode(child);
        path1_node_vector.clear();
        for(unsigned int r=0; r<subpath1->getConnections().size()-1; r++)
        {
          if(subpath1->getConnections().at(r)->getParent()->getAnalyzed() == 0 && subpath1->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), nonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
          {
            // the nodes of the new solution found are added to the set of the nodes to be analyzed
            path1_node_vector.push_back(subpath1->getConnections().at(r)->getParent());
          }
        }
        j = path1_node_vector.size();  //then, j=j-1

        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("NEW J: "<<j-1);
      }
    }

    //if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("informed cycle time mean: "<< informedOnlineReplanning_cycle_time_mean_);

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME-(toc-tic).toSec();

    //if((!an_obstacle_ && available_time_<time_percentage_variability_*informedOnlineReplanning_cycle_time_mean_ && informedOnlineReplanning_cycle_time_mean_ != std::numeric_limits<double>::infinity()) || available_time_<=0.0)
    if(exit || j==0)
    {
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<min_time_pathSwitch);
      //if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<time_percentage_variability_*informedOnlineReplanning_cycle_time_mean_);
      if(informedOnlineReplanning_disp_)
      {
        ROS_INFO("Optimizing...");
        disp_->nextButton();
      }

      double cost_pre_opt = replanned_path->cost();
      ros::WallTime tic_warp = ros::WallTime::now();
      if(success) optimizePath(replanned_path,available_time_*0.95);
      ros::WallTime toc_warp = ros::WallTime::now();
      double cost_opt = replanned_path->cost();

      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Path optimization, max time: "<<available_time_<<" time used: "<<(toc_warp-tic_warp).toSec()<<" previous cost: "<<cost_pre_opt<<" new cost: "<<cost_opt);

      j = -1;
      break;
    }

    j -= 1;

    if(emergency_stop_)
    {
      ROS_WARN("EMERGENCY STOP HANDLED");   //ELIMINA
      emergency_stop_ = false;
      break;
    }

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO("------------------------------------------");
    if(informedOnlineReplanning_disp_) disp_->nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");
  }

  for(unsigned int x=0; x<examined_nodes_.size();x++) examined_nodes_.at(x)->setAnalyzed(0);

  if(success)
  {
    replanned_path_ = replanned_path;
    std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());  //ordered with growing cost
    replanned_paths_vector_ = replanned_path_vector;
    success_ = true;

    toc = ros::WallTime::now();
    time_replanning_ = (toc - tic).toSec();

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost() << " in "<< time_replanning_ << "seconds. Number of sol: " << replanned_path_vector.size());
  }
  else
  {
    success_ = false;
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  available_time_ = MAX_TIME-(toc-tic).toSec();

  return success;
}

bool AIPRO::replan()
{}

}
