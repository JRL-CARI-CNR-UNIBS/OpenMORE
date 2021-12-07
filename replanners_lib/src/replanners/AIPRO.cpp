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
  admissible_current_path = NULL;

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

std::vector<PathPtr> AIPRO::sortPathsOnDistance(const NodePtr& node)
{
  std::multimap<double,PathPtr> path_map;
  for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)
  {
    double distance;
    admissible_other_paths_.at(j)->findCloserNode(node,distance);
    path_map.insert(std::pair<double,PathPtr>(distance,admissible_other_paths_.at(j)));
  }

  std::vector<PathPtr> ordered_paths;
  for(const std::pair<double,PathPtr>& p: path_map)
  {
    ordered_paths.push_back(p.second);
  }

  return ordered_paths;
}

std::vector<NodePtr> AIPRO::nodes2connect2(const PathPtr& path, const NodePtr& this_node)
{
  std::vector<NodePtr> path_nodes = path->getNodes();
  std::multimap<double,NodePtr> node_map;

  double distance;
  for(const NodePtr node:path_nodes)
  {
    distance = (this_node->getConfiguration()-node->getConfiguration()).norm();
    node_map.insert(std::pair<double,NodePtr>(distance,node));
  }

  std::vector<NodePtr> nodes;
  for(const std::pair<double,NodePtr>& p: node_map)
  {
    nodes.push_back(p.second);
  }

  return nodes;
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
  struct connecting_path_and_subpath
  {
    std::vector<ConnectionPtr> connecting_conns;
    PathPtr subpath;
  };

  std::multimap<double,std::vector<ConnectionPtr>> tmp_map;
  std::multimap<double,connecting_path_and_subpath> solutions;
  std::vector<NodePtr> path_nodes;
  PathPtr subpath, solution;
  std::vector<ConnectionPtr> solution_conns;
  double best_cost;

  NodePtr goal = current_path_->getNodes().back();

  solution = current_path_->getSubpathFromNode(start_node);
  best_cost = solution->cost();

  for(const PathPtr& path:admissible_other_paths_)
  {
    path_nodes = path->getNodes();

    for(const NodePtr& path_node:path_nodes)
    {
      double subpath_cost = 0;
      if(path_node != goal)
      {
        subpath = path->getSubpathFromNode(path_node);
        subpath_cost = subpath->cost();
      }

      tmp_map = net_->getNetConnectionBetweenNodes(start_node,path_node);

      if(!tmp_map.empty())
      {
        for(const std::pair<double,std::vector<ConnectionPtr>> tmp_pair:tmp_map)
        {
          if(tmp_pair.second.at(0) != solution->getConnections().at(0))
          {
            std::pair<double,connecting_path_and_subpath> solution_pair;
            solution_pair.first = tmp_pair.first + subpath_cost;
            solution_pair.second.connecting_conns = tmp_pair.second;
            solution_pair.second.subpath = subpath;

            solutions.insert(solution_pair);
          }
        }
      }
    }
  }

  for(const std::pair<double,connecting_path_and_subpath>& solution_pair:solutions)
  {
    if(solution_pair.first<best_cost)
    {
      bool free = true;
      for(unsigned int i=0; i<solution_pair.second.connecting_conns.size();i++)
      {
        ConnectionPtr conn = solution_pair.second.connecting_conns.at(i);
        if(i<solution_pair.second.connecting_conns.size()-1)
        {
          if(!checker_->checkConnection(conn))
          {
            SubtreePtr subtree = std::make_shared<Subtree>(tree_,start_node);
            NodePtr child = conn->getChild();
            subtree->purgeFromHere(child);

            free = false;
          }
        }
        else
        {
          if(!checker_->checkConnection(conn))
          {
            conn->remove();

            free = false;
          }
        }
      }

      if(free)
      {
        solution_conns = solution_pair.second.connecting_conns;
        if(solution_pair.second.subpath != nullptr)
          solution_conns.insert(solution_conns.end(),solution_pair.second.subpath->getConnectionsConst().begin(), solution_pair.second.subpath->getConnectionsConst().end());
      }
    }
  }

  if(!solution_conns.empty())
    solution = std::make_shared<Path>(solution_conns,metrics_,checker_);

  return solution;
}


void AIPRO::simplifyAdmissibleOtherPaths(const bool& no_available_paths, const PathPtr& confirmed_subpath_from_path2, const int& confirmed_connected2path_number, const NodePtr& starting_node_of_pathSwitch, const std::vector<PathPtr>& reset_other_paths)
{
  if(confirmed_subpath_from_path2 && !no_available_paths)
  {
    std::vector<Eigen::VectorXd> node_vector = confirmed_subpath_from_path2->getWaypoints();
    node_vector.pop_back();  // removing the goal from the vector

    int pos = -1;
    for (unsigned int k=0; k<node_vector.size(); k++)
    {
      if(starting_node_of_pathSwitch->getConfiguration() == node_vector.at(k))
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
  {
    ROS_INFO_STREAM("max opt time: "<<max_time<<" used time: "<<(toc_opt-tic_opt).toSec());
  }
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

bool AIPRO::computeConnectingPath(const NodePtr& path1_node, const NodePtr& path2_node, const double& diff_subpath_cost, const ros::WallTime& tic, const ros::WallTime& tic_cycle, PathPtr& connecting_path, bool& direct_connection)
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

  direct_connection = solver_->solved();

  if(pathSwitch_verbose_)
  {
    if(direct_connection)
      ROS_INFO("Solved->direct connection found");
    else
      ROS_INFO("Direct connection NOT found");

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

  if(solver_has_solved) //change the connection to the path2_node_fake with  net connection to path2_node
  {
    std::vector<ConnectionPtr> connections = connecting_path->getConnections();
    ConnectionPtr last_conn = connections.back();

    NetConnectionPtr net_conn = std::make_shared<NetConnection>(last_conn->getParent(),path2_node);
    net_conn->setCost(last_conn->getCost());
    net_conn->add();

    //    connections.at(connections.size()-1) = net_conn;
    *connections.back() = net_conn;
    connecting_path->setConnections(connections);

    std::vector<NodePtr>::iterator it = std::find(subtree->getNodes().begin(), subtree->getNodes().end(), path2_node_fake);
    subtree->removeNode(it);
    last_conn->remove();
  }

  return solver_has_solved;
}

PathPtr AIPRO::concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn,
                                               const std::vector<ConnectionPtr>& subpath2)
{
  std::vector<ConnectionPtr> new_connecting_path_conn = connecting_path_conn;

  if(!subpath2.empty())
    new_connecting_path_conn.insert(new_connecting_path_conn.end(),subpath2.begin(),subpath2.end());

  PathPtr new_path = std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
  new_path->setTree(tree_);

  return new_path;
}

bool AIPRO::pathSwitch(const PathPtr &current_path,
                       const NodePtr &node,
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
  bool time_out = false;

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

  // Identifying the subpath of current_path starting from node
  NodePtr path1_node = node;
  PathPtr path1_node2goal;

  path1_node2goal = current_path->getSubpathFromNode(path1_node);
  double subpath1_cost = path1_node2goal->cost();
  double candidate_solution_cost = subpath1_cost;

  std::vector<PathPtr> ordered_paths = sortPathsOnDistance(path1_node);
  for(const PathPtr& path2:ordered_paths)
  {
    std::vector<NodePtr> path2_node_vector = nodes2connect2(path2,path1_node);
    for(const NodePtr& path2_node : path2_node_vector)
    {
      tic_cycle = ros::WallTime::now();

      PathPtr path2_node2goal = nullptr;
      std::vector<ConnectionPtr> subpath2_conn;
      double subpath2_cost = 0.0;
      if(path2_node->getConfiguration() != current_path->getConnections().back()->getChild()->getConfiguration())
      {
        path2_node2goal = path2->getSubpathFromNode(path2_node);
        subpath2_conn = path2_node2goal->getConnections();
        subpath2_cost = path2_node2goal->cost();
      }

      double diff_subpath_cost = candidate_solution_cost - subpath2_cost; //it is the maximum cost to make the connecting_path convenient
      double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

      if(pathSwitch_disp_)
      {
        int node_n;
        for(unsigned int r=0; r<path2->getWaypoints().size();r++)
        {
          if(path2_node->getConfiguration() == path2->getWaypoints().at(r))
          {
            node_n = r;
          }
        }
        ROS_INFO_STREAM("candidate_solution_cost: "<<candidate_solution_cost<<" subpath2_cost: "<<subpath2_cost);
        ROS_INFO_STREAM("node n: " <<node_n<< " diff_subpath_cost: "<< diff_subpath_cost<<" distance: " << distance_path_node);
      }

      if(diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it is useless to calculate a connecting_path because it surely will not be convenient
      {
        PathPtr connecting_path;
        bool direct_connection = false;
        bool solver_has_solved = computeConnectingPath(path1_node, path2_node, diff_subpath_cost, tic, tic_cycle, connecting_path, direct_connection);

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

          double new_solution_cost = subpath2_cost + connecting_path->cost();
          if(pathSwitch_verbose_ || pathSwitch_disp_)
            ROS_INFO_STREAM("solution cost: "<<new_solution_cost);

          if(new_solution_cost<candidate_solution_cost)
          {
            std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();
            new_path = concatConnectingPathAndSubpath2(connecting_path_conn,subpath2_conn);
            candidate_solution_cost = new_path->cost();

            success = 1;
            an_obstacle_ = false;
            subpath_from_path2 = path2_node2goal;
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
            if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("It is not a better solution");

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

          if(!direct_connection)  //not directly connected, usually it is very fast and it would alterate the mean value
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

        //Regardless if you have found a solution or not, delete the fake nodes
        //        path1_node_fake->disconnect();
        path2_node_fake->disconnect();
      }
      else
      {
        if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("It would not be a better solution");

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
        time_out = true;
        break;
      }
    }
    if(time_out)
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
    {
      ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
    }
    else
    {
      ROS_INFO_STREAM("PathSwitch has NOT found a solution");
    }
  }

  return success;
}

bool AIPRO::replan()
{}

}
