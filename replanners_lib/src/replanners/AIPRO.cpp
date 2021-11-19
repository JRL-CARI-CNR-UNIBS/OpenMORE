#include "replanners_lib/replanners/AIPRO.h"

namespace pathplan
{

AIPRO::AIPRO(const Eigen::VectorXd& current_configuration,
             const PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
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
             const std::vector<PathPtr>& other_paths,
             const double& max_time,
             const TreeSolverPtr &solver): AIPRO(current_configuration,current_path,max_time,solver)
{
  other_paths_ = other_paths;
  admissible_other_paths_ = other_paths_;
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

bool AIPRO::replan()
{}

}
