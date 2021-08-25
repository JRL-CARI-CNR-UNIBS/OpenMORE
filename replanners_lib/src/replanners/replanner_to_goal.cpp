#include "replanner_to_goal.h"

namespace pathplan
{

bool ReplannerToGoal::connect2goal(const NodePtr& node)
{
  success_ = false;

  NodePtr node_fake = std::make_shared<Node>(node->getConfiguration());
  NodePtr goal_node_fake = std::make_shared<Node>(current_path_->getWaypoints().back());

  PathPtr connecting_path;
  bool directly_connected;
  bool solver_has_solved = computeConnectingPath(node_fake, goal_node_fake, std::numeric_limits<double>::infinity(), connecting_path, directly_connected);

  if (solver_has_solved)
  {
    std::vector<ConnectionPtr>  connecting_path_conn = connecting_path->getConnections();

    PathPtr new_path = concatWithNewPathToGoal(connecting_path_conn, node);

    replanned_path_ = new_path;
    success_ = true;

    ROS_INFO_STREAM("Solution found! -> cost: " << new_path->cost());
  }
  else
  {
    ROS_ERROR("New path not found!");
  }

  //Regardless if you have found a solution or not, delete the fake nodes
  node_fake->disconnect();
  goal_node_fake->disconnect();

  return success_;
}

PathPtr ReplannerToGoal::concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn,
                                             const NodePtr& path1_node)
{
  NodePtr path2_node = std::make_shared<Node>(current_path_->getWaypoints().back());
  std::vector<ConnectionPtr> subpath2;
  if(!subpath2.empty())
  {
    if(!subpath2.empty()) ROS_ERROR("Subpath2 not empty!");
    assert(0);
  }

  return concatConnectingPathAndSubpath2(connecting_path_conn, subpath2, path1_node, path2_node);
}

bool ReplannerToGoal::replan()
{
  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    connect2goal(node_replan);
  }
  else //replan not needed
  {
    success_ = false;
    replanned_path_ = current_path_;
  }

  return success_;
}

}
