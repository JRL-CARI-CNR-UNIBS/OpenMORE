#include "replanners_lib/moveit_utils.h"

namespace pathplan
{

MoveitUtils::MoveitUtils(const planning_scene::PlanningScenePtr &planning_scene,
                         const std::string &group_name)
{
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
}

std::vector<moveit::core::RobotState> MoveitUtils::fromWaypoints2State(const std::vector<Eigen::VectorXd> waypoints) const
{
  std::vector<moveit::core::RobotState> wp_state_vector;
  for(const Eigen::VectorXd& waypoint: waypoints)
  {
    moveit::core::RobotState wp_state=fromWaypoints2State(waypoint);
    wp_state_vector.push_back(wp_state);
  }
  return wp_state_vector;
}

moveit::core::RobotState MoveitUtils::fromWaypoints2State(const Eigen::VectorXd& waypoint) const
{
  moveit::core::RobotState wp_state=planning_scene_->getCurrentState();
  wp_state.setJointGroupPositions(group_name_,waypoint);
  wp_state.update();

  return wp_state;
}

}
