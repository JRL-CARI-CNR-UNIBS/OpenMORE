#ifndef MOVEIT_UTILS_10_4_2021_H__
#define MOVEIT_UTILS_10_4_2021_H__

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);

namespace pathplan
{
class MoveitUtils;
typedef std::shared_ptr<MoveitUtils> MoveitUtilsPtr;

class MoveitUtils: public std::enable_shared_from_this<MoveitUtils>
{
protected:
  robot_model::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::string group_name_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MoveitUtils(const planning_scene::PlanningScenePtr& planning_scene,
              const std::string& group_name);

  MoveitUtilsPtr pointer()
  {
    return shared_from_this();
  }

  std::vector<moveit::core::RobotState> fromWaypoints2State(const std::vector<Eigen::VectorXd> waypoints) const;
  moveit::core::RobotState fromWaypoints2State(const Eigen::VectorXd &waypoint) const;

};
}

#endif // MOVEIT_UTILS_H
