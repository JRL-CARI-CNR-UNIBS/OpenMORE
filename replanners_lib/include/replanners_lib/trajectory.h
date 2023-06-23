#ifndef TRAJECTORY_18_5_2022_H__
#define TRAJECTORY_18_5_2022_H__

#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/local_informed_sampler.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <unordered_map>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <replanners_lib/moveit_utils.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);

namespace pathplan
{
class Trajectory;
typedef std::shared_ptr<Trajectory> TrajectoryPtr;

class Trajectory: public std::enable_shared_from_this<Trajectory>
{
protected:

  robot_trajectory::RobotTrajectoryPtr trj_;
  PathPtr path_;
  ros::NodeHandle nh_;
  robot_model::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;  //REMOVE
  std::string group_name_;
  MoveitUtilsPtr moveit_utils_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Trajectory(const pathplan::PathPtr path,
             const ros::NodeHandle& nh,
             const planning_scene::PlanningScenePtr& planning_scene,
             const std::string& group_name);

  Trajectory(const ros::NodeHandle& nh,
             const planning_scene::PlanningScenePtr& planning_scene,
             const std::string& group_name);

  TrajectoryPtr pointer()
  {
    return shared_from_this();
  }

  void setPath(const pathplan::PathPtr& path)
  {
    if(path == nullptr)
      throw std::runtime_error("path is nullptr");

    path_ = path;
  }

  pathplan::PathPtr getPath()
  {
    if(path_ == nullptr)
      ROS_ERROR("Path not computed");

    return path_;
  }

  robot_trajectory::RobotTrajectoryPtr getTrj()
  {
    if(trj_ == nullptr)
      ROS_ERROR("Trj not computed");

    return trj_;
  }

  PathPtr computePath(const NodePtr &start_node, const NodePtr &goal_node, const TreeSolverPtr& solver, const bool& optimize = true, const double &max_time = std::numeric_limits<double>::infinity());
  PathPtr computePath(const Eigen::VectorXd &start_conf, const Eigen::VectorXd &goal_conf, const TreeSolverPtr& solver, const bool& optimizePath = true, const double &max_time = std::numeric_limits<double>::infinity());

  robot_trajectory::RobotTrajectoryPtr fromPath2Trj(const trajectory_msgs::JointTrajectoryPointPtr& pnt = nullptr);
  robot_trajectory::RobotTrajectoryPtr fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint& pnt);

  double getTimeFromTrjPoint(const Eigen::VectorXd &trj_point, const int &n_interval = 10, const int &spline_order = 1);
};
}

#endif // TRAJECTORY_H
