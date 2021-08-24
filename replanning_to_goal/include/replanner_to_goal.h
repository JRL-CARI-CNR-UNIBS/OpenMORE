#ifndef REPLANNERTOGOAL_H__
#define REPLANNERTOGOAL_H__
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <graph_core/util.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>
#include <graph_core/metrics.h>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/collision_checker.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <mutex>

namespace pathplan
{
class ReplannerToGoal;
typedef std::shared_ptr<ReplannerToGoal> ReplannerToGoalPtr;

class ReplannerToGoal: public std::enable_shared_from_this<ReplannerToGoal>
{
protected:
  Eigen::VectorXd current_configuration_;
  PathPtr current_path_;
  PathPtr replanned_path_;
  TreeSolverPtr solver_;
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  std::mutex mtx_;

  bool success_;
  double max_time_;

  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node_fake, const double &diff_subpath_cost, PathPtr &connecting_path, bool &directly_connected);
  PathPtr concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn, const std::vector<ConnectionPtr>& subpath2, const NodePtr& path1_node, const NodePtr& path2_node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerToGoal(Eigen::VectorXd& current_configuration,
                  PathPtr& current_path,
                  double max_time,
                  const TreeSolverPtr& solver,
                  const MetricsPtr& metrics,
                  const CollisionCheckerPtr& checker,
                  const Eigen::VectorXd& lb,
                  const Eigen::VectorXd& ub);

  PathPtr getReplannedPath()
  {
    mtx_.lock();
    PathPtr path = replanned_path_;
    mtx_.unlock();

    return path;
  }

  void setCurrentPath(const PathPtr& path)
  {
    current_path_ = path;
    success_ = 0;
  }

  void setCurrentConf(const Eigen::VectorXd& q)
  {
    current_configuration_ = q;
    solver_->resetProblem();
    solver_->addStart(std::make_shared<Node>(q));
    success_ = 0;
  }

  Eigen::VectorXd getCurrentConf()
  {
    return current_configuration_;
  }

  void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
  }

  bool getSuccess()
  {
    return success_;
  }

  void startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration);
  bool connect2goal(const PathPtr &current_path, const NodePtr& node);
};
}

#endif // REPLANNERTOGOAL_H
