#ifndef REPLANNERBASE_H__
#define REPLANNERBASE_H__

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/connection.h>
#include <graph_core/collision_checker.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/path_solver.h>

namespace pathplan
{
class ReplannerBase;
typedef std::shared_ptr<ReplannerBase> ReplannerBasePtr;

class ReplannerBase: public std::enable_shared_from_this<ReplannerBase>
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
  DisplayPtr disp_;
  NodePtr goal_node_;

  bool is_a_new_node_;
  bool success_;
  bool verbose_;
  double max_time_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerBase(const Eigen::VectorXd &current_configuration,
                const PathPtr &current_path,
                const double &max_time,
                const TreeSolverPtr& solver);
  ~ReplannerBase();

  PathPtr getReplannedPath() const
  {
    return replanned_path_;
  }

  PathPtr getCurrentPath() const
  {
    return current_path_;
  }

  virtual void setCurrentPath(const PathPtr& path)
  {
    success_ = false;
    current_path_ = path;
    goal_node_  = current_path_->getConnections().back()->getChild();
  }

  void setReplannedPath(const PathPtr& path)
  {
    replanned_path_ = path;
  }

  void setMaxTime(const double& max_time)
  {
    max_time_ = max_time;
  }

  virtual void setVerbosity(const bool& verbose)
  {
    verbose_ = verbose;
  }

  virtual void setCurrentConf(const Eigen::VectorXd& q)
  {
    current_configuration_ = q;
    success_ = false;
  }

  NodePtr getGoal() const
  {
    return goal_node_;
  }

  Eigen::VectorXd getCurrentConf() const
  {
    return current_configuration_;
  }

  TreeSolverPtr getSolver() const
  {
    return solver_;
  }

  virtual void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
    solver_->setChecker(checker);
    current_path_->setChecker(checker);

    if(replanned_path_)
      replanned_path_->setChecker(checker);
  }

  void setDisp(const DisplayPtr &disp)
  {
    disp_ = disp;
  }

  DisplayPtr getDisp() const
  {
    return disp_;
  }

  bool replanNodeIsANewNode() const
  {
    return is_a_new_node_;
  }

  bool getSuccess() const
  {
    return success_;
  }

  virtual bool replan() = 0;
};
}

#endif // REPLANNERBASE_H
