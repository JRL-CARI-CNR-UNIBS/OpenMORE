#ifndef REPLANNERTOGOAL_H__
#define REPLANNERTOGOAL_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/solvers/rrt.h>
#include <future>
#include <mutex>

//High-frequency replanning under uncertainty using parallel sampling-based motion planning

namespace pathplan
{
class ReplannerToGoal;
typedef std::shared_ptr<ReplannerToGoal> ReplannerToGoalPtr;

class ReplannerToGoal: public ReplannerBase
{
protected:
  unsigned int number_of_parallel_plannings_;
  std::vector<RRTPtr> solver_vector_;
  std::vector<PathPtr> connecting_path_vector_;
  std::vector<bool> directly_connected_vector_;
  std::vector<NodePtr> node_to_delete_;
  std::mutex mtx_;

  PathPtr concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn, const NodePtr& path1_node);
  bool asyncComputeConnectingPath(const Eigen::VectorXd path1_node_conf, const Eigen::VectorXd path2_node_conf, const double diff_subpath_cost, const int index);
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node_fake, const double &diff_subpath_cost, PathPtr &connecting_path, bool &directly_connected, TreeSolverPtr &solver);
  bool connect2goal(const NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerToGoal(Eigen::VectorXd& current_configuration,
                  PathPtr& current_path,
                  const double& max_time,
                  const TreeSolverPtr& solver,
                  const unsigned int& number_of_parallel_plannings = 1);

  bool replan() override;
};
}

#endif // REPLANNERTOGOAL_H
