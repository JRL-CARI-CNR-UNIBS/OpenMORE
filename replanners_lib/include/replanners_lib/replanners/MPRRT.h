#ifndef MPRRT_H__
#define MPRRT_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/solvers/rrt.h>
#include <future>
#include <mutex>

//High-frequency replanning under uncertainty using parallel sampling-based motion planning

namespace pathplan
{
class MPRRT;
typedef std::shared_ptr<MPRRT> MPRRTPtr;

class MPRRT: public ReplannerBase
{
protected:
  unsigned int number_of_parallel_plannings_;
  std::vector<RRTPtr> solver_vector_;
  std::vector<PathPtr> connecting_path_vector_;
  std::mutex mtx_;

  PathPtr concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn, const NodePtr& path1_node);
  bool asyncComputeConnectingPath(const Eigen::VectorXd path1_node_conf, const Eigen::VectorXd path2_node_conf, const double current_solution_cost, const int index);
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node_fake, const double &current_solution_cost, const double max_time, PathPtr &connecting_path, bool &directly_connected, TreeSolverPtr &solver);
  bool connect2goal(const NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MPRRT(Eigen::VectorXd& current_configuration,
                  PathPtr& current_path,
                  const double& max_time,
                  const TreeSolverPtr& solver,
                  const unsigned int& number_of_parallel_plannings = 1);

  bool replan() override;
};
}

#endif // MPRRT_H
