#ifndef MARSHA_H__
#define MARSHA_H__
#include <replanners_lib/replanners/MARS.h>
#include <length_penalty_metrics.h>

namespace pathplan
{

class MARSHA;
typedef std::shared_ptr<MARSHA> MARSHAPtr;

class MARSHA: public MARS
{
protected:

  /**
   * @brief cost_updated_flag_ is a custom flag for Connection class which allows to keep track of the connections
   * for which the cost has been already updated during this iteration
   */
  unsigned int cost_updated_flag_;

  /**
   * @brief cost_evaluation_condition_ is a pointer to the function passed to net_ to decide when connections cost must be re-evaluated.
   */
  std::shared_ptr<std::function<bool (const ConnectionPtr& connection)>> cost_evaluation_condition_;

  /**
   * @brief starting_time_ is the time at the beginning of the replanning. It is used to check if the cost of a connection was recently updated or not
   */
  double starting_time_;

  /**
   * @brief ha_metrics_ is the human-aware metrics
   */
  LengthPenaltyMetricsPtr ha_metrics_;

  /**
   * @brief euclidean_metrics_ used by the solver to compute the connecting path
   * If and only if you are not using an optimal planner, using the aware metrics is useless; you can use the faster euclidean metrics to build the tree
   * and then the aware metrics to evaluate the cost of the solution found
   */
  MetricsPtr euclidean_metrics_;

  void updateHACost(const PathPtr& path);
  void init(const LengthPenaltyMetricsPtr& ha_metrics);

  void initFlaggedConnections() override;
  void clearFlaggedConnections() override;
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost,
                             const PathPtr &current_solution, const ros::WallTime &tic, const ros::WallTime &tic_cycle,
                             PathPtr &connecting_path, bool &quickly_solved) override;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MARSHA(const Eigen::VectorXd& current_configuration,
         const PathPtr& current_path,
         const double& max_time,
         const TreeSolverPtr &solver);

  MARSHA(const Eigen::VectorXd& current_configuration,
         const PathPtr& current_path,
         const double& max_time,
         const TreeSolverPtr &solver,
         const std::vector<PathPtr> &other_paths);

  MARSHA(const Eigen::VectorXd& current_configuration,
         const PathPtr& current_path,
         const double& max_time,
         const TreeSolverPtr &solver,
         const std::vector<PathPtr> &other_paths,
         const LengthPenaltyMetricsPtr &ha_metrics);

  void setMetricsHA(const LengthPenaltyMetricsPtr& ha_metrics);
  bool addObstaclePosition(const Eigen::Vector3d& obstacle_position);
  bool setObstaclesPosition(const Eigen::Matrix<double,3,Eigen::Dynamic>& obstacles_positions);

  void addOtherPath(const PathPtr& path, bool merge_tree = true) override
  {
    MARS::addOtherPath(path,merge_tree);
    path->setMetrics(ha_metrics_);
  }

  bool replan() override;
  void setCurrentPath(const PathPtr& path) override;
  void setOtherPaths(const std::vector<PathPtr> &other_paths, const bool merge_tree = true);
};
}

#endif // MARSHA_H
