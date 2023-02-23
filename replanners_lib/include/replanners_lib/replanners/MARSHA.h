#ifndef MARSHA_H__
#define MARSHA_H__
#include <replanners_lib/replanners/MARS.h>
#include <length_penalty_metrics.h>
#include <ssm15066_estimators/parallel_ssm15066_estimator2D.h>

namespace pathplan
{
class MARSHA;
typedef std::shared_ptr<MARSHA> MARSHAPtr;

class MARSHA: public MARS
{
protected:

  /**
   * @brief expensive_cost_ is used to understand if a connection has been penalized a lot by the ssm
   */
  static constexpr double expensive_cost_ = 10.0;

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
   * @brief ha_metrics_ is the human-aware metrics
   */
  LengthPenaltyMetricsPtr ha_metrics_;

  /**
   * @brief euclidean_metrics_ used by the solver to compute the connecting path
   * If and only if you are not using an optimal planner, using the aware metrics during path search is useless (time consuming);
   * you can use the faster euclidean metrics to build the tree and then the aware metrics to evaluate the cost of the solution found
   */
  MetricsPtr euclidean_metrics_;

  void updateHACost(const PathPtr& path);
  void init(const LengthPenaltyMetricsPtr& ha_metrics);

  void initFlaggedConnections() override;
//  void clearInvalidConnections() override;
  void clearFlaggedConnections() override;
  std::vector<ps_goal_ptr> sortNodes(const NodePtr& node) override;
  std::vector<NodePtr> startNodes(const std::vector<ConnectionPtr>& subpath1_conn) override;
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost,
                             const PathPtr &current_solution, const ros::WallTime &tic, const ros::WallTime &tic_cycle,
                             PathPtr &connecting_path, bool &quickly_solved) override;
  bool findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double& cost2beat,
                         std::vector<ConnectionPtr>& solution, double &cost, unsigned int &number_of_candidates, bool verbose = false) override;

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
