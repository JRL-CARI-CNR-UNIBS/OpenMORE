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
   * @brief starting_time_ is the time at the beginning of the replanning. It is used to check if the cost of a connection was recently updated or not
   */
  double starting_time_;

  /**
   * @brief ha_metrics_ is the human-aware metrics
   */
  LengthPenaltyMetricsPtr ha_metrics_;

  void init(const LengthPenaltyMetricsPtr& ha_metrics);
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

  bool replan() override;
};
}

#endif // MARSHA_H
