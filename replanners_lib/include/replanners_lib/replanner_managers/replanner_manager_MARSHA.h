#ifndef REPLANNER_MANAGER_MARSHA_H__
#define REPLANNER_MANAGER_MARSHA_H__

#include <replanners_lib/replanner_managers/replanner_manager_MARS.h>
#include <replanners_lib/replanners/MARSHA.h>

namespace pathplan
{
class ReplannerManagerMARSHA;
typedef std::shared_ptr<ReplannerManagerMARSHA> ReplannerManagerMARSHAPtr;

class ReplannerManagerMARSHA: public ReplannerManagerMARS
{
protected:
  LengthPenaltyMetricsPtr ha_metrics_;

  /**
   * @brief unaware_obstacles_ is a list of obstacles to NOT consider for human-aware planning (eg., static obstacles).
   */
  std::vector<std::string> unaware_obstacles_;

  /**
   * @brief poi_names_ is a list containing the names of the points of interest of the robot structure.
   */
  std::vector<std::string> poi_names_;

  /**
   * @brief obstalces_positions_ is the list of locations of the obstacles to consider for human-aware planning.
   */
  Eigen::Matrix <double,3,Eigen::Dynamic> obstalces_positions_;

  /**
   * @brief updateObstaclesPositions updates the locations of obstacles to consider for human-aware planning.
   * @param world is the message containing world information
   * @return the matrix contatining the obstacles positions
   */
  Eigen::Matrix <double,3,Eigen::Dynamic> updateObstaclesPositions(const moveit_msgs::PlanningSceneWorld& world);

  bool replan() override;
  void initReplanner() override;
  void collisionCheckThread() override;

  /**
   * @brief syncPathCost overrides syncPathCost from ReplannerManagerMARS to download the obstacles_positions_ list into ha_metrics_.
   * This operation is not strictly related to path cost synchronization, but operations prior to replanning can be inserted into this method.
   * This way you don't need to edit replanningThread.
   */
  void syncPathCost() override;

  /**
   * @brief MARSHAadditionalParams reads the ROS-parameter specific for MARSHA
   */
  void MARSHAadditionalParams();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //  ReplannerManagerMARSHA(const PathPtr &current_path,
  //                         const TreeSolverPtr &solver,
  //                         const ros::NodeHandle &nh);

  //  ReplannerManagerMARSHA(const PathPtr &current_path,
  //                         const TreeSolverPtr &solver,
  //                         const ros::NodeHandle &nh,
  //                         std::vector<PathPtr> &other_paths);

  ReplannerManagerMARSHA(const PathPtr &current_path,
                         const TreeSolverPtr &solver,
                         const ros::NodeHandle &nh,
                         const LengthPenaltyMetricsPtr &ha_metrics,
                         std::vector<PathPtr> &other_paths);

  void setMetricsHA(const LengthPenaltyMetricsPtr &ha_metrics)
  {
    ha_metrics_ = ha_metrics;

    if(not poi_names_.empty())
      ha_metrics_->setPoiNames(poi_names_);

    if(replanner_)
      std::static_pointer_cast<MARSHA>(replanner_)->setMetricsHA(ha_metrics_);
  }

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;

};

}

#endif // REPLANNER_MANAGER_MARSHA_H__
