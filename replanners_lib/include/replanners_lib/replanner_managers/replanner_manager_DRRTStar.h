#ifndef REPLANNER_MANAGER_DRRTSTAR_H__
#define REPLANNER_MANAGER_DRRTSTAR_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/DRRTStar.h>

namespace pathplan
{
class ReplannerManagerDRRTStar;
typedef std::shared_ptr<ReplannerManagerDRRTStar> ReplannerManagerDRRTStarPtr;

class ReplannerManagerDRRTStar: public ReplannerManagerBase
{
protected:
  NodePtr old_current_node_ = nullptr;
  bool is_a_new_node_;

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRTStar(const PathPtr &current_path,
                           const TreeSolverPtr &solver,
                           const ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration) override;
};

}

#endif // REPLANNER_MANAGER_DRRTSTAR_H__
