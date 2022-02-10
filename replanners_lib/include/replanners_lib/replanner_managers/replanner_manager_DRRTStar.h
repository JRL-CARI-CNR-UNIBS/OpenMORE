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

  bool haveToReplan(const bool path_obstructed);
  void initReplanner();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRTStar(PathPtr &current_path,
                           TreeSolverPtr solver,
                           ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration);
};

}

#endif // REPLANNER_MANAGER_DRRTSTAR_H__
