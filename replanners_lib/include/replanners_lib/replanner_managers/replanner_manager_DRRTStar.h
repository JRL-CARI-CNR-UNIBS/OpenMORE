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
  bool replan();
  void initReplanner();
  void connectToReplannedPath();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRTStar(PathPtr &current_path,
                           TreeSolverPtr solver,
                           ros::NodeHandle &nh);

};

}

#endif // REPLANNER_MANAGER_DRRTSTAR_H__
