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

  bool haveToReplan(const bool path_osbtructed);
  bool replan();
  void initReplanner();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRTStar(PathPtr &current_path,
                           ros::NodeHandle &nh):ReplannerManagerBase(current_path,nh){}

};

}

#endif // REPLANNER_MANAGER_DRRTSTAR_H__
