#ifndef REPLANNER_MANAGER_ANYTIMEDRRT_H__
#define REPLANNER_MANAGER_ANYTIMEDRRT_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/anytimeDRRT.h>

namespace pathplan
{
class ReplannerManagerAnytimeDRRT;
typedef std::shared_ptr<ReplannerManagerAnytimeDRRT> ReplannerManagerAnytimeDRRTPtr;

class ReplannerManagerAnytimeDRRT: public ReplannerManagerBase
{
protected:

  bool haveToReplan(const bool path_osbtructed);
  bool replan();
  void initReplanner();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerAnytimeDRRT(PathPtr &current_path,
                              ros::NodeHandle &nh):ReplannerManagerBase(current_path,nh){}

};

}

#endif // REPLANNER_MANAGER_ANYTIMEDRRT_H__
