#ifndef REPLANNER_MANAGER_DRRT_H__
#define REPLANNER_MANAGER_DRRT_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/DRRT.h>

namespace pathplan
{
class ReplannerManagerDRRT;
typedef std::shared_ptr<ReplannerManagerDRRT> ReplannerManagerDRRTPtr;

class ReplannerManagerDRRT: public ReplannerManagerBase
{
protected:

  bool haveToReplan(const bool path_osbtructed);
  bool replan();
  void initReplanner();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRT(PathPtr &current_path,
                       ros::NodeHandle &nh):ReplannerManagerBase(current_path,nh){}

};

}

#endif // REPLANNER_MANAGER_DRRT_H__
