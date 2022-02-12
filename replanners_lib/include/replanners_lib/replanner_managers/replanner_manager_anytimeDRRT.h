#ifndef REPLANNER_MANAGER_ANYTIMEDRRT_H__
#define REPLANNER_MANAGER_ANYTIMEDRRT_H__

#include <replanners_lib/replanner_managers/replanner_manager_DRRT.h>
#include <replanners_lib/replanners/anytimeDRRT.h>

namespace pathplan
{
class ReplannerManagerAnytimeDRRT;
typedef std::shared_ptr<ReplannerManagerAnytimeDRRT> ReplannerManagerAnytimeDRRTPtr;

class ReplannerManagerAnytimeDRRT: public ReplannerManagerDRRT
{
protected:

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerAnytimeDRRT(const PathPtr &current_path,
                              const TreeSolverPtr &solver,
                              const ros::NodeHandle &nh);

};

}

#endif // REPLANNER_MANAGER_ANYTIMEDRRT_H__
