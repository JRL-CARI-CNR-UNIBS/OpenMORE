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

  virtual bool haveToReplan(const bool path_obstructed);
  virtual void initReplanner();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRT(PathPtr &current_path,
                       TreeSolverPtr solver,
                       ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration);
};

}

#endif // REPLANNER_MANAGER_DRRT_H__
