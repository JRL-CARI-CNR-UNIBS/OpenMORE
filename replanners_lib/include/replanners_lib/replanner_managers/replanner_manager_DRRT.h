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

  virtual bool haveToReplan(const bool path_obstructed) override;
  virtual void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerDRRT(const PathPtr &current_path,
                       const TreeSolverPtr &solver,
                       const ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}

#endif // REPLANNER_MANAGER_DRRT_H__
