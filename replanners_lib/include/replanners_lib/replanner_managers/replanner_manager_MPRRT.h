#ifndef REPLANNER_MANAGER_MPRRT_H__
#define REPLANNER_MANAGER_MPRRT_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/MPRRT.h>

namespace pathplan
{
class ReplannerManagerMPRRT;
typedef std::shared_ptr<ReplannerManagerMPRRT> ReplannerManagerMPRRTPtr;

class ReplannerManagerMPRRT: public ReplannerManagerBase
{
protected:
  int n_threads_replan_;

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;
  void additionalParams();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerMPRRT(const PathPtr &current_path,
                         const TreeSolverPtr &solver,
                         const ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration) override;
};

}

#endif // REPLANNER_MANAGER_MPRRT_H__
