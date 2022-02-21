#ifndef REPLANNER_MANAGER_AIPRO_H__
#define REPLANNER_MANAGER_AIPRO_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/AIPRO.h>

namespace pathplan
{
class ReplannerManagerAIPRO;
typedef std::shared_ptr<ReplannerManagerAIPRO> ReplannerManagerAIPROPtr;

class ReplannerManagerAIPRO: public ReplannerManagerBase
{
protected:
  std::vector<PathPtr> other_paths_;

  void replanningThread() override;
  void collisionCheckThread() override;
  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerAIPRO(const PathPtr &current_path,
                        const TreeSolverPtr &solver,
                        const ros::NodeHandle &nh);

  ReplannerManagerAIPRO(const PathPtr &current_path,
                        const TreeSolverPtr &solver,
                        const ros::NodeHandle &nh,
                        std::vector<PathPtr> &other_paths);

  void setOtherPaths(std::vector<PathPtr>& other_paths)
  {
    other_paths_ = other_paths;
    if(replanner_)
    {
      AIPROPtr aipro_replanner = std::static_pointer_cast<AIPRO>(replanner_);
      aipro_replanner->setOtherPaths(other_paths);
    }
  }
  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}

#endif // REPLANNER_MANAGER_AIPRO_H__
