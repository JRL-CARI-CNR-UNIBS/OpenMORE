#ifndef REPLANNER_MANAGER_AIPRO_H__
#define REPLANNER_MANAGER_AIPRO_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/AIPRO.h>
#include <future>

namespace pathplan
{
class ReplannerManagerAIPRO;
typedef std::shared_ptr<ReplannerManagerAIPRO> ReplannerManagerAIPROPtr;

class ReplannerManagerAIPRO: public ReplannerManagerBase
{
protected:
  bool first_replanning_;
  bool current_path_cost_update_ready_;
  bool other_paths_cost_update_ready_;
  double updating_cost_pause_;
  double dt_replan_relaxed_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> other_paths_shared_;
  std::mutex other_paths_mtx_;

  bool checkPathTask(const PathPtr& path);
  void checkCurrentPath();
  void checkOtherPaths();
  void fromParam() override;
  void syncPathCost() override;
  void updatePathCost(const PathPtr& current_path_updated_copy) override;
  void updateOtherPathsCost(const std::vector<PathPtr>& other_paths_updated_copy);
  void attributeInitialization() override;
  void collisionCheckThread() override;
  bool replan() override;
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
