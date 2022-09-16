#ifndef REPLANNER_MANAGER_MARS_H__
#define REPLANNER_MANAGER_MARS_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/MARS.h>
#include <future>

namespace pathplan
{
class ReplannerManagerMARS;
typedef std::shared_ptr<ReplannerManagerMARS> ReplannerManagerMARSPtr;

class ReplannerManagerMARS: public ReplannerManagerBase
{
protected:
  bool full_net_search_;
  bool first_replanning_;
  bool reverse_start_nodes_;
  int verbosity_level_;
  double dt_replan_relaxed_;
  NodePtr old_current_node_;
  PathPtr initial_path_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> other_paths_shared_;
  std::mutex other_paths_mtx_;
  std::vector<bool> other_paths_sync_needed_;


  bool checkPathTask(const PathPtr& path);
  void displayCurrentPath();
  void displayOtherPaths();
  void additionalParam();
  void syncPathCost() override;
  void updateSharedPath() override;
  void updatePathsCost(const PathPtr& current_path_updated_copy, const std::vector<PathPtr>& other_paths_updated_copy);
  void attributeInitialization() override;
  void collisionCheckThread() override;
  void displayThread() override;
  bool replan() override;
  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerMARS(const PathPtr &current_path,
                        const TreeSolverPtr &solver,
                        const ros::NodeHandle &nh);

  ReplannerManagerMARS(const PathPtr &current_path,
                        const TreeSolverPtr &solver,
                        const ros::NodeHandle &nh,
                        std::vector<PathPtr> &other_paths);

  void setOtherPaths(std::vector<PathPtr>& other_paths);
  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}

#endif // REPLANNER_MANAGER_MARS_H__