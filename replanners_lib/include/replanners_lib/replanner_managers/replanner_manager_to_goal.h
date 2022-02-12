#ifndef REPLANNER_MANAGER_TO_GOAL_H__
#define REPLANNER_MANAGER_TO_GOAL_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/replanner_to_goal.h>

namespace pathplan
{
class ReplannerManagerToGoal;
typedef std::shared_ptr<ReplannerManagerToGoal> ReplannerManagerToGoalPtr;

class ReplannerManagerToGoal: public ReplannerManagerBase
{
protected:
  int n_threads_replan_;

  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;
  void additionalParams();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerToGoal(const PathPtr &current_path,
                         const TreeSolverPtr &solver,
                         const ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration) override;
};

}

#endif // REPLANNER_MANAGER_TO_GOAL_H__
