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

  bool replan();
  bool haveToReplan(const bool path_obstructed);
  void initReplanner();
  void additionalParams();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerToGoal(PathPtr &current_path,
                         ros::NodeHandle &nh);

};

}

#endif // REPLANNER_MANAGER_TO_GOAL_H__
