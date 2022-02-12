#include "replanners_lib/replanner_managers/replanner_manager_AIPRO.h"

namespace pathplan
{
ReplannerManagerAIPRO::ReplannerManagerAIPRO(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
}

void ReplannerManagerAIPRO::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  return;
}

bool ReplannerManagerAIPRO::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerAIPRO::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::AIPRO>(configuration_replan_, current_path_replanning_, time_for_repl, solver_);
}


void ReplannerManagerAIPRO::replanningThread()
{
  return;
}
void ReplannerManagerAIPRO::collisionCheckThread()
{
  return;
}

}
