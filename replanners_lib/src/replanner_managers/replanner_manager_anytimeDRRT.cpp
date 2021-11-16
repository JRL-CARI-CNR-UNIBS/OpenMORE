#include "replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h"

namespace pathplan
{

ReplannerManagerAnytimeDRRT::ReplannerManagerAnytimeDRRT(PathPtr &current_path,
                                                         TreeSolverPtr solver,
                                                         ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  AnytimeRRTPtr tmp_solver = std::make_shared<pathplan::AnytimeRRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerAnytimeDRRT::connectToReplannedPath()
{
  connectCurrentConfToTree();
}

bool ReplannerManagerAnytimeDRRT::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();

//  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerAnytimeDRRT::initReplanner()
{  
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::AnytimeDynamicRRT>(configuration_replan_, current_path_replanning_, time_for_repl, solver_);

}

bool ReplannerManagerAnytimeDRRT::replan()
{
  return replanner_->replan();
}

}
