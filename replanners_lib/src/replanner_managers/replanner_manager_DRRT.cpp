#include "replanners_lib/replanner_managers/replanner_manager_DRRT.h"

namespace pathplan
{
ReplannerManagerDRRT::ReplannerManagerDRRT(PathPtr &current_path,
                                           TreeSolverPtr solver,
                                           ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTPtr tmp_solver = std::make_shared<pathplan::RRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerDRRT::connectToReplannedPath()
{
//  replanner_->startReplannedPathFromNewCurrentConf(current_configuration_);
}

bool ReplannerManagerDRRT::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRT::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRT>(configuration_replan_, current_path_replanning_, time_for_repl, solver_);
}

bool ReplannerManagerDRRT::replan()
{
  return replanner_->replan();
}
}
