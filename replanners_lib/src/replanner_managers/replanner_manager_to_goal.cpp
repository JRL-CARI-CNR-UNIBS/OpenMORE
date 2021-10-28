#include "replanners_lib/replanner_managers/replanner_manager_to_goal.h"

namespace pathplan
{

ReplannerManagerToGoal::ReplannerManagerToGoal(PathPtr &current_path,
                                               TreeSolverPtr solver,
                                               ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTPtr tmp_solver = std::make_shared<pathplan::RRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_ = tmp_solver;

  additionalParams();
}

void ReplannerManagerToGoal::additionalParams()
{
  if(!nh_.getParam("/to_goal/n_threads_replan",n_threads_replan_))
  {
    ROS_ERROR("n_threads_replan not set, set 5");
    n_threads_replan_ = 5;
  }
  else
  {
    if(n_threads_replan_<1)
    {
      ROS_ERROR("n_threads_replan can not be less than 1, set 1");
      n_threads_replan_ = 1;
    }
  }
}

void ReplannerManagerToGoal::connectToReplannedPath()
{
  replanner_->startReplannedPathFromNewCurrentConf(current_configuration_);
}

bool ReplannerManagerToGoal::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerToGoal::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::ReplannerToGoal>(configuration_replan_, current_path_replanning_, time_for_repl, solver_,n_threads_replan_);
}

bool ReplannerManagerToGoal::replan()
{
  return replanner_->replan();
}
}
