#include "replanners_lib/replanner_managers/replanner_manager_to_goal.h"

namespace pathplan
{

ReplannerManagerToGoal::ReplannerManagerToGoal(PathPtr &current_path,
                                               ros::NodeHandle &nh):ReplannerManagerBase(current_path,nh)
{
  additionalParams();
}

void ReplannerManagerToGoal::additionalParams()
{
  if(!nh_.getParam("/to_goal/n_threads_replan",n_threads_replan_))
  {
    ROS_ERROR("n_thread_replan not set, set 5");
    n_threads_replan_ = 5;
  }
  else
  {
    if(n_threads_replan_<1)
    {
      ROS_ERROR("n_thread_replan can not be less than 1, set 1");
      n_threads_replan_ = 1;
    }
  }
}

bool ReplannerManagerToGoal::haveToReplan(const bool path_osbtructed)
{
  return replanIfObstructed(path_osbtructed);
}

void ReplannerManagerToGoal::initReplanner()
{
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb_, ub_);
  pathplan::RRTPtr     solver  = std::make_shared<pathplan::RRT>(metrics, checker_, sampler);
  solver->config(nh_);

  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::ReplannerToGoal>(configuration_replan_, current_path_, time_for_repl, solver,n_threads_replan_);
}

bool ReplannerManagerToGoal::replan()
{
  return replanner_->replan();
}
}
