#include "replanners_lib/replanner_managers/replanner_manager_DRRT.h"

namespace pathplan
{

bool ReplannerManagerDRRT::haveToReplan(const bool path_osbtructed)
{
  return replanIfObstructed(path_osbtructed);
}

void ReplannerManagerDRRT::initReplanner()
{
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb_, ub_);
  pathplan::RRTPtr     solver  = std::make_shared<pathplan::RRT>(metrics, checker_, sampler);
  solver->config(nh_);

  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRT>(configuration_replan_, current_path_, time_for_repl, solver);
}

bool ReplannerManagerDRRT::replan()
{
  return replanner_->replan();
}
}
