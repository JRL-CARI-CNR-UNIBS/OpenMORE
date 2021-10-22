#include "replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h"

namespace pathplan
{

bool ReplannerManagerAnytimeDRRT::haveToReplan(const bool path_osbtructed)
{
  return alwaysReplan();
}

void ReplannerManagerAnytimeDRRT::initReplanner()
{
  pathplan::MetricsPtr    metrics = std::make_shared<pathplan::Metrics>();
  pathplan::SamplerPtr    sampler = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb_, ub_);
  pathplan::AnytimeRRTPtr solver  = std::make_shared<pathplan::AnytimeRRT>(metrics, checker_, sampler);
  solver->config(nh_);

  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::AnytimeDynamicRRT>(configuration_replan_, current_path_, time_for_repl, solver);
}

bool ReplannerManagerAnytimeDRRT::replan()
{
  return replanner_->replan();
}

}
