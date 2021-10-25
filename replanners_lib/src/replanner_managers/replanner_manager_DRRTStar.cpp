#include "replanners_lib/replanner_managers/replanner_manager_DRRTStar.h"

namespace pathplan
{
ReplannerManagerDRRTStar::ReplannerManagerDRRTStar(PathPtr &current_path,
                                                   TreeSolverPtr solver,
                                                   ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  solver_  = std::make_shared<pathplan::RRTStar>(solver_->getMetrics(), checker_, solver_->getSampler());
  solver_->importFromSolver(solver);
}

bool ReplannerManagerDRRTStar::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRTStar::initReplanner()
{
  //  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  //  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb_, ub_);
  //  pathplan::RRTStarPtr solver  = std::make_shared<pathplan::RRTStar>(metrics, checker_, sampler);
  //  solver->config(nh_);

  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRTStar>(configuration_replan_, current_path_, time_for_repl, solver_);
}

bool ReplannerManagerDRRTStar::replan()
{
  return replanner_->replan();
}
}
