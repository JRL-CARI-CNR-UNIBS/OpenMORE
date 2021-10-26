#include "replanners_lib/replanner_managers/replanner_manager_DRRT.h"

namespace pathplan
{
ReplannerManagerDRRT::ReplannerManagerDRRT(PathPtr &current_path,
                                           TreeSolverPtr solver,
                                           ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTPtr tmp_solver = std::make_shared<pathplan::RRT>(solver_->getMetrics(), checker_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

bool ReplannerManagerDRRT::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRT::initReplanner()
{
  //  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  //  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb_, ub_);
  //  pathplan::RRTPtr     solver  = std::make_shared<pathplan::RRT>(metrics, checker_, sampler);
  //  solver->config(nh_);

  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRT>(configuration_replan_, current_path_, time_for_repl, solver_);
}

bool ReplannerManagerDRRT::replan()
{
  return replanner_->replan();
}
}
