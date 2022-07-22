#include "replanners_lib/replanners/replanner_base.h"

namespace pathplan
{
ReplannerBase::ReplannerBase(const Eigen::VectorXd& current_configuration,
                             const PathPtr& current_path,
                             const double& max_time,
                             const TreeSolverPtr &solver)
{
  current_configuration_ = current_configuration;
  current_path_ = current_path;
  replanned_path_ = current_path;

  goal_node_ = current_path_->getGoalNode();

  solver_  = solver;
  metrics_ = solver->getMetrics();
  checker_ = solver->getChecker();

  lb_ = solver->getSampler()->getLB();
  ub_ = solver->getSampler()->getUB();

  max_time_ = max_time;
  success_ = false;

  disp_ = nullptr;
  verbose_ = false;

  assert(TOLERANCE>0);
}

ReplannerBase::~ReplannerBase()
{
}

}
