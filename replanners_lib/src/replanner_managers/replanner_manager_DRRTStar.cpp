#include "replanners_lib/replanner_managers/replanner_manager_DRRTStar.h"

namespace pathplan
{
ReplannerManagerDRRTStar::ReplannerManagerDRRTStar(const PathPtr &current_path,
                                                   const TreeSolverPtr &solver,
                                                   const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTStarPtr tmp_solver = std::make_shared<pathplan::RRTStar>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerDRRTStar::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  PathPtr current_path = replanner_->getCurrentPath();
  PathPtr replanned_path = replanner_->getReplannedPath();
  TreePtr tree = current_path->getTree();

  if(not tree->changeRoot(current_path->getStartNode()))
    throw std::runtime_error("root can not be changed");

  if(old_current_node_ && old_current_node_ != tree->getRoot())
    current_path->removeNode(old_current_node_,{});

  NodePtr current_node;
  ConnectionPtr conn = current_path->findConnection(configuration);

  if(conn->isValid())
    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true);
  else  //if the conn of current conf is the conn before the replan goal, it is not valid
  {
    assert(conn->getParent() != nullptr && conn->getParent() != nullptr);

    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,false);
    conn = std::make_shared<Connection>(conn->getParent(),current_node);
    conn->setCost(tree->getMetrics()->cost(conn->getParent(),current_node));
    conn->add();

    tree->addNode(current_node);
  }

  if(not tree->changeRoot(current_node))
    throw std::runtime_error("root can not be changed");

  std::vector<ConnectionPtr> new_conns = tree->getConnectionToNode(replanned_path->getGoalNode());
  replanned_path->setConnections(new_conns);

  old_current_node_ = current_node;
}

bool ReplannerManagerDRRTStar::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRTStar::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRTStar>(configuration_replan_, current_path_replanning_, time_for_repl, solver_);
}

}
