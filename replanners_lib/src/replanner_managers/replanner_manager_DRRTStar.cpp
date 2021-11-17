#include "replanners_lib/replanner_managers/replanner_manager_DRRTStar.h"

namespace pathplan
{
ReplannerManagerDRRTStar::ReplannerManagerDRRTStar(PathPtr &current_path,
                                                   TreeSolverPtr solver,
                                                   ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTStarPtr tmp_solver = std::make_shared<pathplan::RRTStar>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerDRRTStar::connectToReplannedPath()
{
  std::vector<NodePtr> nodes;
  std::vector<double> costs;
  detachAddedBranch(nodes,costs);

  connectCurrentConfToTree();

  if(replanner_->getReplannedPath()->removeNodes())
    ROS_INFO_STREAM("removed node");
  else
    ROS_INFO_STREAM("node can not be removed");

  //NON POSSO SEMPLICEMENTE AGGIUNGERE UNA CONFIGURAZIONE NEL TREE E FARMI DARE IL PATH PERCHE IL REWIRE POTREBBE SPOSTARE
  //LA CONNESSIONE A CUI LA CONFIGURAZIONE APPARTENEVA, DUNQUE LA CONFIGURAZIONE NON POTREBBE VENIRE INSERITA NEL TREE
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

bool ReplannerManagerDRRTStar::replan()
{
  bool replanned = replanner_->replan();

  return replanned;
}
}
