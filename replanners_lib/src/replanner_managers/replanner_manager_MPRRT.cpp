#include "replanners_lib/replanner_managers/replanner_manager_MPRRT.h"

namespace pathplan
{

ReplannerManagerMPRRT::ReplannerManagerMPRRT(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTPtr tmp_solver = std::make_shared<pathplan::RRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_ = tmp_solver;

  additionalParams();
}

void ReplannerManagerMPRRT::additionalParams()
{
  if(!nh_.getParam("MPRRT/n_threads_replan",n_threads_replan_))
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

void ReplannerManagerMPRRT::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  std::vector<pathplan::ConnectionPtr> path_connections;
  PathPtr replanned_path = replanner_->getReplannedPath();
  Eigen::VectorXd replanned_path_start_conf = replanned_path->getStartNode()->getConfiguration();
  std::vector<ConnectionPtr> conn_replanned = replanned_path->getConnections();

  //If the configuration matches to a node of the replanned path
  for(const Eigen::VectorXd& wp:replanned_path->getWaypoints())
  {
    if((wp-configuration).norm()<TOLERANCE)
    {
      assert(wp != replanned_path->getWaypoints().back());
      replanned_path = replanned_path->getSubpathFromNode(configuration);

      return;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  PathPtr current_path = replanner_->getCurrentPath();

  PathPtr path_conf2replanned;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path->curvilinearAbscissaOfPoint(replanned_path_start_conf,idx_replanned_path_start);

  assert(abscissa_current_conf != abscissa_replanned_path_start);

  if(abscissa_current_conf < abscissa_replanned_path_start)  //the replanned path starts from a position after the current one
  {
    path_conf2replanned = current_path->clone();
    NodePtr n1 = path_conf2replanned->addNodeAtCurrentConfig(configuration,true);
    NodePtr n2 = path_conf2replanned->addNodeAtCurrentConfig(replanned_path_start_conf,true);

    path_conf2replanned = path_conf2replanned->getSubpathFromNode(n1);
    path_conf2replanned = path_conf2replanned->getSubpathToNode  (n2);

    path_connections = path_conf2replanned->getConnections();

    assert((path_connections.back()->getChild()->getConfiguration()-conn_replanned.front()->getParent()->getConfiguration()).norm()<TOLERANCE);

    ConnectionPtr conn = std::make_shared<Connection>(path_connections.back()->getParent(),conn_replanned.front()->getParent(),false);
    conn->setCost(path_connections.back()->getCost());
    conn->add();

    path_connections.back()->remove();
    path_connections.pop_back();
    path_connections.push_back(conn);

    path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
  }
  else
  {
    path_conf2replanned = current_path->clone();
    NodePtr n1 = path_conf2replanned->addNodeAtCurrentConfig(replanned_path_start_conf,true);
    NodePtr n2 = path_conf2replanned->addNodeAtCurrentConfig(configuration,true);

    path_conf2replanned = path_conf2replanned->getSubpathFromNode(n1);
    path_conf2replanned = current_path->getSubpathToNode(n2);

    path_conf2replanned->flip();
    path_connections = path_conf2replanned->getConnections();

    ConnectionPtr conn = std::make_shared<Connection>(path_connections.back()->getParent(),conn_replanned.front()->getParent(),false);
    conn->setCost(path_connections.back()->getCost());
    conn->add();

    path_connections.back()->remove();
    path_connections.pop_back();
    path_connections.push_back(conn);

    path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
  }

  replanned_path->setConnections(path_connections);
  replanned_path->simplify(0.01);

}

bool ReplannerManagerMPRRT::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerMPRRT::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::MPRRT>(configuration_replan_, current_path_, time_for_repl, solver_,n_threads_replan_);
}

}
