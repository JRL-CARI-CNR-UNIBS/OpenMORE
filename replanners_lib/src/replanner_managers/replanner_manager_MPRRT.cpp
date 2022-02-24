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

void ReplannerManagerMPRRT::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  ROS_WARN("START FROM NEW CONF"); //ELIMINA

  std::vector<pathplan::ConnectionPtr> path_connections;
  PathPtr replanned_path = replanner_->getReplannedPath();
  pathplan::NodePtr replanned_path_start = replanned_path->getConnections().front()->getParent();
  std::vector<ConnectionPtr> conn_replanned = replanned_path->getConnections();

  //If the configuration matches to a node of the replanned path
  for(const Eigen::VectorXd& wp:replanned_path->getWaypoints())
  {
    if(wp == configuration)
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
  double abscissa_replanned_path_start = current_path->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  assert(abscissa_current_conf != abscissa_replanned_path_start);

  if(abscissa_current_conf < abscissa_replanned_path_start)  //the replanned path starts from a position after the current one
  {
    path_conf2replanned = current_path->getSubpathToConf(replanned_path_start->getConfiguration(),true);
    path_conf2replanned = path_conf2replanned->getSubpathFromConf(configuration,true);

    path_connections = path_conf2replanned->getConnections();
    path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
  }
  else
  {
    int idx_current_conf_on_replanned;
    ConnectionPtr conn = replanned_path->findConnection(configuration,idx_current_conf_on_replanned);

    if(conn)
      path_connections = replanned_path->getSubpathFromConf(configuration,true)->getConnections();
    else
    {
      path_conf2replanned = current_path->getSubpathToConf(configuration,true);
      path_conf2replanned = path_conf2replanned->getSubpathFromConf(replanned_path_start->getConfiguration(),true);

      std::vector<ConnectionPtr> conn_conf2replanned = path_conf2replanned->getConnections();

      //Delete redundant connections
      bool delete_conn = false;
      int idx = 0;

      for(unsigned int i=0;i<conn_conf2replanned.size();i++)
      {
        bool match = (conn_conf2replanned.at(i)->getChild()->getConfiguration()
                      - conn_replanned.at(i)->getChild()->getConfiguration()).norm()<1e-06;

        if(match)
        {
          idx = i;
          delete_conn = true;
        }
        else
          break;
      }

      path_conf2replanned->flip();
      conn_conf2replanned = path_conf2replanned->getConnections();

      if(delete_conn)
      {
        path_connections.insert(path_connections.end(),conn_conf2replanned.begin(),conn_conf2replanned.begin()+(conn_conf2replanned.size()-(1+idx)));
        path_connections.insert(path_connections.end(),conn_replanned.begin()+(idx+1),conn_replanned.end());
      }
      else
      {
        path_connections = conn_conf2replanned;
        path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
      }
    }
  }

  replanned_path->setConnections(path_connections);
}

bool ReplannerManagerMPRRT::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerMPRRT::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::MPRRT>(configuration_replan_, current_path_replanning_, time_for_repl, solver_,n_threads_replan_);
}

}
