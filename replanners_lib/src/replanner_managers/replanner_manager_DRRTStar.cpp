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

  for(const ConnectionPtr& conn:current_path->getConnections()) //ELIMINA
  {
    ROS_INFO_STREAM("pc: "<<conn->getParent()->getConfiguration().transpose());
    ROS_INFO_STREAM("cc: "<<conn->getChild() ->getConfiguration().transpose());
  }

  for(const ConnectionPtr& conn:replanned_path->getConnections()) //ELIMINA
  {
    ROS_INFO_STREAM("pr: "<<conn->getParent()->getConfiguration().transpose());
    ROS_INFO_STREAM("cr: "<<conn->getChild() ->getConfiguration().transpose());
  }

  //  if(old_current_node_)
  //    current_path->removeNode(old_current_node_,{});

  for(const NodePtr& n:replanned_path->getNodes()) //elimina
  {
    if(n->parent_connections_.size() != 1 && n!=tree->getRoot())
    {
      for(const NodePtr& nn:replanned_path->getNodes())
        ROS_INFO_STREAM("rp node:\n"<<*nn);

      assert(0);
    }
  }

  NodePtr current_node;
  ConnectionPtr conn = current_path->findConnection(configuration);

  if(conn->isValid())
    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true);
  else  //if the conn of current conf is the conn before the replan goal, it is not valid
  {
    ROS_WARN("NOT VALID"); //elimina
    ROS_INFO_STREAM("parent nv: "<<conn->getParent()->getConfiguration().transpose());
    ROS_INFO_STREAM("child nv: " <<conn->getChild() ->getConfiguration().transpose());

    for(const ConnectionPtr& cc: current_path->getConnections()) //elimina
      ROS_INFO_STREAM("curr conn: "<<cc<<"\n"<<*cc);

    for(const ConnectionPtr& cc: replanned_path->getConnections()) //elimina
      ROS_INFO_STREAM("rep conn: "<<cc<<"\n"<<*cc);

    assert(conn->getParent() != nullptr && conn->getParent() != NULL);

    current_node = current_path->addNodeAtCurrentConfig(configuration,conn,false);
    conn = std::make_shared<Connection>(conn->getParent(),current_node);
    conn->setCost(tree->getMetrics()->cost(conn->getParent(),current_node));
    conn->add();

    tree->addNode(current_node);
  }

  for(const NodePtr& n:replanned_path->getNodes()) //elimina
  {
    if(n->parent_connections_.size() != 1 && n!=tree->getRoot())
    {
      for(const NodePtr& nn:replanned_path->getNodes())
        ROS_INFO_STREAM("rp node:\n"<<*nn);

      assert(0);
    }
  }

  tree->changeRoot(current_node);

  for(const NodePtr& n:replanned_path->getNodes()) //elimina
  {
    if(n->parent_connections_.size() != 1 && n!=tree->getRoot())
    {
      for(const NodePtr& nn:replanned_path->getNodes())
        ROS_INFO_STREAM("rp node:\n"<<*nn);

      assert(0);
    }
  }

  std::vector<ConnectionPtr> new_conns = tree->getConnectionToNode(replanner_->getGoal());
  replanned_path->setConnections(new_conns);

  //  old_current_node_ = current_node;
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
