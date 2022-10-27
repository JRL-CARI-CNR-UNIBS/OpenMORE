#include "replanners_lib/replanner_managers/replanner_manager_DRRT.h"

namespace pathplan
{

ReplannerManagerDRRT::ReplannerManagerDRRT(const PathPtr &current_path,
                                           const TreeSolverPtr &solver,
                                           const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  RRTPtr tmp_solver = std::make_shared<pathplan::RRT>(solver_->getMetrics(), checker_replanning_, solver_->getSampler());
  tmp_solver->importFromSolver(solver);

  solver_  = tmp_solver;
}

void ReplannerManagerDRRT::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  paths_mtx_.lock();
  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_replanning_);
  paths_mtx_.unlock();

  std::vector<ConnectionPtr> path_connections;

  PathPtr replanned_path = replanner_->getReplannedPath();
  NodePtr replanned_path_start = replanned_path->getStartNode();
  NodePtr goal = replanned_path->getGoalNode();
  TreePtr tree = replanned_path->getTree();

  assert(goal == replanner_->getGoal());
  assert(tree->isInTree(replanned_path_start));

  //If the configuration matches to a node of the replanned path
  for(const NodePtr& node:replanned_path->getNodes())
  {
    if((node->getConfiguration()-configuration).norm()<TOLERANCE)
    {
      assert(node->getConfiguration() != replanned_path->getWaypoints().back());
      assert(tree->isInTree(node));

      tree->changeRoot(node);
      path_connections = tree->getConnectionToNode(goal);
      replanned_path->setConnections(path_connections);

      return;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  NodePtr current_node;
  PathPtr new_tree_branch;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path_copy->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path_copy->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  assert(abscissa_current_conf != abscissa_replanned_path_start);

  if(abscissa_current_conf < abscissa_replanned_path_start)
  {
    try
    {
      new_tree_branch = current_path_copy->getSubpathToConf(replanned_path_start->getConfiguration(),false);
    }
    catch(...)
    {
      ROS_INFO_STREAM("replanned_path_start conf:  "<<replanned_path_start->getConfiguration().transpose());
      for(const Eigen::VectorXd& wp:current_path_copy->getWaypoints())
        ROS_INFO_STREAM("CURRENT PATH COPY WP: "<<wp.transpose());
    }

    new_tree_branch = new_tree_branch->getSubpathFromConf(configuration,false);
    new_tree_branch = new_tree_branch->clone(); //if you not clone, you flip only a part of current_path_copy and this crates nodes with more than one parent and nodes with zero parents
    new_tree_branch->flip();

    std::vector<ConnectionPtr> new_tree_branch_connections = new_tree_branch->getConnections();
    current_node = new_tree_branch_connections.back()->getChild();
    assert(current_node->getConfiguration() == configuration);

    ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
    NodePtr child = conn2delete->getChild();

    ConnectionPtr new_conn = std::make_shared<Connection>(replanned_path_start,child);
    new_conn->setCost(conn2delete->getCost());
    new_conn->add();

    conn2delete->remove();

    new_tree_branch_connections.at(0) = new_conn;

    if(not tree->addBranch(new_tree_branch_connections))
      ROS_ERROR("Branch from current node not added to the replanned tree");

    assert(tree->isInTree(current_node));

    if(not tree->changeRoot(current_node))
      ROS_ERROR("Root can't be changed to current node");

    path_connections = tree->getConnectionToNode(goal);
    replanned_path->setConnections(path_connections);

    return;
  }
  else
  {
    double cost;
    ConnectionPtr conn;
    int idx_current_conf_on_replanned;

    ConnectionPtr conn_on_replannned_path = replanned_path->findConnection(configuration,idx_current_conf_on_replanned);
    if(conn_on_replannned_path)
    {
      current_node = std::make_shared<Node>(configuration);

      NodePtr child = replanned_path->getConnections().at(idx_current_conf_on_replanned)->getChild();
      conn = std::make_shared<Connection>(child,current_node);

      MetricsPtr metrics = solver_->getMetrics();
      if(replanned_path->getConnections().at(idx_current_conf_on_replanned)->getCost() == std::numeric_limits<double>::infinity())
      {
        checker_replanning_->checkConnection(conn)?
              (cost = metrics->cost(child->getConfiguration(),configuration)):
              (cost = std::numeric_limits<double>::infinity());
      }
      else
        cost = metrics->cost(child->getConfiguration(),configuration);

      conn->setCost(cost);
      conn->add();

      tree->addNode(current_node);
      assert(tree->isInTree(current_node));

      if(not tree->changeRoot(current_node))
        ROS_ERROR("Root can't be changed to current node");

      path_connections = tree->getConnectionToNode(goal);
      replanned_path->setConnections(path_connections);

      return;
    }
    else
    {
      ROS_INFO("no conn");
      try
      {
        new_tree_branch = current_path_copy->getSubpathToConf(configuration,false);
      }
      catch(...)
      {
        ROS_INFO_STREAM("rconfiguration:  "<<configuration.transpose());
        for(const Eigen::VectorXd& wp:current_path_copy->getWaypoints())
          ROS_INFO_STREAM("CURRENT PATH COPY WP: "<<wp.transpose());
      }

      new_tree_branch = new_tree_branch->getSubpathFromConf(replanned_path_start->getConfiguration(),false);

      std::vector<ConnectionPtr> new_tree_branch_connections = new_tree_branch->getConnections();

      //Delete redundant connections
      bool delete_conn = false;
      int idx = 0;

      std::vector<ConnectionPtr> replanned_path_conns = replanned_path->getConnections();
      for(unsigned int i=0;i<new_tree_branch_connections.size();i++)
      {
        bool match = (new_tree_branch_connections.at(i)->getChild()->getConfiguration()
                      - replanned_path_conns.at(i)->getChild()->getConfiguration()).norm()<1e-06;

        if(match)
        {
          idx = i;
          delete_conn = true;
        }
        else
          break;
      }

      ConnectionPtr new_conn;
      if(delete_conn)
      {
        int size_branch = new_tree_branch_connections.size()-(idx+1);
        new_tree_branch_connections.insert(new_tree_branch_connections.begin(),new_tree_branch_connections.begin()+(idx+1),new_tree_branch_connections.end());
        new_tree_branch_connections.resize(size_branch);

        ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
        NodePtr parent = replanned_path->getConnections().at(idx)->getChild();
        NodePtr child = conn2delete->getChild();

        new_conn = std::make_shared<Connection>(parent,child);
        new_conn->setCost(conn2delete->getCost());
        new_conn->add();

        conn2delete->remove();
      }
      else
      {
        ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
        NodePtr child = conn2delete->getChild();

        new_conn = std::make_shared<Connection>(replanned_path_start,child);
        new_conn->setCost(conn2delete->getCost());
        new_conn->add();

        conn2delete->remove();
      }

      new_tree_branch_connections.at(0) = new_conn;

      current_node = new_tree_branch_connections.back()->getChild();

      if(not tree->addBranch(new_tree_branch_connections))
        ROS_ERROR("Branch from current node not added to the replanned tree");

      assert(tree->isInTree(current_node));
      if(not tree->changeRoot(current_node))
        ROS_ERROR("Root can't be changed to current node");

      path_connections = tree->getConnectionToNode(goal);
      replanned_path->setConnections(path_connections);

      return;
    }
  }
}

bool ReplannerManagerDRRT::haveToReplan(const bool path_obstructed)
{
  return replanIfObstructed(path_obstructed);
}

void ReplannerManagerDRRT::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::DynamicRRT>(configuration_replan_, current_path_, time_for_repl, solver_);
}

}
