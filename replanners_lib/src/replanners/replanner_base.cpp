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

  solver_  = solver;
  metrics_ = solver->getMetrics();
  checker_ = solver->getChecker();

  lb_ = solver->getSampler()->getLB();
  ub_ = solver->getSampler()->getUB();

  max_time_ = max_time;
  success_ = false;

  disp_ = NULL;
  verbose_ = false;
}

std::vector<ConnectionPtr> ReplannerBase::startReplannedTreeFromNewCurrentConf(const Eigen::VectorXd &configuration,
                                                                               const PathPtr &current_path_copy)
{
  //NB: replanned_path_start must be a node of the tree; current_path_copy because the real current path can be modified after tree rewiring

  std::vector<ConnectionPtr> path_connections, new_tree_branch_connections;

  TreePtr tree = replanned_path_->getTree();
  NodePtr goal = replanned_path_->getConnections().back()->getChild();
  NodePtr replanned_path_start = replanned_path_->getConnections().front()->getParent();

  assert(tree->isInTree(replanned_path_start));

  //If the configuration matches to a node of the replanned path
  for(const NodePtr& node:replanned_path_->getNodes())
  {
    if(node->getConfiguration() == configuration)
    {
      assert(node->getConfiguration() != replanned_path_->getWaypoints().back());
      assert(tree->isInTree(node));

      tree->changeRoot(node);
      path_connections = tree->getConnectionToNode(goal);
      replanned_path_->setConnections(path_connections);

      new_tree_branch_connections.clear();
      return new_tree_branch_connections;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  NodePtr current_node;
  PathPtr new_tree_branch;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path_copy->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path_copy->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  if(abscissa_current_conf == abscissa_replanned_path_start)
  {
    new_tree_branch_connections.clear();
    return new_tree_branch_connections;
  }
  else if(abscissa_current_conf < abscissa_replanned_path_start)
  {
    new_tree_branch = current_path_copy->getSubpathToConf(replanned_path_start->getConfiguration(),true);
    new_tree_branch = new_tree_branch->getSubpathFromConf(configuration,true);
    new_tree_branch->flip();

    new_tree_branch_connections = new_tree_branch->getConnections();

    ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
    NodePtr child = conn2delete->getChild();

    ConnectionPtr new_conn = std::make_shared<Connection>(replanned_path_start,child);
    new_conn->setCost(conn2delete->getCost());
    new_conn->add();

    conn2delete->remove();

    new_tree_branch_connections.at(0) = new_conn;

    current_node = new_tree_branch_connections.back()->getChild();
    tree->addBranch(new_tree_branch_connections);
    tree->changeRoot(current_node);

    new_tree_branch_connections = tree->getConnectionToNode(replanned_path_start);

    path_connections = tree->getConnectionToNode(goal);
    replanned_path_->setConnections(path_connections);

    return new_tree_branch_connections;
  }
  else
  {
    double cost;
    ConnectionPtr conn;
    int idx_current_conf_on_replanned;

    ConnectionPtr conn_on_replannned_path = replanned_path_->findConnection(configuration,idx_current_conf_on_replanned);
    if(conn_on_replannned_path)
    {
      current_node = std::make_shared<Node>(configuration);

      NodePtr child = replanned_path_->getConnections().at(idx_current_conf_on_replanned)->getChild();
      conn = std::make_shared<Connection>(child,current_node);

      if(replanned_path_->getConnections().at(idx_current_conf_on_replanned)->getCost() == std::numeric_limits<double>::infinity())
      {
        checker_->checkConnection(conn)?
              (cost = metrics_->cost(child->getConfiguration(),configuration)):
              (cost = std::numeric_limits<double>::infinity());
      }
      else
        cost = metrics_->cost(child->getConfiguration(),configuration);

      conn->setCost(cost);
      conn->add();

      tree->addNode(current_node);
      tree->changeRoot(current_node);

      new_tree_branch_connections = tree->getConnectionToNode(child);

      path_connections = tree->getConnectionToNode(goal);
      replanned_path_->setConnections(path_connections);

      //      new_tree_branch_connections.push_back(conn);
      return new_tree_branch_connections;
    }
    else
    {
      new_tree_branch = current_path_copy->getSubpathToConf(configuration,true);
      new_tree_branch = new_tree_branch->getSubpathFromConf(replanned_path_start->getConfiguration(),true);

      new_tree_branch_connections = new_tree_branch->getConnections();

      //Delete redundant connections
      bool delete_conn = false;
      int idx = 0;

      std::vector<ConnectionPtr> replanned_path_conns = replanned_path_->getConnections();
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

        //        if(delete_conn)
        //          break;
      }

      ConnectionPtr new_conn;
      if(delete_conn)
      {
        int new_size = new_tree_branch_connections.size()-(idx+1);
        new_tree_branch_connections.insert(new_tree_branch_connections.begin(),new_tree_branch_connections.begin()+(idx+1),new_tree_branch_connections.end());
        new_tree_branch_connections.resize(new_size);

        ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
        NodePtr parent = replanned_path_->getConnections().at(idx)->getChild();
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
      tree->addBranch(new_tree_branch_connections);
      tree->changeRoot(current_node);

      new_tree_branch_connections = tree->getConnectionToNode(new_tree_branch_connections.front()->getParent());

      path_connections = tree->getConnectionToNode(goal);
      replanned_path_->setConnections(path_connections);

      return new_tree_branch_connections;
    }
  }
}

void ReplannerBase::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd &configuration)
{
  std::vector<pathplan::ConnectionPtr> path_connections;
  pathplan::NodePtr replanned_path_start = replanned_path_->getConnections().front()->getParent();
  std::vector<ConnectionPtr> conn_replanned = replanned_path_->getConnections();

  //If the configuration matches to a node of the replanned path
  for(const Eigen::VectorXd& wp:replanned_path_->getWaypoints())
  {
    if(wp == configuration)
    {
      assert(wp != replanned_path_->getWaypoints().back());
      replanned_path_ = replanned_path_->getSubpathFromNode(configuration);
      return;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  PathPtr path_conf2replanned;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path_->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  assert(abscissa_current_conf != abscissa_replanned_path_start);

  if(abscissa_current_conf < abscissa_replanned_path_start)  //the replanned path starts from a position after the current one
  {
    path_conf2replanned = current_path_->getSubpathToConf(replanned_path_start->getConfiguration(),true);
    path_conf2replanned = path_conf2replanned->getSubpathFromConf(configuration,true);

    path_connections = path_conf2replanned->getConnections();
    path_connections.insert(path_connections.end(),conn_replanned.begin(),conn_replanned.end());
  }
  else
  {
    int idx_current_conf_on_replanned;
    ConnectionPtr conn = replanned_path_->findConnection(configuration,idx_current_conf_on_replanned);

    if(conn)
    {
      path_connections = replanned_path_->getSubpathFromConf(configuration,true)->getConnections();
    }
    else
    {
      path_conf2replanned = current_path_->getSubpathToConf(configuration,true);
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

        //        if(delete_conn)
        //          break;
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

  replanned_path_->setConnections(path_connections);
}
}
