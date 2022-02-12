#include "replanners_lib/replanner_managers/replanner_manager_DRRT.h"

namespace pathplan
{

void plotBranch(const std::vector<ConnectionPtr> &branch) //ELIMINA
{
  for(const ConnectionPtr& conn:branch) //ELIMINA
  {
    ROS_INFO_STREAM("bn p: "<<conn->getParent()->getConfiguration().transpose()<<" "<<conn->getParent());
    ROS_INFO_STREAM("bn c: "<<conn->getChild()->getConfiguration().transpose()<<" "<<conn->getChild());
  }

  ROS_INFO_STREAM("bn: "<<(branch.front()->getParent())<<"\n"<<*(branch.front()->getParent()));
  for(const ConnectionPtr& conn:branch)
    ROS_INFO_STREAM("bn: "<<conn->getChild()<<"\n"<<*conn->getChild()<<" its_parent: "<<conn->getChild()->getParents().front()->getConfiguration().transpose());
}

void checkNewBranch(const std::vector<ConnectionPtr> &branch) //ELIMINA
{
  for(const ConnectionPtr& conn:branch)
  {
    if(conn->getChild()->parent_connections_.size() != 1)
    {
      ROS_INFO_STREAM("Node is unconnected: "<<conn->getChild()->getConfiguration().transpose());
      plotBranch(branch);
      assert(0);
    }
    if(conn->norm() == 0)
    {
      for(const ConnectionPtr& conn:branch)
      {
        ROS_WARN_STREAM("branch p"<<conn->getParent()->getConfiguration().transpose());
        ROS_WARN_STREAM("branch c"<<conn->getChild()->getConfiguration().transpose());
      }
      assert(0);
    }
  }
}

bool checkTree(const TreePtr& tree) //ELIMINA
{
  std::vector<NodePtr> nodes = tree->getNodes();
  NodePtr root = tree->getRoot();

  for(const NodePtr& n:nodes)
  {
    if(n == root)
      continue;

    if(n->parent_connections_.size() != 1)
      return false;
  }
  return true;
}

void plotTree(const TreePtr& tree) //ELIMINA
{
  std::vector<NodePtr> nodes = tree->getNodes();
  NodePtr root = tree->getRoot();

  ROS_INFO("Tree nodes");

  for(const NodePtr& n:nodes)
  {
    if(n == root)
      continue;

    ROS_INFO_STREAM(*n);
  }

  ROS_INFO_STREAM("Root:"<<root<<"\n"<<*root);
}

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
  ROS_INFO("------");
  paths_mtx_.lock();
  PathPtr current_path_copy = current_path_shared_->clone();
  paths_mtx_.unlock();

  std::vector<ConnectionPtr> path_connections;

  PathPtr replanned_path = replanner_->getReplannedPath();
  TreePtr tree = replanned_path->getTree();
  NodePtr goal = replanned_path->getConnections().back()->getChild();
  NodePtr replanned_path_start = replanned_path->getConnections().front()->getParent();

  ROS_INFO_STREAM("ddrt goal ptr "<<goal);

  if(not checkTree(tree)) //ELIMINA
  {
    plotTree(tree);
    ROS_INFO_STREAM("Current conf"<<configuration.transpose());
    ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

    assert(0);
  }

  assert(goal == replanner_->getGoal());
  assert(tree->isInTree(replanned_path_start));

  //If the configuration matches to a node of the replanned path
  for(const NodePtr& node:replanned_path->getNodes())
  {
    if(node->getConfiguration() == configuration)
    {
      assert(node->getConfiguration() != replanned_path->getWaypoints().back());
      assert(tree->isInTree(node));

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

      ROS_INFO("QUA0");
      tree->changeRoot(node);
      path_connections = tree->getConnectionToNode(goal);
      replanned_path->setConnections(path_connections);

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

      return;
    }
  }

  //Otherwise, if the configuration does not match to any path node..
  NodePtr current_node;
  PathPtr new_tree_branch;
  int idx_current_conf, idx_replanned_path_start;

  double abscissa_current_conf = current_path_copy->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_replanned_path_start = current_path_copy->curvilinearAbscissaOfPoint(replanned_path_start->getConfiguration(),idx_replanned_path_start);

  if(abscissa_current_conf == abscissa_replanned_path_start)     //Double check..
  {

    if(not checkTree(tree)) //ELIMINA
    {
      plotTree(tree);
      ROS_INFO_STREAM("Current conf"<<configuration.transpose());
      ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

      assert(0);
    }

    return;
  }
  else if(abscissa_current_conf < abscissa_replanned_path_start)
  {
    new_tree_branch = current_path_copy->getSubpathToConf(replanned_path_start->getConfiguration(),true);
    ROS_INFO("TO REPLANNED");
    plotBranch(new_tree_branch->getConnectionsConst()); //ELIMINA

    new_tree_branch = new_tree_branch->getSubpathFromConf(configuration,true);
    ROS_INFO("FROM CONF");
    plotBranch(new_tree_branch->getConnectionsConst()); //ELIMINA

    new_tree_branch->flip();
    ROS_INFO("FLIPPED");
    plotBranch(new_tree_branch->getConnectionsConst()); //ELIMINA

    std::vector<ConnectionPtr> new_tree_branch_connections = new_tree_branch->getConnections();
    ROS_INFO("BEFORE CHANGING FIRST CONNECTION");
    plotBranch(new_tree_branch_connections); //ELIMINA

    ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
    NodePtr child = conn2delete->getChild();

    ConnectionPtr new_conn = std::make_shared<Connection>(replanned_path_start,child);
    new_conn->setCost(conn2delete->getCost());
    new_conn->add();

    conn2delete->remove();

    if(not checkTree(tree)) //ELIMINA
    {
      plotTree(tree);
      ROS_INFO_STREAM("Current conf"<<configuration.transpose());
      ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

      assert(0);
    }

    new_tree_branch_connections.at(0) = new_conn;

    current_node = new_tree_branch_connections.back()->getChild();
    assert(current_node->getConfiguration() == configuration);

    checkNewBranch(new_tree_branch_connections); //ELIMINA
    ROS_INFO("AFTER CHANGING FIRST CONNECTION");
    plotBranch(new_tree_branch_connections); //ELIMINA

    if(not tree->addBranch(new_tree_branch_connections))
      ROS_ERROR("Branch from current node not added to the replanned tree");

    assert(tree->isInTree(current_node));

    if(not checkTree(tree)) //ELIMINA
    {
      plotTree(tree);
      ROS_INFO_STREAM("Current conf"<<configuration.transpose());
      ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

      assert(0);
    }

    ROS_INFO("QUA1");
    ROS_INFO_STREAM("curr node \n"<<*current_node);
    ROS_INFO_STREAM("repl node \n"<<*replanned_path_start);
    if(not tree->changeRoot(current_node))
      ROS_ERROR("Root can't be changed to current node");

    if(not checkTree(tree)) //ELIMINA
    {
      plotTree(tree);
      ROS_INFO_STREAM("Current conf"<<configuration.transpose());
      ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

      assert(0);
    }

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

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

      assert(tree->isInTree(current_node));

      ROS_INFO("QUA2");
      if(not tree->changeRoot(current_node))
        ROS_ERROR("Root can't be changed to current node");

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

      path_connections = tree->getConnectionToNode(goal);
      replanned_path->setConnections(path_connections);

      return;
    }
    else
    {
      new_tree_branch = current_path_copy->getSubpathToConf(configuration,true);
      new_tree_branch = new_tree_branch->getSubpathFromConf(replanned_path_start->getConfiguration(),true);

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
        new_tree_branch_connections.clear();
        new_tree_branch_connections.insert(new_tree_branch_connections.begin(),new_tree_branch_connections.begin()+(idx+1),new_tree_branch_connections.end());

        ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
        NodePtr parent = replanned_path->getConnections().at(idx)->getChild();
        NodePtr child = conn2delete->getChild();

        if(not checkTree(tree)) //ELIMINA
        {
          plotTree(tree);
          ROS_INFO_STREAM("Current conf"<<configuration.transpose());
          ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

          assert(0);
        }

        new_conn = std::make_shared<Connection>(parent,child);
        new_conn->setCost(conn2delete->getCost());
        new_conn->add();

        conn2delete->remove();

        if(not checkTree(tree)) //ELIMINA
        {
          plotTree(tree);
          ROS_INFO_STREAM("Current conf"<<configuration.transpose());
          ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

          assert(0);
        }
      }
      else
      {
        ConnectionPtr conn2delete = new_tree_branch_connections.at(0);
        NodePtr child = conn2delete->getChild();

        if(not checkTree(tree)) //ELIMINA
        {
          plotTree(tree);
          ROS_INFO_STREAM("Current conf"<<configuration.transpose());
          ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

          assert(0);
        }

        new_conn = std::make_shared<Connection>(replanned_path_start,child);
        new_conn->setCost(conn2delete->getCost());
        new_conn->add();

        conn2delete->remove();

        if(not checkTree(tree)) //ELIMINA
        {
          plotTree(tree);
          ROS_INFO_STREAM("Current conf"<<configuration.transpose());
          ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

          assert(0);
        }
      }

      new_tree_branch_connections.at(0) = new_conn;

      current_node = new_tree_branch_connections.back()->getChild();

      checkNewBranch(new_tree_branch_connections); //ELIMINA

      plotBranch(new_tree_branch_connections); //ELIMINA

      if(not tree->addBranch(new_tree_branch_connections))
        ROS_ERROR("Branch from current node not added to the replanned tree");

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

      assert(tree->isInTree(current_node));

      ROS_INFO("QUA3");
      ROS_INFO_STREAM("Current conf"<<configuration.transpose());
      ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

      if(not tree->changeRoot(current_node))
        ROS_ERROR("Root can't be changed to current node");

      if(not checkTree(tree)) //ELIMINA
      {
        plotTree(tree);
        ROS_INFO_STREAM("Current conf"<<configuration.transpose());
        ROS_INFO_STREAM("Replan start:"<<replanned_path_start<<"\n"<<*replanned_path_start);

        assert(0);
      }

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
  replanner_ = std::make_shared<pathplan::DynamicRRT>(configuration_replan_, current_path_replanning_, time_for_repl, solver_);
}

}
