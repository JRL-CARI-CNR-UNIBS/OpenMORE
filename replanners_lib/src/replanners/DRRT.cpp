#include "replanners_lib/replanners/DRRT.h"

namespace pathplan
{

void checkPathConn(const PathPtr& path) //ELIMINA
{
  for(const ConnectionPtr& conn:path->getConnections())
  {
    ROS_INFO_STREAM("pp "<<conn->getParent()->getConfiguration().transpose());
    ROS_INFO_STREAM("pc "<<conn->getChild()->getConfiguration().transpose());
  }

  for(const ConnectionPtr& conn:path->getConnections())
  {
    if(conn->norm() == 0.0)
    {
      ROS_WARN_STREAM("Parent\n "<<*conn->getParent());
      ROS_WARN_STREAM("Child\n "<<*conn->getChild());

      assert(0);
    }
  }
}

DynamicRRT::DynamicRRT(Eigen::VectorXd& current_configuration,
                       PathPtr& current_path,
                       const double& max_time,
                       const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(RRT);
  const std::type_info& ti2 = typeid(*solver);

  RRTPtr tmp_solver;

  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<pathplan::RRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
  {
    tmp_solver = std::static_pointer_cast<RRT>(solver);
  }

  solver_ = tmp_solver;
  sampler_ =  std::make_shared<InformedSampler>(lb_,ub_,lb_,ub_);
  tree_is_trimmed_ = false;

  goal_conf_ = current_path_->getWaypoints().back(); //ELIMINA
}

void DynamicRRT::fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr>& old_nodes, std::vector<double>& old_connections_costs)
{
  if(success_)
    return;

  if(tree_is_trimmed_) //rebuild the tree adding the old path as a branch
  {
    std::reverse(old_nodes.begin(),old_nodes.end());  //so the node are ordered from the goal to the start
    std::reverse(old_connections_costs.begin(),old_connections_costs.end());

    assert(not trimmed_tree_->isInTree(node_replan));
    assert(old_nodes.front() == goal_node_);

    std::vector<ConnectionPtr> restore_old_path;
    for(unsigned int i=1; i<old_nodes.size();i++) //the goal (i = 0) is in the tree (it is the root)
    {
      if(trimmed_tree_->isInTree(old_nodes.at(i)))
        continue;
      else
      {
        for(unsigned z=i;z<old_nodes.size();z++) //ELIMINA
        {
          if(not old_nodes.at(z)->parent_connections_.empty() || not old_nodes.at(z)->child_connections_.empty())
          {
            for(const NodePtr& n:old_nodes)
              ROS_INFO_STREAM("path node\n"<<*n);

            assert(0);
          }
        }

        for(unsigned int j=i;j<old_nodes.size();j++)
        {
          ConnectionPtr conn = std::make_shared<Connection>(old_nodes.at(j-1),old_nodes.at(j));
          conn->setCost(old_connections_costs.at(j-1));
          conn->add();

          restore_old_path.push_back(conn);
        }
        break;
      }
    }

    trimmed_tree_->addBranch(restore_old_path);
    tree_is_trimmed_ = false;

    trimmed_tree_->changeRoot(root); //the old root (the starting point of the current path)
    current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));

    assert(not trimmed_tree_->isInTree(node_replan));
  }
  else
  {
    assert(trimmed_tree_->isInTree(node_replan));

    if(std::find(old_nodes.begin(),old_nodes.end(),node_replan) == old_nodes.end()) //if the node_replan has been added to the path and tree, remove it. If it was already present, do not remove it
    {
      assert(node_replan->parent_connections_.size() == 1 && node_replan->child_connections_.size() == 1);

      NodePtr parent = node_replan->getParents().front();
      NodePtr child = node_replan->getChildren().front();

      ConnectionPtr conn = std::make_shared<Connection>(parent,child);
      double cost = node_replan->parent_connections_.front()->getCost()+child->parent_connections_.front()->getCost();
      conn->setCost(cost);
      conn->add();

      trimmed_tree_->removeNode(node_replan);
      trimmed_tree_->changeRoot(root);

      current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));

      assert(not trimmed_tree_->isInTree(node_replan));
    }
  }
}

bool DynamicRRT::trimInvalidTree(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  bool trimmed = false;

  TreePtr tree= current_path_->getTree();
  std::vector<ConnectionPtr> node2goal = tree->getConnectionToNode(node); //Note: the root must be the goal (set in regrowRRT())

  std::vector<NodePtr> white_list; //not needed
  unsigned int removed_nodes;      //not needed
  for(const ConnectionPtr &conn: node2goal)
  {
    if((ros::WallTime::now()-tic).toSec()>=max_time_)
      break;

    if(not checker_->checkConnection(conn))
    {
      NodePtr child = conn->getChild();
      tree->purgeFromHere(child,white_list,removed_nodes); //remove the successors and the connection from parent to child

      trimmed = true;
      break;
    }
  }

  trimmed_tree_ = tree;

  return trimmed;
}

bool DynamicRRT::regrowRRT(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;

  //First thing to do: set the goal as the root
  if(not current_path_->getTree()->changeRoot(goal_node_)) //revert the tree so the goal is the root
  {
    ROS_ERROR("The goal can't be set as root!");
    ROS_INFO_STREAM("Goal node: "<<goal_node_<<"\n"<<*goal_node_);
    ROS_INFO_STREAM("Current path end node: "<<current_path_->getConnections().back()->getChild()<<"\n"<<*current_path_->getConnections().back()->getChild());

    assert(0);
  }

  if(not tree_is_trimmed_)
  {
    //Trim the tree
    if(not trimInvalidTree(node))
    {
      if(verbose_)
        ROS_INFO("Tree not trimmed");

      tree_is_trimmed_ = false;
      return false;
    }
    else
      tree_is_trimmed_ = true;
  }

  //Regrow the tree
  double max_distance = trimmed_tree_->getMaximumDistance();
  //InformedSampler sampler(lb_,ub_,lb_,ub_);

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && not success_)
  {
    NodePtr new_node;
    Eigen::VectorXd conf = sampler_->sample(); //CHIEDI A MANUEL PERCHE CAPITA CHE VENGA CAMPIONATA LA STESSA CONFIGURAZIONE PIU VOLTE
    if(trimmed_tree_->extend(conf,new_node))
    {
      ROS_INFO_STREAM("new_node: "<<new_node->getConfiguration().transpose()<<" conf: "<<conf.transpose()); //ELIMINA
      if((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if(checker_->checkPath(new_node->getConfiguration(), node->getConfiguration()))
        {
          if(not node->parent_connections_.empty() && not node->child_connections_.empty())
          {
            ROS_INFO_STREAM("node:\n"<<*node);
            assert(0);
          }

          ConnectionPtr conn = std::make_shared<Connection>(new_node, node);
          conn->setCost(metrics_->cost(new_node, node));
          conn->add();

          trimmed_tree_->addNode(node);

          //Set the root in the node and extract the new path
          trimmed_tree_->changeRoot(node);
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
          replanned_path_->setTree(trimmed_tree_);

          // SOLUZIONE MOMENTANEA
          for(unsigned int i=0;i<replanned_path_->getConnections().size();i++)
          {
            if(replanned_path_->getConnections().at(i)->norm() <1e-06)
            {
              if(replanned_path_->getConnections().at(i)->getParent()->child_connections_.size() == 1)
              {
                ConnectionPtr conn = std::make_shared<Connection>(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                double cost = metrics_->cost(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                conn->setCost(cost);
                conn->add();

                replanned_path_->getConnections().at(i)->remove();

                replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
                break;
              }
              else if(replanned_path_->getConnections().at(i)->getChild()->child_connections_.size() == 1)
              {
                ConnectionPtr conn = std::make_shared<Connection>(replanned_path_->getConnections().at(i)->getParent(),replanned_path_->getConnections().at(i+1)->getChild());
                double cost = metrics_->cost(replanned_path_->getConnections().at(i)->getParent(),replanned_path_->getConnections().at(i+1)->getChild());
                conn->setCost(cost);
                conn->add();

                replanned_path_->getConnections().at(i)->remove();

                replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
                break;
              }
            }
          }
          // FINO A QUA

          solver_->setStartTree(trimmed_tree_);
          solver_->setSolution(replanned_path_,true);

          tree_is_trimmed_ = false;

          success_ = true;
          break;
        }
      }
    }
    time = (ros::WallTime::now()-tic).toSec();
  }

  return success_;
}

bool DynamicRRT::replan()
{
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    std::vector<Eigen::VectorXd> old_wp; //ELIMINA
    old_wp = current_path_->getWaypoints(); //ELIMINA

    for(const Eigen::VectorXd& wp:old_wp)
      ROS_INFO_STREAM("old wp: "<<wp.transpose());

    std::vector<NodePtr> path_nodes = current_path_->getNodes();  //save nodes pointers (the same pointers stored in the tree)
    assert(path_nodes.front() == current_path_->getTree()->getRoot());

    std::vector<double> connections_costs;
    for(const ConnectionPtr& conn:current_path_->getConnections())
      connections_costs.push_back(conn->getCost());

    for(const NodePtr& n:path_nodes)
      assert(current_path_->getTree()->isInTree(n));

    NodePtr root = current_path_->getTree()->getRoot();
    assert(root == current_path_->getConnections().front()->getParent());

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    if(std::find(path_nodes.begin(),path_nodes.end(),node_replan) == path_nodes.end()) //ELIMINA
    {
      for(const Eigen::VectorXd& wp:old_wp)
      {
        if(wp == node_replan->getConfiguration())
        {
          for(const Eigen::VectorXd& wp2:old_wp)
            ROS_INFO_STREAM("2old wp: "<<wp2.transpose());
          assert(0);
        }
      }
    }

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(not regrowRRT(node_replan)) //root is goal
      fixTree(node_replan,root,path_nodes,connections_costs);

    if(not success_) //ELIMINA
    {
      std::vector<Eigen::VectorXd> wp_now = current_path_->getWaypoints();

      if(old_wp.size() != wp_now.size())
      {
        for(const Eigen::VectorXd& w:old_wp)
          ROS_INFO_STREAM("old wp: "<<w.transpose());

        for(const Eigen::VectorXd& w:wp_now)
          ROS_INFO_STREAM("now wp: "<<w.transpose());

        assert(0);
      }

      for(unsigned int i = 0; i<old_wp.size(); i++)
      {
        if(old_wp.at(i) != wp_now.at(i))
        {
          for(const Eigen::VectorXd& w:old_wp)
            ROS_INFO_STREAM("old wp: "<<w.transpose());

          for(const Eigen::VectorXd& w:wp_now)
            ROS_INFO_STREAM("now wp: "<<w.transpose());

          assert(0);
        }
      }
    }
  }
  else //replan not needed
  {
    assert(current_path_->isValidFromConf(current_configuration_));

    success_ = false;
    replanned_path_ = current_path_;
  }

  checkPathConn(replanned_path_); //ELIMINA

  return success_;
}


//bool DynamicRRT::replan()
//{
//  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

//  if(cost_from_conf == std::numeric_limits<double>::infinity())
//  {
//    NodePtr node_replan;
//    std::vector<ConnectionPtr> connections_without_replan_node;
//    std::vector<Eigen::VectorXd> wp; //ELIMINA

//    if(not tree_is_trimmed_)
//    {
//      wp = current_path_->getWaypoints(); //ELIMINA


//      connections_without_replan_node = current_path_->getConnections();

//      ConnectionPtr conn = current_path_->findConnection(current_configuration_);
//      node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);
//    }
//    else
//    {
//      node_replan = std::make_shared<Node>(current_configuration_);
//    }

//    if(verbose_)
//      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

//    regrowRRT(node_replan);

//    if((not success_) && (not connections_without_replan_node.empty()))
//    {
//      //regrowRRT/trimInvalidTree is failed but you have added a node to the current path, you should remove it

//      if(tree_is_trimmed_) //regrowRRT failed, node_replan is not in the tree (tree is trimmed), remove only from the path
//      {
//        assert(not current_path_->getTree()->isInTree(node_replan));
//        current_path_->setConnections(connections_without_replan_node);
//      }
//      else //trimInvalidTree is failed, node_replan belongs to the tree, remove from the path and from the tree
//      {
//        assert(current_path_->getTree()->isInTree(node_replan));
//        assert(node_replan->parent_connections_.size() == 1 && node_replan->child_connections_.size() == 1);

//        NodePtr parent = node_replan->getParents().front();
//        NodePtr child = node_replan->getChildren().front();

//        ConnectionPtr conn = std::make_shared<Connection>(parent,child);
//        double cost = metrics_->cost(parent->getConfiguration(),child->getConfiguration());
//        conn->setCost(cost);
//        conn->add();

//        current_path_->getTree()->removeNode(node_replan);
//        assert(not current_path_->getTree()->isInTree(node_replan));

//        current_path_->setConnections(connections_without_replan_node);
//      }
//    }

//    if(not success_ && (not wp.empty()))
//    {
//      std::vector<Eigen::VectorXd> wp_now = current_path_->getWaypoints();

//      if(wp.size() != wp_now.size())
//      {
//        for(const Eigen::VectorXd& w:wp)
//          ROS_INFO_STREAM("old wp: "<<w.transpose());

//        for(const Eigen::VectorXd& w:wp_now)
//          ROS_INFO_STREAM("now wp: "<<w.transpose());

//        assert(0);
//      }

//      for(unsigned int i = 0; i<wp.size(); i++)
//      {
//        if(wp.at(i) != wp_now.at(i))
//        {
//          for(const Eigen::VectorXd& w:wp)
//            ROS_INFO_STREAM("old wp: "<<w.transpose());

//          for(const Eigen::VectorXd& w:wp_now)
//            ROS_INFO_STREAM("now wp: "<<w.transpose());

//          assert(0);
//        }
//      }
//    }
//  }
//  else //replan not needed
//  {
//    assert(current_path_->isValidFromConf(current_configuration_));

//    success_ = false;
//    replanned_path_ = current_path_;
//  }

//  checkPathConn(replanned_path_); //ELIMINA

//  return success_;
//}

}
