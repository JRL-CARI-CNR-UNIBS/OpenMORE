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
    Eigen::VectorXd conf = sampler_->sample(); //sistema
    if(trimmed_tree_->extend(conf,new_node))
    {
      ROS_INFO_STREAM("new_node: "<<new_node->getConfiguration().transpose()<<" conf: "<<conf.transpose()); //ELIMINA
      if((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if(checker_->checkPath(new_node->getConfiguration(), node->getConfiguration()))
        {
          assert(node->parent_connections_.empty() && node->child_connections_.empty());

          ConnectionPtr conn = std::make_shared<Connection>(new_node, node);
          conn->setCost(metrics_->cost(new_node, node));
          conn->add();

          trimmed_tree_->addNode(node);

          //Set the root in the node and extract the new path
          trimmed_tree_->changeRoot(node);
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
          replanned_path_->setTree(trimmed_tree_);

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
    NodePtr node_replan;
    std::vector<ConnectionPtr> connections_without_replan_node;
    std::vector<Eigen::VectorXd> wp; //ELIMINA

    if(not tree_is_trimmed_)
    {
      wp = current_path_->getWaypoints(); //ELIMINA


      connections_without_replan_node = current_path_->getConnections();

      ConnectionPtr conn = current_path_->findConnection(current_configuration_);
      node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);
    }
    else
    {
      node_replan = std::make_shared<Node>(current_configuration_);
    }

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    regrowRRT(node_replan);

    if((not success_) && (not connections_without_replan_node.empty()))
    {
      //regrowRRT/trimInvalidTree is failed but you have added a node to the current path, you should remove it

      if(tree_is_trimmed_) //regrowRRT failed, node_replan is not in the tree (tree is trimmed), remove only from the path
      {
        assert(not current_path_->getTree()->isInTree(node_replan));
        current_path_->setConnections(connections_without_replan_node);
      }
      else //trimInvalidTree is failed, node_replan belongs to the tree, remove from the path and from the tree
      {
        assert(current_path_->getTree()->isInTree(node_replan));
        assert(node_replan->parent_connections_.size() == 1 && node_replan->child_connections_.size() == 1);

        NodePtr parent = node_replan->getParents().front();
        NodePtr child = node_replan->getChildren().front();

        ConnectionPtr conn = std::make_shared<Connection>(parent,child);
        double cost = metrics_->cost(parent->getConfiguration(),child->getConfiguration());
        conn->setCost(cost);
        conn->add();

        current_path_->getTree()->removeNode(node_replan);
        assert(not current_path_->getTree()->isInTree(node_replan));

        current_path_->setConnections(connections_without_replan_node);
      }
    }

    if(not success_ && (not wp.empty()))
    {
      std::vector<Eigen::VectorXd> wp_now = current_path_->getWaypoints();

      if(wp.size() != wp_now.size())
      {
        for(const Eigen::VectorXd& w:wp)
          ROS_INFO_STREAM("old wp: "<<w.transpose());

        for(const Eigen::VectorXd& w:wp_now)
          ROS_INFO_STREAM("now wp: "<<w.transpose());

        assert(0);
      }

      for(unsigned int i = 0; i<wp.size(); i++)
      {
        if(wp.at(i) != wp_now.at(i))
        {
          for(const Eigen::VectorXd& w:wp)
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

}
