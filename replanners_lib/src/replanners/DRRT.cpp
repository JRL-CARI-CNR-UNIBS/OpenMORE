#include "DRRT.h"

namespace pathplan
{

DynamicRRT::DynamicRRT(Eigen::VectorXd& current_configuration,
                       PathPtr& current_path,
                       const double& max_time,
                       const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(RRT);
  const std::type_info& ti2 = typeid(*solver);

  if(std::type_index(ti1) != std::type_index(ti2))
  {
    solver_ = std::make_shared<pathplan::RRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
  }

  goal_conf_ = current_path->getConnections().back()->getChild()->getConfiguration();
}

bool DynamicRRT::trimInvalidTree(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  bool trimmed = false;

  TreePtr tree= current_path_->getTree();
  NodePtr goal_node = current_path_->getConnections().back()->getChild();

  if(!tree->isInTree(goal_node))
    assert(0);

  // 1)Flip the tree
  //tree->changeRoot(goal_node);  //Dovresti calcolare l'albero al contrario fin dal principio..

  // 2)Check for all connections of the tree
  std::vector<ConnectionPtr> connections2goal = tree->getConnectionToNode(goal_node);
  std::vector<ConnectionPtr> node2goal;
  bool from_here = false;
  for(const ConnectionPtr &conn: connections2goal)
  {
    if(!from_here)
    {
      if(conn->getParent() == node)
        from_here = true;
    }
    else
      node2goal.push_back(conn);
  }

  if(!from_here)
    assert(0);

  std::vector<NodePtr> white_list; //not needed
  unsigned int removed_nodes;      //not needed
  for(const ConnectionPtr &conn: node2goal)
  {
    if((ros::WallTime::now()-tic).toSec()>=max_time_)
      break;

    if(!checker_->checkConnection(conn))
    {
      if(!checker_->check(conn->getParent()->getConfiguration()))
      {
        NodePtr parent = conn->getParent();
        tree->purgeFromHere(parent,white_list,removed_nodes);
      }
      else
      {
        NodePtr child = conn->getChild();
        tree->purgeFromHere(child,white_list,removed_nodes);
      }

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

  if(!trimInvalidTree(node))
    return false;

  double max_distance = trimmed_tree_->getMaximumDistance();

  InformedSampler sampler (lb_,ub_,lb_,ub_);

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && !success_)
  {
    NodePtr new_node;
    if (trimmed_tree_->extend(sampler.sample(),new_node))
    {
      if((new_node->getConfiguration() - goal_conf_).norm() < max_distance)
      {
        if (checker_->checkPath(new_node->getConfiguration(), goal_conf_))
        {
          NodePtr goal_node = std::make_shared<Node>(goal_conf_);
          ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node);
          conn->setCost(metrics_->cost(new_node, goal_node));
          conn->add();
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node), metrics_, checker_);
          replanned_path_->setTree(trimmed_tree_);
          trimmed_tree_->addNode(goal_node);
          success_ = true;
          return true;
        }
      }

      time = (ros::WallTime::now()-tic).toSec();
    }
  }
}

bool DynamicRRT::replan()
{
  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    regrowRRT(node_replan);
  }
  else //replan not needed
  {
    success_ = false;
    replanned_path_ = current_path_;
  }

  return success_;
}

}
