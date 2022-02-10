#include "replanners_lib/replanners/DRRT.h"

namespace pathplan
{

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
  tree_is_trimmed_ = false;
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
    assert(0);
  }

  if(not tree_is_trimmed_)
  {
    //Trim the tree
    if(not trimInvalidTree(node))
    {
      if(verbose_)
        ROS_INFO("Tree not trimmed");
      return false;
    }
    else
      tree_is_trimmed_ = true;
  }

  //Regrow the tree
  double max_distance = trimmed_tree_->getMaximumDistance();
  InformedSampler sampler (lb_,ub_,lb_,ub_);

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && not success_)
  {
    NodePtr new_node;
    if(trimmed_tree_->extend(sampler.sample(),new_node))
    {
      if((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if(checker_->checkPath(new_node->getConfiguration(), node->getConfiguration()))
        {
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

    if(not tree_is_trimmed_)
    {
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
  }
  else //replan not needed
  {
    assert(current_path_->isValidFromConf(current_configuration_));

    success_ = false;
    replanned_path_ = current_path_;
  }

  return success_;
}

}
