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

  ROS_INFO_STREAM("type: "<<ti2.name());

  RRTPtr tmp_solver;

  if(std::type_index(ti1) != std::type_index(ti2))
  {
    ROS_INFO("NO RRT");

    tmp_solver = std::make_shared<pathplan::RRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
  {
    ROS_INFO("RRT");
    tmp_solver = std::static_pointer_cast<RRT>(solver);
  }

  solver_ = tmp_solver;
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

    if(!checker_->checkConnection(conn))
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
  NodePtr initial_goal = current_path_->getConnections().back()->getChild();
  current_path_->getTree()->changeRoot(initial_goal); //revert the tree so the goal is the root

  //Trim the tree
  if(!trimInvalidTree(node))
  {
    ROS_INFO("Tree not trimmed");
    return false;
  }

  //Regrow the tree
  double max_distance = trimmed_tree_->getMaximumDistance();
  InformedSampler sampler (lb_,ub_,lb_,ub_);

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && !success_)
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
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(initial_goal), metrics_, checker_);
          replanned_path_->setTree(trimmed_tree_);

          solver_->setStartTree(trimmed_tree_);
          solver_->setSolution(replanned_path_,true);

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
