#include "replanners_lib/replanners/DRRTStar.h"

namespace pathplan
{
DynamicRRTStar::DynamicRRTStar(Eigen::VectorXd& current_configuration,
                               PathPtr& current_path,
                               const double& max_time,
                               const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(RRTStar);
  const std::type_info& ti2 = typeid(*solver);

  RRTStarPtr tmp_solver;

  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<pathplan::RRTStar>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
    tmp_solver = std::static_pointer_cast<RRTStar>(solver);

  solver_ = tmp_solver;
}

bool DynamicRRTStar::nodeBehindObs(NodePtr& node_behind)
{
  for(int i=current_path_->getConnectionsSize()-1; i>=0; i--)
  {
    if(current_path_->getConnections().at(i)->getCost() == std::numeric_limits<double>::infinity())
    {
      (i<current_path_->getConnectionsSize()-1)?
            (node_behind=current_path_->getConnections().at(i+1)->getChild()):
            (node_behind=current_path_->getConnections().at(i)->getChild());

      if(verbose_)
        ROS_INFO_STREAM("Replanning goal: \n"<< *node_behind);

      return true;
    }
  }

  ROS_ERROR("Gaol behind obstacle not found");
  return false;
}

bool DynamicRRTStar::connectBehindObs(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  TreePtr tree = current_path_->getTree();
  NodePtr root = tree->getRoot();

  if(not tree->isInTree(node))
  {
    ROS_ERROR("The starting node for replanning doesn't belong to the tree");
    return false;
  }

  NodePtr replan_goal;
  if(not nodeBehindObs(replan_goal))
    return false;

  double radius = 1.5*((replan_goal->getConfiguration()-node->getConfiguration()).norm());
  LocalInformedSampler sampler (node->getConfiguration(),replan_goal->getConfiguration(),lb_,ub_,std::numeric_limits<double>::infinity());
  sampler.addBall(node->getConfiguration(),radius);

  //*  STEP 1: REWIRING  *//
  std::vector<ConnectionPtr> checked_connections;
  std::vector<NodePtr> white_list = current_path_->getNodes();  //first save the path nodes

  tree->changeRoot(node);  //then change the root
  tree->rewireOnlyWithPathCheck(node,checked_connections,radius,white_list,2); //rewire only children

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  double max_distance = tree->getMaximumDistance();

  std::vector<NodePtr> black_list;
  black_list.push_back(replan_goal);
  SubtreePtr subtree = Subtree::createSubtree(tree,node,black_list);

  if(disp_ && verbose_)
    disp_->changeNodeSize({0.01,0.01,0.01});

  double cost2goal = std::numeric_limits<double>::infinity();
  double distance_new_node_goal, cost2new_node;

  double max_time = 0.98*max_time_;
  double time = (ros::WallTime::now()-tic).toSec();
  while(time<0.98*max_time)
  {
    NodePtr new_node;
    Eigen::VectorXd q=sampler.sample();

    if(subtree->rewireWithPathCheck(q,checked_connections,radius,white_list,new_node))
    {
      if(disp_ && verbose_)
        disp_->displayNode(new_node);

      assert(replan_goal->getParents().size() == 1);

      distance_new_node_goal = (new_node->getConfiguration()-replan_goal->getConfiguration()).norm();
      if(distance_new_node_goal > max_distance)
        continue;

      cost2new_node = subtree->costToNode(new_node);

      if((cost2new_node+distance_new_node_goal) < cost2goal)
      {
        if(checker_->checkPath(new_node->getConfiguration(),replan_goal->getConfiguration()))
        {
          if(not replan_goal->parent_connections_.empty())
          {
            replan_goal->parent_connections_.front()->remove(); //delete the connection between replan_goal and the old parent
            replan_goal->parent_connections_.clear();           //remove the old parent connections because now the parents of replan_goal come frome new_node
          }

          double cost = metrics_->cost(new_node->getConfiguration(),replan_goal->getConfiguration());
          ConnectionPtr conn = std::make_shared<Connection>(new_node,replan_goal);
          conn->setCost(cost);
          conn->add();

          cost2goal = cost+cost2new_node;

          success_ = true;
        }
      }
    }

    time = (ros::WallTime::now()-tic).toSec();
  }

  if(disp_ && verbose_)
    disp_->defaultNodeSize();

  if(success_)
  {
    std::vector<ConnectionPtr> new_connections = tree->getConnectionToNode(goal_node_);

    replanned_path_ = std::make_shared<Path>(new_connections,metrics_,checker_);
    replanned_path_->setTree(tree);

    assert(replanned_path_->cost()<std::numeric_limits<double>::infinity());
    assert(replanned_path_->isValid());

    solver_->setStartTree(tree);
    solver_->setSolution(replanned_path_);

    tree->changeRoot(root);
  }

  return success_;
}

bool DynamicRRTStar::replan()
{
  success_ = false;
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    if(verbose_)
      ROS_WARN("Current path obstructed");

    std::vector<NodePtr> nodes = current_path_->getNodes();

    NodePtr root = current_path_->getTree()->getRoot();
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);

    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true,is_a_new_node_);

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    connectBehindObs(node_replan);

    if(success_)
    {
      if(disp_ && verbose_)
      {
        disp_->clearMarkers();
        disp_->displayTree(current_path_->getTree());
      }

      return true;  //path is changed
    }
    else
    {
      if(not current_path_->getTree()->changeRoot(root))
      {
        ROS_ERROR("Root can't be restored");
        assert(0);
      }
      assert(((root != node_replan) && (current_path_->getTree()->getRoot() != node_replan)) || (root == node_replan));

      if(current_path_->removeNode(node_replan,nodes)) //if added node is removed the path is not changed, otherwise it is changed
      {
        if(verbose_)
          ROS_INFO("Node replan removed");

        return false; //path is no changed
      }
      else
      {
        if(verbose_)
          ROS_INFO("Node replan not removed");

        return true;  //path is changed
      }
    }
  }
  else //replan not needed
  {
    assert(current_path_->isValidFromConf(current_configuration_));

    success_ = false;
    replanned_path_ = current_path_;

    return false;
  }
}
}
