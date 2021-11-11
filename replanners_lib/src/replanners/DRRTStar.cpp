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
  for(int i=current_path_->getConnections().size()-1; i>=0; i--)
  {
    if(current_path_->getConnections().at(i)->getCost() == std::numeric_limits<double>::infinity())
    {
      if(i < current_path_->getConnections().size()-1)
        node_behind = current_path_->getConnections().at(i+1)->getChild();
      else
        node_behind = current_path_->getConnections().at(i)->getChild();

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

  bool tree_modified = true;
  success_ = false;
  TreePtr tree = current_path_->getTree();

  if(!tree->isInTree(node))
  {
    ROS_ERROR("The starting node for replanning doesn't belong to the tree");
    return false;
  }

  NodePtr replan_goal;
  if(!nodeBehindObs(replan_goal)) return false;

  double radius = 1*((replan_goal->getConfiguration()-node->getConfiguration()).norm());
  LocalInformedSampler sampler (node->getConfiguration(),replan_goal->getConfiguration(),lb_,ub_,std::numeric_limits<double>::infinity());
  sampler.addBall(node->getConfiguration(),radius);

  //*  STEP 1: REWIRING  *//
  tree->changeRoot(node);

  std::vector<ConnectionPtr> checked_connections;
  tree->rewireOnlyWithPathCheck(node,checked_connections,radius,2);

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  double max_distance = tree->getMaximumDistance();

  if(disp_)
    disp_->changeNodeSize({0.01,0.01,0.01});

  ros::WallTime toc = ros::WallTime::now();
  while(max_time_-((toc-tic).toSec())>0.0)
  {
    NodePtr new_node;
    Eigen::VectorXd q=sampler.sample();

    if(tree->rewireWithPathCheck(q,checked_connections,radius,new_node))
    {
      tree_modified = true;  //at least a rewiring has been done

      if(disp_)
        disp_->displayNode(new_node);

      if(replan_goal->getParents().at(0) == new_node)
      {
        success_ = true;
      }

      if(!success_)  //if success, i should not try to connect to goal but only rewire to improve the path
      {
        if((new_node->getConfiguration()-replan_goal->getConfiguration()).norm()<max_distance)
        {
          if(checker_->checkPath(new_node->getConfiguration(),replan_goal->getConfiguration()))
          {
            if(replan_goal->getParents().size()>0)
            {
              replan_goal->parent_connections_.at(0)->remove(); //delete the connection between replan_goal and the old parent
              replan_goal->parent_connections_.clear();         //remove the old parent connections because now the parents of replan_goal come frome new_node
            }

            double cost = metrics_->cost(new_node->getConfiguration(),replan_goal->getConfiguration());
            ConnectionPtr conn = std::make_shared<Connection>(new_node,replan_goal);
            conn->setCost(cost);

            conn->add(); //add connection between new_node (the new parent) and replan_goal in parent_connections of replan_goal

            success_ = true;
          }
        }
      }
    }
    toc = ros::WallTime::now();
  }

  if(disp_)
    disp_->defaultNodeSize();

  if(tree_modified)
  {
    std::vector<ConnectionPtr> new_connections = tree->getConnectionToNode(replan_goal);

    if(replan_goal->getConfiguration() != current_path_->getWaypoints().back())
    {
      std::vector<ConnectionPtr> subpath_connections = current_path_->getSubpathFromNode(replan_goal)->getConnections();
      new_connections.insert(new_connections.end(),subpath_connections.begin(),subpath_connections.end());
    }

    replanned_path_ = std::make_shared<Path>(new_connections,metrics_,checker_);
    replanned_path_->setTree(tree);

    solver_->setStartTree(tree);
    solver_->setSolution(replanned_path_);
  }

  return tree_modified;
}

bool DynamicRRTStar::replan()
{
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  ROS_INFO("COST FROM CONF: %f",cost_from_conf);

  ROS_INFO_STREAM("ROOT IN REPLAN: "<< *(replanned_path_->getTree()->getRoot())<<"\n"<<replanned_path_->getTree()->getRoot());

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    NodePtr root = current_path_->getTree()->getRoot();
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);

    NodePtr parent = conn->getParent();
    NodePtr child = conn->getChild();

    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);
    ROS_INFO_STREAM("node replan in tree: "<<current_path_->getTree()->isInTree(node_replan));

    bool tree_modified =  connectBehindObs(node_replan);

    ROS_INFO_STREAM("NUOVA ROOT IN REPLAN: "<< *(replanned_path_->getTree()->getRoot())<<"\n"<<replanned_path_->getTree()->getRoot());

    return tree_modified; // tree can be changed also if success_ == false

//    if(!success_)
//    {
//      if(!current_path_->getTree()->isInTree(root))
//        assert(0);

//      current_path_->getTree()->changeRoot(root);

//      ROS_INFO_STREAM("before removing added node");
//      for(const Eigen::VectorXd& wp:current_path_->getWaypoints())
//        ROS_INFO_STREAM("path node: "<<wp.transpose());

//      current_path_->removeNodeAddedInConn(node_replan);

//      ROS_INFO_STREAM("after removing added node");
//      for(const Eigen::VectorXd& wp:current_path_->getWaypoints())
//        ROS_INFO_STREAM("path node: "<<wp.transpose());
//    }
  }
  else //replan not needed
  {
    success_ = false;
    replanned_path_ = current_path_;

    ROS_INFO_STREAM("NUOVA ROOT IN REPLAN: "<< *(replanned_path_->getTree()->getRoot())<<"\n"<<replanned_path_->getTree()->getRoot());

    return false;
  }
}

}
