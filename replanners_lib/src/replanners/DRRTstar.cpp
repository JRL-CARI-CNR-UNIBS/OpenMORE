#include "DRRTstar.h"

namespace pathplan
{

DynamicRRTstar::DynamicRRTstar(Eigen::VectorXd& current_configuration,
                               PathPtr& current_path,
                               const double& max_time,
                               const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(RRTStarPtr);
  const std::type_info& ti2 = typeid(solver);

  ROS_INFO_STREAM("SOLVER TYPE: "<<ti2.name());
  if(std::type_index(ti1) != std::type_index(ti2))
  {
    solver_ = std::make_shared<pathplan::RRTStar>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
  }
}

bool DynamicRRTstar::nodeBehindObs(NodePtr& node_behind)
{
  for(int i=current_path_->getConnections().size()-1; i>=0; i--)
  {
    if(current_path_->getConnections().at(i)->getCost() == std::numeric_limits<double>::infinity())
    {
      if(i < current_path_->getConnections().size()-1)
      {
        node_behind = current_path_->getConnections().at(i+1)->getChild();
      }
      else
      {
        node_behind = current_path_->getConnections().at(i)->getChild();
      }

      return true;
    }
  }
  return false;
}

bool DynamicRRTstar::connectBehindObs(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  TreePtr tree = current_path_->getTree();

  if(!tree->isInTree(node))
  {
    ROS_ERROR("The starting node for replanning doesn't belong to the tree");
    return false;
  }

  NodePtr replan_goal;
  if(!nodeBehindObs(replan_goal)) return false;

  double radius = 1.5*((replan_goal->getConfiguration()-node->getConfiguration()).norm());
  InformedSampler sampler (node->getConfiguration(),node->getConfiguration(),lb_,ub_,radius);

  //*  STEP 1: REWIRING  *//

  //change root!!!!
  tree->rewireOnly(node,radius,2);

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  ros::WallTime toc = ros::WallTime::now();
  while(((toc-tic).toSec()-max_time_)>0.0)
  {
    NodePtr new_node;
    Eigen::VectorXd q=sampler.sample();

    if (tree->rewire(q,radius,new_node))
    {
      if ((new_node->getConfiguration()-replan_goal->getConfiguration()).norm()<1e-03)
      {
        if(checker_->checkPath(new_node->getConfiguration(),replan_goal->getConfiguration()))
        {
          double cost = metrics_->cost(new_node->getConfiguration(),replan_goal->getConfiguration());
          Connection conn = Connection(new_node,replan_goal);
          conn.setCost(cost);
          conn.add();

          break;
        }
      }
    }
    toc = ros::WallTime::now();
  }

  PathPtr connecting_path = std::make_shared<Path>(tree->getConnectionToNode(replan_goal),metrics_,checker_); //Passa per il mio nodo?
  connecting_path = connecting_path->getSubpathFromNode(node);

  std::vector<ConnectionPtr> new_connections = connecting_path->getConnections();
  std::vector<ConnectionPtr> subpath_connections = current_path_->getSubpathFromNode(replan_goal)->getConnections();
  new_connections.insert(new_connections.end(),subpath_connections.begin(),subpath_connections.end());

  replanned_path_ = std::make_shared<Path>(new_connections,metrics_,checker_);

  return success_;
}

bool DynamicRRTstar::replan()
{
  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    connectBehindObs(node_replan);
  }
  else //replan not needed
  {
    success_ = false;
    replanned_path_ = current_path_;
  }

  return success_;
}

}
