#include "DRRTstar.h"

namespace pathplan
{

bool DynamicRRTstar::nodeBehindObs(NodePtr& node_behind)
{
  for(int i=current_path_->getConnections().size()-1; i>=0; i--)
  {
    if(current_path_->getConnections().at(i)->getCost() == std::numeric_limits<double>::infinity())
    {
      node_behind = current_path_->getConnections().at(i)->getChild();
      return true;
    }
  }
  return false;
}

bool DynamicRRTstar::connectBehindObs(NodePtr& node)
{
  success_ = false;
  TreePtr tree = current_path_->getTree();

  if(!tree->isInTree(node))
  {
    ROS_ERROR("The starting node for replanning doesn't belong to the tree");
    return false;
  }

  NodePtr replan_goal;
  if(!nodeBehindObs(replan_goal)) return false;

  double radius = 2*((replan_goal->getConfiguration()-node->getConfiguration()).norm());
  InformedSampler sampler (node->getConfiguration(),node->getConfiguration(),lb_,ub_,radius);

  //*  STEP 1: REWIRING  *//
  tree->rewireOnly(node,radius);

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  unsigned int n_existing_nodes = (tree->near(replan_goal,radius)).size();
  if(n_existing_nodes<1000)
  {
    unsigned int n_new_nodes = (unsigned int) n_existing_nodes/2;
    ROS_INFO_STREAM("Adding "<<n_new_nodes<<" new nodes inside the proximity circle with readius "<<radius);
    for(unsigned int i=0;i<n_new_nodes;i++)
    {
      Eigen::VectorXd sample = sampler.sample();
      NodePtr new_node = std::make_shared<Node>(sample);

      tree->addNode(new_node);
    }

    tree->rewireOnly(node,radius);
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
