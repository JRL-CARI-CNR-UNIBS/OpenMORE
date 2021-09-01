#include "DRRTstar.h"

namespace pathplan
{

DynamicRRTstar::DynamicRRTstar(Eigen::VectorXd& current_configuration,
                               PathPtr& current_path,
                               const double& max_time,
                               const TreeSolverPtr &solver): ReplannerBase(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(RRTStar);
  const std::type_info& ti2 = typeid(*solver);

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

      ROS_INFO_STREAM("Replanning goal: \n"<< *node_behind);
      return true;
    }
  }

  ROS_ERROR("Gaol behind obstacle not found");
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

  double radius = 1.2*((replan_goal->getConfiguration()-node->getConfiguration()).norm());
  //  InformedSampler sampler (node->getConfiguration(),node->getConfiguration(),lb_,ub_,radius); DIVISIONE PER ZERO NELLA MATRICE DI ROTAZIONE!
  InformedSampler sampler (node->getConfiguration(),replan_goal->getConfiguration(),lb_,ub_,std::numeric_limits<double>::infinity());

  std::vector<ConnectionPtr> checked_connections;

  int cont = 0;
  for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
  {
    if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
      cont +=1;
  }
  ROS_INFO_STREAM("NODES DISCONNECTED: "<<cont);

  //*  STEP 1: REWIRING  *//
  tree->changeRoot(node);

  cont = 0;
  for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
  {
    if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
      cont +=1;
  }
  ROS_INFO_STREAM("NODES DISCONNECTED: "<<cont);

  tree->rewireOnlyWithPathCheck(node,checked_connections,radius,2);

  cont = 0;
  for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
  {
    if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
      cont +=1;
  }
  ROS_INFO_STREAM("NODES DISCONNECTED: "<<cont);

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  double max_distance = tree->getMaximumDistance();

  if(disp_ != NULL) disp_->changeNodeSize({0.01,0.01,0.01});

  ros::WallTime toc = ros::WallTime::now();
  while(max_time_-((toc-tic).toSec())>0.0)
  {
    NodePtr new_node;
    Eigen::VectorXd q=sampler.sample();

    /*ELIMINA*/
    bool skip = false;
    if((q-node->getConfiguration()).norm()>radius) skip = true;
    /*      */

    if(!skip) //ELIMINA
    {
      ROS_INFO("PRIMA REWIRE");
      if(tree->rewireWithPathCheck(q,checked_connections,radius,new_node))
      {
        int cont =0;
        for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
        {
          if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
            cont +=1;
        }
        if(cont>0) ROS_INFO_STREAM("DOPO REWIRE NODES DISCONNECTED: "<<cont);

        ROS_INFO("DOPO REWIRE");

        //        if(disp_ != NULL) disp_->displayNode(new_node);

        if(replan_goal->getParents().at(0) == new_node)
        {
          success_ = true;
        }

        if(!success_)  //if success, i should not try to connect to goal but only rewire to improve the path
        {
          if ((new_node->getConfiguration()-replan_goal->getConfiguration()).norm()<max_distance)
          {
            if(checker_->checkPath(new_node->getConfiguration(),replan_goal->getConfiguration()))
            {
              cont = 0;
              for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
              {
                if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
                  cont +=1;
              }
              if(cont>0)ROS_INFO_STREAM("IN1 WHILE NODES DISCONNECTED: "<<cont);


              std::vector<ConnectionPtr> conn2new_node;
              if(replan_goal->getParents().size()>0)
              {
                replan_goal->parent_connections_.clear();  //remove the old parent connections because now the parents of replan_goal come frome new_node
                conn2new_node = new_node->parent_connections_;
              }

              double cost = metrics_->cost(new_node->getConfiguration(),replan_goal->getConfiguration());
              ConnectionPtr conn = std::make_shared<Connection>(new_node,replan_goal);
              conn->setCost(cost);

              conn->add(); //add connection between new_node and replan_goal as first connection in parent_connections of replan_goal

              if(!conn2new_node.empty())  //and THEN add all the connections from start to new_node in parent_connections of replan_goal
              {
                replan_goal->parent_connections_.insert(replan_goal->parent_connections_.end(),conn2new_node.begin(),conn2new_node.end());
              }

              cont = 0;
              for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
              {
                if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
                  cont +=1;
              }
              if(cont>0) ROS_INFO_STREAM("IN2 WHILE NODES DISCONNECTED: "<<cont);

              success_ = true;
            }
          }
        }
      }

      cont = 0;
      for(const NodePtr tree_node : tree->getNodes()) //ELIMINA
      {
        if(tree_node->getParents().size() == 0 && tree_node->getChildren().size() == 0)
          cont +=1;
      }
      if(cont>0)ROS_INFO_STREAM("END WHILE NODES DISCONNECTED: "<<cont);
      ROS_INFO("---------------------------");
    }
    toc = ros::WallTime::now();
  }

  if(disp_ != NULL) disp_->defaultNodeSize();

  if(success_)
  {
    PathPtr connecting_path = std::make_shared<Path>(tree->getConnectionToNode(replan_goal),metrics_,checker_);
    std::vector<ConnectionPtr> new_connections = connecting_path->getConnections();

    if(replan_goal->getConfiguration() != current_path_->getWaypoints().back())
    {
      std::vector<ConnectionPtr> subpath_connections = current_path_->getSubpathFromNode(replan_goal)->getConnections();
      new_connections.insert(new_connections.end(),subpath_connections.begin(),subpath_connections.end());
    }

    replanned_path_ = std::make_shared<Path>(new_connections,metrics_,checker_);
  }

  return success_;
}

bool DynamicRRTstar::replan()
{
  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

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
