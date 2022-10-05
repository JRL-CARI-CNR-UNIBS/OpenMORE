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

bool DynamicRRTStar::nodeBeforeObs(const PathPtr& subpath, NodePtr& node_before)
{
  for(const ConnectionPtr& c:subpath->getConnectionsConst())
  {
    if(c->getCost() == std::numeric_limits<double>::infinity())
    {
      node_before = c->getParent();
      return true;
    }
  }

  return false;
}


bool DynamicRRTStar::nodeBehindObs(NodePtr& node_behind)
{
  for(int i=current_path_->getConnectionsSize()-1; i>=0; i--)
  {
    if(current_path_->getConnections().at(i)->getCost() == std::numeric_limits<double>::infinity())
    {
      (node_behind=current_path_->getConnections().at(i)->getChild());
      //      (i<current_path_->getConnectionsSize()-1)?
      //            (node_behind=current_path_->getConnections().at(i+1)->getChild()):
      //            (node_behind=current_path_->getConnections().at(i)->getChild());

      return true;
    }
  }

  ROS_ERROR("Goal behind obstacle not found");
  return false;
}

bool DynamicRRTStar::connectBehindObs(const NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  TreePtr tree = current_path_->getTree();

  if(not tree->isInTree(node))
  {
    ROS_ERROR("The starting node for replanning doesn't belong to the tree");
    return false;
  }

  NodePtr replan_goal;
  if(not nodeBehindObs(replan_goal))
    return false;

  if(verbose_)
    ROS_INFO_STREAM("Replan goal: \n"<< *replan_goal);

  NodePtr replan_start;
  PathPtr subpath = current_path_->getSubpathFromNode(node);
  if(not nodeBeforeObs(subpath,replan_start))
    return false;

  if(verbose_)
    ROS_INFO_STREAM("Replan start: \n"<< *replan_start);

  double radius = 1.1*((replan_goal->getConfiguration()-replan_start->getConfiguration()).norm())/2;
  Eigen::VectorXd u = (replan_goal->getConfiguration()-replan_start->getConfiguration())/(replan_goal->getConfiguration()-replan_start->getConfiguration()).norm();
  Eigen::VectorXd ball_center = replan_start->getConfiguration()+u*(((replan_goal->getConfiguration()-replan_start->getConfiguration()).norm())/2);
  LocalInformedSampler sampler (replan_start->getConfiguration(),replan_goal->getConfiguration(),lb_,ub_,std::numeric_limits<double>::infinity());
  sampler.addBall(ball_center,radius);

  //*  STEP 1: REWIRING  *//
  std::vector<ConnectionPtr> checked_connections = current_path_->getConnections();
  std::for_each(checked_connections.begin(),checked_connections.end(),[&](ConnectionPtr c) {c->setRecentlyChecked(true);});

  std::vector<NodePtr> white_list = current_path_->getNodes();  //first save the path nodes
  assert(std::find(white_list.begin(),white_list.end(),node        )<white_list.end());
  assert(std::find(white_list.begin(),white_list.end(),replan_start)<white_list.end());
  assert(std::find(white_list.begin(),white_list.end(),replan_goal )<white_list.end());

  if(not tree->changeRoot(replan_start))  //then change the root
  {
    ROS_INFO_STREAM("replan start "<<*replan_start<<replan_start);
    ROS_INFO_STREAM("node "<<*node<<node);

    ROS_INFO_STREAM("current path "<<*current_path_);
    for(const NodePtr& n: current_path_->getNodes())
      ROS_INFO_STREAM("node "<<n<< "in tree "<<tree->isInTree(n));
    throw std::runtime_error("root can't be changed (replan_start)");
  }

  std::vector<NodePtr> black_list;
  black_list.push_back(replan_goal);
  SubtreePtr subtree = Subtree::createSubtree(tree,replan_start,black_list);

  subtree->rewireOnlyWithPathCheck(replan_start,checked_connections,radius,white_list,2); //rewire only children

  //*  STEP 2: ADDING NEW NODES AND SEARCHING WITH RRT*  *//
  if(disp_ && verbose_)
    disp_->changeNodeSize({0.01,0.01,0.01});

  double cost2goal = std::numeric_limits<double>::infinity();
  double distance_new_node_goal, cost2new_node;

  NodePtr new_node;
  Eigen::VectorXd q;

  double max_distance = tree->getMaximumDistance();

  double max_time = 0.98*max_time_;
  double time = (ros::WallTime::now()-tic).toSec();

  while(time<0.98*max_time)
  {
    q = sampler.sample();

    if(subtree->rewireWithPathCheck(q,checked_connections,radius,white_list,new_node))
    {
      if(disp_ && verbose_)
        disp_->displayNode(new_node);

      assert(replan_goal->getParents().size() == 1);

      distance_new_node_goal = (new_node->getConfiguration()-replan_goal->getConfiguration()).norm();
      if(distance_new_node_goal > max_distance)
        continue;

      cost2new_node = subtree->costToNode(new_node);

      if((cost2new_node+distance_new_node_goal)<cost2goal)
      {
        if(checker_->checkPath(new_node->getConfiguration(),replan_goal->getConfiguration()))
        {
          if(replan_goal->getParentConnectionsSize() != 0)
          {
            replan_goal->parentConnection(0)->remove();    //delete the connection between replan_goal and the old parent, because now the parents of replan_goal come from new_node
            assert(replan_goal->getParentConnectionsSize() == 0);
          }

          double cost = metrics_->cost(new_node->getConfiguration(),replan_goal->getConfiguration());
          ConnectionPtr conn = std::make_shared<Connection>(new_node,replan_goal);
          conn->setCost(cost);
          conn->add();

          conn->setRecentlyChecked(true);
          checked_connections.push_back(conn);

          cost2goal = cost+cost2new_node;

          success_ = true;
        }
      }
    }

    time = (ros::WallTime::now()-tic).toSec();
  }

  if(disp_ && verbose_)
    disp_->defaultNodeSize();

  if(not tree->changeRoot(node))
    throw std::runtime_error("root can't be changed");

  if(success_)
  {
    std::vector<ConnectionPtr> new_connections = tree->getConnectionToNode(goal_node_);

    replanned_path_ = std::make_shared<Path>(new_connections,metrics_,checker_);
    replanned_path_->setTree(tree);

    assert(replanned_path_->cost()<std::numeric_limits<double>::infinity());
    assert([&]()->bool{
             if(not replanned_path_->isValid())
             {
               if(replanned_path_->getConnectionsConst().front()->getCost() == std::numeric_limits<double>::infinity())
               return true;

               ROS_INFO_STREAM(*replanned_path_);
               return false;
             }
             return true;
           }());

    solver_->setStartTree(tree);
    solver_->setSolution(replanned_path_);
  }

  std::for_each(checked_connections.begin(),checked_connections.end(),[&](ConnectionPtr c) {c->setRecentlyChecked(false);});

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

    NodePtr root = current_path_->getStartNode();
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);

    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true,is_a_new_node_);

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(node_replan == current_path_->getGoalNode())
      return false;

    connectBehindObs(node_replan);

    if(not current_path_->getTree()->changeRoot(root))
      throw std::runtime_error("root can not be changed");

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
      replanned_path_ = current_path_;

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
