#include "replanners_lib/replanners/LI_DRRT.h"

namespace pathplan
{
LazyInformedDRRT::LazyInformedDRRT(Eigen::VectorXd& current_configuration,
                                   PathPtr& current_path,
                                   const double& max_time,
                                   const TreeSolverPtr &solver):
  DynamicRRT(current_configuration,current_path,max_time,solver)
{
}
bool LazyInformedDRRT::trimInvalidTree(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  bool trimmed = false;
  TreePtr tree = current_path_->getTree();

  NodePtr child;
  unsigned int removed_nodes;      //will not be used;
  std::vector<NodePtr> white_list; //will not be used;

  //Firstly trim the tree starting from the path to node
  std::vector<ConnectionPtr> node2goal = tree->getConnectionToNode(node); //Note: the root must be the goal (set in regrowRRT())
  for(const ConnectionPtr &conn: node2goal)
  {
    if((ros::WallTime::now()-tic).toSec()>=max_time_)
    {
      if(verbose_)
        ROS_INFO("Time to trim expired");

      break;
    }

    assert(conn->isRecentlyChecked());

    if(conn->getCost() == std::numeric_limits<double>::infinity())
    {
      child = conn->getChild();
      tree->purgeFromHere(child,white_list,removed_nodes); //remove the successors and the connection from parent to child

      trimmed = true;
      break;
    }
  }

  trimmed_tree_ = tree;
  return trimmed;
}
bool LazyInformedDRRT::regrowRRT(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();
  trimmed_tree_ = current_path_->getTree();

  //Set the goal as the root
  if(not trimmed_tree_->changeRoot(goal_node_)) //revert the tree so the goal is the root
  {
    ROS_ERROR("The goal can't be set as root!");
    ROS_INFO_STREAM("Goal node: "<<goal_node_<<"\n"<<*goal_node_);
    ROS_INFO_STREAM("Current path end node: "<<current_path_->getGoalNode()<<"\n"<<*current_path_->getGoalNode());

    throw std::runtime_error("The goal can't be set as root!");
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
  assert(max_distance>0.0);

  sampler_ = std::make_shared<InformedSampler>(sampler_->getLB(),sampler_->getUB(),sampler_->getLB(),sampler_->getUB());

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && not success_)
  {
    NodePtr new_node;
    Eigen::VectorXd conf = sampler_->sample();
    if(trimmed_tree_->extend(conf,new_node))
    {
      assert(new_node->getParentConnectionsSize() == 1);
      new_node->getParentConnections().front()->setRecentlyChecked(true);
      checked_connections_.push_back(new_node->getParentConnections().front());

      if((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if(checker_->checkPath(new_node->getConfiguration(), node->getConfiguration()))
        {
          assert([&]() ->bool{
                   if(not (node->getParentConnectionsSize() == 0) && not (node->getChildConnectionsSize() == 0))
                   {
                     ROS_INFO_STREAM("node:\n"<<*node);
                     return false;
                   }
                   return true;
                 }());

          ConnectionPtr conn = std::make_shared<Connection>(new_node, node);
          conn->setCost(metrics_->cost(new_node, node));
          conn->add();

          conn->setRecentlyChecked(true);
          checked_connections_.push_back(conn);

          trimmed_tree_->addNode(node);

          bool free = true;
          NodePtr purge_from_here = nullptr;
          std::vector<ConnectionPtr> conn2goal = trimmed_tree_->getConnectionToNode(node);
          for(const ConnectionPtr& c:conn2goal)
          {
            if(c->isRecentlyChecked())
            {
              assert(c->getCost()<std::numeric_limits<double>::infinity());
              continue;
            }
            else
            {
              if(not checker_->checkConnection(c))
              {
                c->setCost(std::numeric_limits<double>::infinity());
                purge_from_here = c->getChild();
                free = false;
              }

              c->setRecentlyChecked(true);
              checked_connections_.push_back(c);

              if(not free)
                break;
            }
          }

          if(free)
          {
            //Set the root in the node and extract the new path
            trimmed_tree_->changeRoot(node);
            replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
            replanned_path_->setTree(trimmed_tree_);

            // SOLUZIONE MOMENTANEA
            for(unsigned int i=0;i<replanned_path_->getConnectionsSize();i++)
            {
              if(replanned_path_->getConnections().at(i)->norm() <1e-06)
              {
                if(replanned_path_->getConnections().at(i)->getParent()->getChildConnectionsSize() == 1)
                {
                  ConnectionPtr conn = std::make_shared<Connection>(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                  double cost = metrics_->cost(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                  conn->setCost(cost);
                  conn->add();

                  replanned_path_->getConnections().at(i)->remove();

                  replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
                  break;
                }
                else if(replanned_path_->getConnections().at(i)->getChild()->getChildConnectionsSize() == 1)
                {
                  ConnectionPtr conn = std::make_shared<Connection>(replanned_path_->getConnections().at(i)->getParent(),replanned_path_->getConnections().at(i+1)->getChild());
                  double cost = metrics_->cost(replanned_path_->getConnections().at(i)->getParent(),replanned_path_->getConnections().at(i+1)->getChild());
                  conn->setCost(cost);
                  conn->add();

                  replanned_path_->getConnections().at(i)->remove();

                  replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
                  break;
                }
              }
            }
            // FINO A QUA

            solver_->setStartTree(trimmed_tree_);
            solver_->setSolution(replanned_path_,true);

            tree_is_trimmed_ = false;

            success_ = true;
            break;
          }
          else
          {
            trimmed_tree_->purgeFromHere(purge_from_here);
          }
        }
      }
    }
    time = (ros::WallTime::now()-tic).toSec();
  }

  return success_;
}

bool LazyInformedDRRT::improvePath(const double& max_time)
{
  //root in current conf
  ros::WallTime tic = ros::WallTime::now();

  bool success = false;
  TreePtr tree = current_path_->getTree();
  double r_rewire = 2*tree->getMaximumDistance();
  sampler_ = std::make_shared<InformedSampler>(current_configuration_,goal_node_->getConfiguration(),sampler_->getLB(),sampler_->getUB(),cost_from_conf_);

  double new_cost;
  Eigen::VectorXd configuration;
  while((ros::WallTime::now()-tic).toSec()<max_time)
  {
    configuration = sampler_->sample();
    if(tree->rewireWithPathCheck(configuration,checked_connections_,r_rewire))
    {
      new_cost = tree->costToNode(goal_node_);
      if(new_cost<cost_from_conf_)
      {
        cost_from_conf_ = new_cost;
        replanned_path_ = std::make_shared<Path>(tree->getConnectionToNode(goal_node_), metrics_, checker_);
        replanned_path_->setTree(tree);

        sampler_->setCost(new_cost);
        success = true;
      }
    }
  }
  return success;
}

bool LazyInformedDRRT::replan()
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  cost_from_conf_ = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf_ == std::numeric_limits<double>::infinity())
  {
    if(verbose_)
      ROS_WARN("Current path obstructed");

    std::vector<NodePtr> path_nodes = current_path_->getNodes();  //save nodes pointers (the same pointers stored in the tree)
    assert(path_nodes.front() == current_path_->getTree()->getRoot());

    std::vector<double> connections_costs;
    for(const ConnectionPtr& conn:current_path_->getConnections())
      connections_costs.push_back(conn->getCost());

    assert([&]() ->bool{
             for(const NodePtr& n:path_nodes)
             {
               if(not current_path_->getTree()->isInTree(n))
               return false;
             }
             return true;
           }());

    NodePtr root = current_path_->getTree()->getRoot();
    assert(root == current_path_->getStartNode());

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    node_replan_ = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true,is_a_new_node_);

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan_);

    checked_connections_.clear();
    checked_connections_ = current_path_->getSubpathFromNode(node_replan_)->getConnections();
    std::for_each(checked_connections_.begin(),checked_connections_.end(),[&](ConnectionPtr c) {c->setRecentlyChecked(true);});

    if(not regrowRRT(node_replan_)) //root is goal
    {
      fixTree(node_replan_,root,path_nodes,connections_costs);
      replanned_path_ = current_path_;
      success_ = false;
    }
    else
    {
      double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
      current_path_ = replanned_path_;
      improvePath(max_time_impr);
      success_ = true;
    }

    std::for_each(checked_connections_.begin(),checked_connections_.end(),[&](ConnectionPtr c) {c->setRecentlyChecked(false);});
    checked_connections_.clear();
  }
  else //replan not needed
  {
    assert(current_path_->isValidFromConf(current_configuration_));

    double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
    if(improvePath(max_time_impr))
      success_ = true;
  }

  return success_;
}

}
