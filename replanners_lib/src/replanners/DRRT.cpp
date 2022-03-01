#include "replanners_lib/replanners/DRRT.h"

namespace pathplan
{

bool checkTreeNodes(const TreePtr& tree) //ELIMINA
{
  std::vector<NodePtr> nodes = tree->getNodes();
  NodePtr root = tree->getRoot();

  for(const NodePtr& n:nodes)
  {
    if(n == root)
      continue;

    if(n->parent_connections_.size() != 1)
      assert(0);
  }
  return true;
}

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
  sampler_ =  std::make_shared<InformedSampler>(lb_,ub_,lb_,ub_);
  tree_is_trimmed_ = false;
}

void DynamicRRT::fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr>& old_nodes, std::vector<double>& old_connections_costs)
{
  if(success_)
    return;

  if(tree_is_trimmed_) //rebuild the tree adding the old path as a branch
  {
    std::reverse(old_nodes.begin(),old_nodes.end());  //so the node are ordered from the goal to the start
    std::reverse(old_connections_costs.begin(),old_connections_costs.end());

    assert(not trimmed_tree_->isInTree(node_replan));
    assert(old_nodes.front() == goal_node_);

    std::vector<ConnectionPtr> restore_old_path;
    for(unsigned int i=1; i<old_nodes.size();i++) //the goal (i = 0) is in the tree (it is the root)
    {
      if(trimmed_tree_->isInTree(old_nodes.at(i)))
        continue;
      else
      {
        for(unsigned int j=i;j<old_nodes.size();j++)
        {
          ConnectionPtr conn = std::make_shared<Connection>(old_nodes.at(j-1),old_nodes.at(j));
          conn->setCost(old_connections_costs.at(j-1));
          conn->add();

          restore_old_path.push_back(conn);
        }
        break;
      }
    }

    trimmed_tree_->addBranch(restore_old_path);
    tree_is_trimmed_ = false;

    trimmed_tree_->changeRoot(root); //the old root (the starting point of the current path)
    current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));

    if(std::find(old_nodes.begin(),old_nodes.end(),node_replan) == old_nodes.end())
      assert(not trimmed_tree_->isInTree(node_replan));
  }
  else
  {
    assert(trimmed_tree_->isInTree(node_replan));

    if(std::find(old_nodes.begin(),old_nodes.end(),node_replan) == old_nodes.end()) //if the node_replan has been added to the path and tree, remove it. If it was already present, do not remove it
    {
      assert(node_replan->parent_connections_.size() == 1 && node_replan->child_connections_.size() == 1);

      NodePtr parent = node_replan->getParents().front();
      NodePtr child = node_replan->getChildren().front();

      ConnectionPtr conn = std::make_shared<Connection>(parent,child);
      double cost = node_replan->parent_connections_.front()->getCost()+child->parent_connections_.front()->getCost();
      conn->setCost(cost);
      conn->add();

      trimmed_tree_->removeNode(node_replan);
      assert(not trimmed_tree_->isInTree(node_replan));
    }
    trimmed_tree_->changeRoot(root);
    current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));
  }

  assert(trimmed_tree_->getRoot() == root);
}

bool DynamicRRT::trimInvalidTree(NodePtr& node, std::vector<ConnectionPtr>& checked_connections)
{
  checkTreeNodes(current_path_->getTree()); //elimina

  ros::WallTime tic = ros::WallTime::now();

  bool trimmed = false;
  TreePtr tree= current_path_->getTree();

  NodePtr child;
  unsigned int removed_nodes;      //not needed
  std::vector<NodePtr> white_list; //not needed

  //Firstly trim the tree starting from the path to node
  std::vector<ConnectionPtr> node2goal = tree->getConnectionToNode(node); //Note: the root must be the goal (set in regrowRRT())
  for(const ConnectionPtr &conn: node2goal)
  {
    if((ros::WallTime::now()-tic).toSec()>=max_time_)
    {
      ROS_INFO("Time to trim expired");
      break;
    }

    bool obstructed = false;
    if(conn->getCost() == std::numeric_limits<double>::infinity())
      obstructed = true;
    else
    {
      if(std::find(checked_connections.begin(),checked_connections.end(),conn) != checked_connections.end())
      {
        if(not checker_->checkConnection(conn))
        {
          conn->setCost(std::numeric_limits<double>::infinity());
          obstructed = true;
        }

        checked_connections.push_back(conn);
      }
    }

    if(obstructed)
    {
      child = conn->getChild();
      tree->purgeFromHere(child,white_list,removed_nodes); //remove the successors and the connection from parent to child

      trimmed = true;
      break;
    }
  }

  checkTreeNodes(current_path_->getTree()); //elimina

  //Now check the remaining tree

  int iter = 0; //elimina

  bool trimmed_again;
  do
  {
    trimmed_again = false;

    std::vector<NodePtr> leaves;
    std::vector<ConnectionPtr> path_connections;

    for(const NodePtr& n:tree->getNodes())
    {
      assert(((n->parent_connections_.size() == 1) && (n!=tree->getRoot())) || (n == tree->getRoot()));
      if(n->child_connections_.empty())
        leaves.push_back(n);
    }

    for(const NodePtr& leave:leaves)
    {
      if(not tree->checkPathToNode(leave,checked_connections,path_connections))
      {
        for(const ConnectionPtr& conn:path_connections)
        {
          if(conn->getCost() == std::numeric_limits<double>::infinity())
          {
            child = conn->getChild();
            tree->purgeFromHere(child,white_list,removed_nodes);

            trimmed = true;
            trimmed_again = true;

            break;
          }
        }
        if(trimmed_again)
          break;
      }
    }

    ROS_INFO_STREAM("Iter trim: "<<iter);
    checkTreeNodes(current_path_->getTree()); //elimina

    iter++;

  }while(trimmed_again);

  trimmed_tree_ = tree;
  return trimmed;
}

bool DynamicRRT::regrowRRT(NodePtr& node)
{
  ros::WallTime tic = ros::WallTime::now();

  std::vector<ConnectionPtr> checked_connections = current_path_->getSubpathFromNode(node)->getConnections();

  //Set the goal as the root
  if(not current_path_->getTree()->changeRoot(goal_node_)) //revert the tree so the goal is the root
  {
    ROS_ERROR("The goal can't be set as root!");
    ROS_INFO_STREAM("Goal node: "<<goal_node_<<"\n"<<*goal_node_);
    ROS_INFO_STREAM("Current path end node: "<<current_path_->getConnections().back()->getChild()<<"\n"<<*current_path_->getConnections().back()->getChild());

    assert(0);
  }

  checkTreeNodes(current_path_->getTree()); //elimina

  if(not tree_is_trimmed_)
  {
    //Trim the tree
    if(not trimInvalidTree(node,checked_connections))
    {
      if(verbose_)
        ROS_INFO("Tree not trimmed");

      tree_is_trimmed_ = false;
      return false;
    }
    else
      tree_is_trimmed_ = true;
  }
  double perc = ((ros::WallTime::now()-tic).toSec()/max_time_)*100;
  ROS_WARN_STREAM("tempo impiegato "<<perc<< "%"); //ELIMINA

  checkTreeNodes(current_path_->getTree()); //elimina

  //Regrow the tree
  double max_distance = trimmed_tree_->getMaximumDistance();
  assert(max_distance>0.0);

  //InformedSampler sampler(lb_,ub_,lb_,ub_);

  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time_ && not success_)
  {
    NodePtr new_node;
    Eigen::VectorXd conf = sampler_->sample(); //CHIEDI A MANUEL PERCHE CAPITA CHE VENGA CAMPIONATA LA STESSA CONFIGURAZIONE PIU VOLTE
    //if(trimmed_tree_->extendWithPathCheck(conf,new_node,checked_connections))
    if(trimmed_tree_->extend(conf,new_node))
    {
      checkTreeNodes(current_path_->getTree()); //elimina

      ROS_INFO_STREAM("new_node: "<<new_node->getConfiguration().transpose()<<" conf: "<<conf.transpose()); //ELIMINA
      if((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if(checker_->checkPath(new_node->getConfiguration(), node->getConfiguration()))
        {
          if(not node->parent_connections_.empty() && not node->child_connections_.empty())
          {
            ROS_INFO_STREAM("node:\n"<<*node);
            assert(0);
          }

          ConnectionPtr conn = std::make_shared<Connection>(new_node, node);
          conn->setCost(metrics_->cost(new_node, node));
          conn->add();

          checked_connections.push_back(conn);

          trimmed_tree_->addNode(node);

          checkTreeNodes(current_path_->getTree()); //elimina

          //Set the root in the node and extract the new path
          trimmed_tree_->changeRoot(node);
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
          replanned_path_->setTree(trimmed_tree_);

          // SOLUZIONE MOMENTANEA
          for(unsigned int i=0;i<replanned_path_->getConnections().size();i++)
          {
            if(replanned_path_->getConnections().at(i)->norm() <1e-06)
            {
              if(replanned_path_->getConnections().at(i)->getParent()->child_connections_.size() == 1)
              {
                ConnectionPtr conn = std::make_shared<Connection>(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                double cost = metrics_->cost(replanned_path_->getConnections().at(i-1)->getParent(),replanned_path_->getConnections().at(i)->getChild());
                conn->setCost(cost);
                conn->add();

                replanned_path_->getConnections().at(i)->remove();

                replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
                break;
              }
              else if(replanned_path_->getConnections().at(i)->getChild()->child_connections_.size() == 1)
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

          checkTreeNodes(current_path_->getTree()); //elimina

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
  success_ = false;
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    if(verbose_)
      ROS_WARN("Current path obstructed");

    std::vector<NodePtr> path_nodes = current_path_->getNodes();  //save nodes pointers (the same pointers stored in the tree)
    assert(path_nodes.front() == current_path_->getTree()->getRoot());

    std::vector<double> connections_costs;
    for(const ConnectionPtr& conn:current_path_->getConnections())
      connections_costs.push_back(conn->getCost());

    for(const NodePtr& n:path_nodes)
      assert(current_path_->getTree()->isInTree(n));

    NodePtr root = current_path_->getTree()->getRoot();
    assert(root == current_path_->getConnections().front()->getParent());

    checkTreeNodes(current_path_->getTree()); //elimina

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    checkTreeNodes(current_path_->getTree()); //elimina

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(not regrowRRT(node_replan)) //root is goal
      fixTree(node_replan,root,path_nodes,connections_costs);

    checkTreeNodes(current_path_->getTree()); //elimina
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
