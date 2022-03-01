#include "replanners_lib/replanners/anytimeDRRT.h"

namespace pathplan
{

AnytimeDynamicRRT::AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                                     PathPtr& current_path,
                                     const double& max_time,
                                     const TreeSolverPtr &solver): DynamicRRT(current_configuration,current_path,max_time,solver)
{
  const std::type_info& ti1 = typeid(AnytimeRRT);
  const std::type_info& ti2 = typeid(*solver);

  AnytimeRRTPtr tmp_solver;
  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<pathplan::AnytimeRRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
    tmp_solver = std::static_pointer_cast<AnytimeRRT>(solver);

  solver_ = tmp_solver;

  assert(tmp_solver->solved());
  assert(solver_->getSolution());
  assert(solver_->getStartTree());
}

bool AnytimeDynamicRRT::improvePath(NodePtr &node, const double& max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  bool success = false;

  AnytimeRRTPtr forced_cast_solver = std::static_pointer_cast<AnytimeRRT>(solver_);  //solver_ is of type AnytimeRRT (see constructor)

  if(replanned_path_)
  {
    if(current_path_->getTree() != replanned_path_->getTree())
    {
      ROS_INFO_STREAM("Current path tree, replanned path tree: "<<current_path_->getTree()<<" "<<replanned_path_->getTree());
      assert(0);
    }
  }

  if(current_path_->getTree() != solver_->getStartTree())
  {
    ROS_INFO_STREAM("Current path tree, solver tree: "<<current_path_->getTree()<<" "<<solver_->getStartTree());
    assert(0);
  }

  assert(forced_cast_solver->getStartTree());
  assert(forced_cast_solver->getSolution());

  double imprv = forced_cast_solver->getCostImpr();
  double path_cost = solver_->getSolution()->getCostFromConf(node->getConfiguration());
  forced_cast_solver->setPathCost(path_cost);

  int n_fail = 0;
  PathPtr solution;
  double cost2beat;
  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time && n_fail<FAILED_ITER)
  {
    cost2beat = (1-imprv)*path_cost;

    if(verbose_)
      ROS_INFO_STREAM("Path cost: "<<path_cost<<", cost2beat: "<<cost2beat);

    NodePtr start_node = std::make_shared<Node>(node->getConfiguration());
    NodePtr goal_node  = std::make_shared<Node>(goal_node_->getConfiguration());

    bool improved = forced_cast_solver->improve(start_node,goal_node,solution,cost2beat,10000,(max_time-time));

    if(improved)
    {
      replanned_path_ = solution;
      goal_node_ = goal_node;
      success = true;
      n_fail = 0;

      assert(replanned_path_->getConnections().back()->getChild()->getConfiguration() == goal_node->getConfiguration());

      solver_->setStartTree(solution->getTree());
      solver_->setSolution(solution,true);

      if(verbose_)
        ROS_INFO_STREAM("Improved cost: "<<solution->cost());

      assert(replanned_path_->getTree());
    }
    else
    {
      n_fail +=1;

      if(verbose_)
        ROS_INFO("Not improved");
    }

    time = (ros::WallTime::now()-tic).toSec();
  }

  return success;
}

bool AnytimeDynamicRRT::replan()
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  NodePtr node_replan;
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
    assert(root == path_nodes.front());

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    if(verbose_)
      ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(regrowRRT(node_replan))
    {
      success_ = true;

      current_path_ = replanned_path_;  //try to improve the path replanned with regrowRRT()

      solver_->setStartTree(current_path_->getTree());
      solver_->setSolution(current_path_,true);

      double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
      if(improvePath(node_replan,max_time_impr)) //if not improved, success_ = true anyway beacuse a new path has been found with regrowRRT()
      {
        solver_->setStartTree(replanned_path_->getTree());
        solver_->setSolution(replanned_path_,true);       //should be after setStartTree
      }
    }
    else
    {
      fixTree(node_replan,root,path_nodes,connections_costs);

      success_ = false;
      if(verbose_)
        ROS_ERROR("Tree can not be regrown using regrowRRT");
    }
  }
  else
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,false);

    solver_->setStartTree(current_path_->getTree());
    solver_->setSolution(current_path_,true);       //should be after setStartTree

    double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
    if(improvePath(node_replan,max_time_impr))
    {
      solver_->setStartTree(replanned_path_->getTree());
      solver_->setSolution(replanned_path_,true);   //should be after setStartTree

      success_ = true;
    }
    else
      success_ = false;
  }

  return success_;
}
}
