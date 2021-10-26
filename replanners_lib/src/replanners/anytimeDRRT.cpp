#include "replanners_lib/replanners/anytimeDRRT.h"

namespace pathplan
{

AnytimeDynamicRRT::AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                                     PathPtr& current_path,
                                     const double& max_time,
                                     const TreeSolverPtr &solver): DynamicRRT(current_configuration,current_path,max_time,solver)
{

  goal_conf_ = current_path->getConnections().back()->getChild()->getConfiguration();

  ROS_INFO("QUA1");

  const std::type_info& ti1 = typeid(AnytimeRRT);
  const std::type_info& ti2 = typeid(*solver);

  ROS_INFO("QUA2");

  AnytimeRRTPtr tmp_solver;
  if(std::type_index(ti1) != std::type_index(ti2))
  {
    ROS_INFO("QUA3");

    tmp_solver = std::make_shared<pathplan::AnytimeRRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
    tmp_solver->importFromSolver(solver); //copy the required fields

    ROS_INFO("QUA4");

  }
  else
  {

    ROS_INFO("QUA5");

    tmp_solver = std::static_pointer_cast<AnytimeRRT>(solver);

    const std::type_info& ti3 = typeid(*tmp_solver);
    ROS_INFO_STREAM("type tmp solver "<<ti3.name());
    ROS_INFO_STREAM("type solver "<<ti2.name());
    ROS_INFO("QUA5.5");

//    ROS_INFO_STREAM("goal conf: "<<tmp_solver->getGoal()->getConfiguration());
  }

  solver_ = tmp_solver;

  ROS_INFO("QUA6");

  if(!tmp_solver->solved())
    assert(0);

  ROS_INFO("QUA7");

//  if(goal_conf_ != tmp_solver->getGoal()->getConfiguration())
//    assert(0);

//    ROS_INFO("QUA8");
}
void AnytimeDynamicRRT::updatePath(NodePtr& node)
{
  TreePtr tree = solver_->getStartTree();
  NodePtr goal = solver_->getSolution()->getConnections().back()->getChild();
  if(tree->getRoot() != node)
    assert(0);

  tree->changeRoot(node);
  PathPtr updated_path = std::make_shared<Path>(tree->getConnectionToNode(goal),metrics_,checker_);
  solver_->setSolution(updated_path,true);
}
bool AnytimeDynamicRRT::improvePath(NodePtr &node, const double& max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  bool success = false;

  const std::type_info& ti1 = typeid(AnytimeRRT);
  const std::type_info& ti2 = typeid(*solver_);

  if(ti1 != ti2)
  {
    ROS_ERROR_STREAM("Solver not of type AnytimeRRT");
    return false;
  }

  AnytimeRRTPtr forced_cast_solver = std::static_pointer_cast<AnytimeRRT>(solver_);

  double path_cost = solver_->getSolution()->getCostFromConf(node->getConfiguration());
  double imprv = forced_cast_solver->getCostImpr();

  int n_fail = 0;
  PathPtr solution;
  double cost2beat;
  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time && n_fail<FAILED_ITER)
  {
    cost2beat = (1-imprv)*path_cost;
    ROS_INFO_STREAM("path cost: "<<path_cost<<" cost2beat: "<<cost2beat);

    NodePtr start_node = std::make_shared<Node>(node->getConfiguration());
    NodePtr goal_node  = std::make_shared<Node>(goal_conf_);

    bool improved = forced_cast_solver->improve(start_node,goal_node,solution,cost2beat,1000,(max_time-time));

    if(improved)
    {
      replanned_path_ = solution;
      path_cost = solution->cost();
      success = true;
      n_fail = 0;
    }
    else
      n_fail +=1;

    time = (ros::WallTime::now()-tic).toSec();
  }

  return success;
}

bool AnytimeDynamicRRT::replan()
{
  ros::WallTime tic = ros::WallTime::now();

  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(regrowRRT(node_replan))
    {
      //      updatePath(node_replan);

      double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
      improvePath(node_replan,max_time_impr); //if not improved, success_ = true anyway beacuse a new path has been found with regrowRRT()
    }
  }
  else //replan not needed
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);

    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    //    updatePath(node_replan);

    double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
    if(improvePath(node_replan,max_time_impr))
      success_ = true;
    else
      success_ = false;
  }

  return success_;
}

}
