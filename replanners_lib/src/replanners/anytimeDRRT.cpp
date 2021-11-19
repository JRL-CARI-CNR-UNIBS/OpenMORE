#include "replanners_lib/replanners/anytimeDRRT.h"

namespace pathplan
{

AnytimeDynamicRRT::AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                                     PathPtr& current_path,
                                     const double& max_time,
                                     const TreeSolverPtr &solver): DynamicRRT(current_configuration,current_path,max_time,solver)
{
  goal_conf_ = current_path->getConnections().back()->getChild()->getConfiguration();

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

  if(!solver_->getStartTree() || !solver_->getSolution())
    assert(0);

  if(!tmp_solver->solved())
    assert(0);
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

  if(!forced_cast_solver->getStartTree() || !forced_cast_solver->getSolution())
    assert(0);

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

      solver_->setStartTree(solution->getTree());
      solver_->setSolution(solution,true);

      NodePtr initial_goal = replanned_path_->getNodes().back();
      ROS_INFO_STREAM("goal after improve: "<<initial_goal<<*initial_goal);

      if(!replanned_path_->getTree())
        assert(0);
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

  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);
  ROS_INFO("COST FROM CONF: %f",cost_from_conf);

  if(cost_from_conf == std::numeric_limits<double>::infinity() || tree_is_trimmed_)
  {
    NodePtr node_replan;

    if(!tree_is_trimmed_)
    {
      ConnectionPtr conn = current_path_->findConnection(current_configuration_);
      node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true);
    }
    else
    {
      node_replan = std::make_shared<Node>(current_configuration_);
    }
    ROS_INFO_STREAM("Starting node for replanning: \n"<< *node_replan);

    if(regrowRRT(node_replan))
    {
      ROS_WARN("REGROW");
      success_ = true;

      solver_->setStartTree(replanned_path_->getTree());
      solver_->setSolution(replanned_path_,true);

      double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
      if(improvePath(node_replan,max_time_impr)) //if not improved, success_ = true anyway beacuse a new path has been found with regrowRRT()
      {
        solver_->setStartTree(replanned_path_->getTree());
        solver_->setSolution(replanned_path_,true);       //should be after setStartTree

        for(const Eigen::VectorXd& wp:replanned_path_->getWaypoints())
          ROS_INFO_STREAM("wp improved: "<<wp.transpose());
      }
      ROS_INFO_STREAM("root after improve: "<<*replanned_path_->getTree()->getRoot());
    }
    else
    {
      success_ = false;
      ROS_ERROR("Tree can not be regrown using regrowRRT");

      if(!tree_is_trimmed_)
        assert(0);
    }
  }
  else //replan not needed
  {
    if(tree_is_trimmed_)
      assert(0);

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,false);

    solver_->setStartTree(current_path_->getTree());
    solver_->setSolution(current_path_,true);       //should be after setStartTree

    double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
    if(improvePath(node_replan,max_time_impr))
    {
      ROS_WARN("IMPROVED");

      solver_->setStartTree(replanned_path_->getTree());
      solver_->setSolution(replanned_path_,true);       //should be after setStartTree

      success_ = true;
    }
    else
      success_ = false;

    if(!success_)
      current_path_->removeNodes();
  }

  return success_;
}

}
