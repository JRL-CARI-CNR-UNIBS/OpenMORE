#include "replanners_lib/replanners/MPRRT.h"

namespace pathplan
{

MPRRT::MPRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr& solver,
             const unsigned int& number_of_parallel_plannings): ReplannerBase(current_configuration,current_path,max_time,solver)
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

  if(number_of_parallel_plannings<1)
    number_of_parallel_plannings_ = 1;
  else
    number_of_parallel_plannings_ =  number_of_parallel_plannings;

  solver_vector_.clear();
  for(unsigned int i=0;i<number_of_parallel_plannings_;i++)
  {
    SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb_,ub_,lb_,ub_);
    MetricsPtr metrics = metrics_->clone();

    CollisionCheckerPtr checker = checker_->clone();
    RRTPtr sv = std::static_pointer_cast<RRT>(solver_->clone(metrics, checker, sampler));
    sv->importFromSolver(solver_);

    solver_vector_.push_back(sv);
  }

  connecting_path_vector_.resize(number_of_parallel_plannings_,nullptr);
}

bool MPRRT::asyncComputeConnectingPath(const Eigen::VectorXd path1_node_conf,
                                       const Eigen::VectorXd path2_node_conf,
                                       const double current_solution_cost,
                                       const int index)
{
  ros::WallTime tic = ros::WallTime::now();

  TreeSolverPtr solver = solver_vector_.at(index);

  double best_cost = std::numeric_limits<double>::infinity();
  PathPtr best_solution = nullptr;
  bool success = false;
  int iter = 0;
  double time;

  do
  {
    iter++;

    NodePtr path1_node = std::make_shared<Node>(path1_node_conf);
    NodePtr path2_node = std::make_shared<Node>(path2_node_conf);

    PathPtr connecting_path = nullptr;
    bool directly_connected = false;

    time = max_time_-(ros::WallTime::now()-tic).toSec();

    bool solved = computeConnectingPath(path1_node,path2_node,current_solution_cost,time,
                                        connecting_path,directly_connected,solver);

    if(solved)
    {
      success = true;

      double new_cost = connecting_path->cost();
      if(new_cost<best_cost)
      {
        best_solution = connecting_path;
        best_cost = new_cost;
      }
    }
  } while((0.98*max_time_-(ros::WallTime::now()-tic).toSec())>0.0 && ros::ok());

  mtx_.lock();
  connecting_path_vector_.at(index) = best_solution;
  mtx_.unlock();

  double cost;
  success?
        (cost = best_solution->cost()):
        (cost = -1);

  if(verbose_)
    ROS_INFO_STREAM("\n--- THREAD REASUME ---\nthread n: "<<index<<"\nsuccess: "<<success<<"\nconnecting path cost: "<< cost<<"\nn iter: "<<iter<<" time: "<<(ros::WallTime::now()-tic).toSec());

  return success;
}

bool MPRRT::connect2goal(const NodePtr& node)
{
  success_ = false;
  bool solved = false;
  std::vector<std::shared_future<bool>> futures;

  double current_cost = current_path_->getCostFromConf(node->getConfiguration());

  if(current_cost <= 1.05*(node->getConfiguration()-goal_node_->getConfiguration()).norm())
  {
    success_ = false;
    return success_;
  }

  if(verbose_)
  {
    if(current_cost == std::numeric_limits<double>::infinity())
      ROS_WARN("COST INF");
  }

  for(unsigned int i=0; i<number_of_parallel_plannings_;i++)
  {
    int index = i;
    futures.push_back(std::async(std::launch::async,
                                 &MPRRT::asyncComputeConnectingPath,
                                 this,node->getConfiguration(),
                                 goal_node_->getConfiguration(),current_cost,index));
  }

  std::vector<double> marker_color;
  marker_color = {1.0,1.0,0.0,1.0};

  unsigned int idx_best_sol = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  for(unsigned int i=0; i<number_of_parallel_plannings_;i++)
  {
    if(futures.at(i).get())
    {
      assert(connecting_path_vector_.at(i));

      solved = true;
      double i_cost = connecting_path_vector_.at(i)->cost();

      if(i_cost<best_cost)
      {
        best_cost = i_cost;
        idx_best_sol = i;

        if(verbose_)
          ROS_INFO_STREAM("New cost: "<<best_cost);
      }

      if(verbose_ && disp_)
        disp_->displayPath(connecting_path_vector_.at(i),"pathplan",marker_color);
    }
  }

  if(solved)
  {
    std::vector<ConnectionPtr>  connecting_path_conn = connecting_path_vector_.at(idx_best_sol)->getConnections();
    PathPtr new_path = concatWithNewPathToGoal(connecting_path_conn, node);
    replanned_path_ = new_path;
    double replanned_path_cost = replanned_path_->cost();

    assert(current_path_->findConnection(replanned_path_->getConnections().front()->getParent()->getConfiguration()) != nullptr);

    if(replanned_path_cost<current_cost)
    {
      success_ = true;
      if(verbose_)
        ROS_INFO_STREAM("Solution found! -> cost: " << replanned_path_cost);
    }
    else
    {
      success_ = false;
      if(verbose_)
        ROS_INFO_STREAM("Solution found but cost ("<<replanned_path_cost<<") is not better than the current cost (" << current_cost<<")");
    }
  }
  else
  {
    if(verbose_)
      ROS_ERROR("New path not found!");
  }

  return success_;
}

PathPtr MPRRT::concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn,
                                       const NodePtr& path1_node)
{
  std::vector<ConnectionPtr> new_connecting_path_conn;
  NodePtr path2_node = std::make_shared<Node>(current_path_->getWaypoints().back());

  if(connecting_path_conn.size()>1)
  {
    NodePtr node1 = connecting_path_conn.front()->getChild();
    NodePtr node2 = connecting_path_conn.back()->getParent();

    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1,false);
    ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node,false);

    conn1->setCost(connecting_path_conn.front()->getCost());
    conn2->setCost(connecting_path_conn.back()->getCost());

    conn1->add();
    conn2->add();

    new_connecting_path_conn.push_back(conn1);

    if(connecting_path_conn.size()>2)
      new_connecting_path_conn.insert(new_connecting_path_conn.end(), connecting_path_conn.begin()+1, connecting_path_conn.end()-1);

    new_connecting_path_conn.push_back(conn2);

    connecting_path_conn.front()->remove();
    connecting_path_conn.back()->remove();
  }
  else
  {
    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node,false);
    conn1->setCost(connecting_path_conn.front()->getCost());
    conn1->add();

    new_connecting_path_conn.push_back(conn1);
    connecting_path_conn.front()->remove();
  }

  return std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
}

bool MPRRT::computeConnectingPath(const NodePtr &path1_node_fake,
                                  const NodePtr &path2_node_fake,
                                  const double &current_solution_cost,
                                  const double max_time,
                                  PathPtr &connecting_path,
                                  bool &directly_connected,
                                  TreeSolverPtr& solver)
{
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,current_solution_cost);

  solver->setSampler(sampler);
  solver->resetProblem();
  solver->addStart(path1_node_fake);

  ros::WallTime tic_solver = ros::WallTime::now();
  solver->addGoal(path2_node_fake,max_time);
  ros::WallTime toc_solver = ros::WallTime::now();

  directly_connected = solver->solved();
  bool solver_has_solved;

  if(directly_connected)
  {
    connecting_path = solver->getSolution();
    solver_has_solved = true;
  }
  else
  {
    double solver_time = max_time-(toc_solver-tic_solver).toSec();
    solver_has_solved = solver->solve(connecting_path,10000,solver_time);
  }

  return solver_has_solved;
}

bool MPRRT::replan()
{
  //Update the scene for all the planning threads
  moveit_msgs::PlanningScene scene_msg;
  checker_->getPlanningScene()->getPlanningSceneMsg(scene_msg);

  for(const RRTPtr& solver:solver_vector_)
    solver->getChecker()->setPlanningSceneMsg(scene_msg);

  //Replan
  ConnectionPtr conn = current_path_->findConnection(current_configuration_);
  NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,false);

  assert(current_path_->findConnection(node_replan->getConfiguration()) != nullptr);

  success_ = connect2goal(node_replan);

  if(not success_)
    replanned_path_ = current_path_;

  return success_;
}

}
