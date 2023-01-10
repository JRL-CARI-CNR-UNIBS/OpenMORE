#include "replanners_lib/trajectory.h"

namespace pathplan
{
Trajectory::Trajectory(const PathPtr path,
                       const ros::NodeHandle &nh,
                       const planning_scene::PlanningScenePtr &planning_scene,
                       const std::string &group_name)
{
  trj_ = nullptr;
  path_ = path;
  nh_ = nh;
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
  moveit_utils_ = std::make_shared<MoveitUtils>(planning_scene,group_name);
}

Trajectory::Trajectory(const ros::NodeHandle &nh,
                       const planning_scene::PlanningScenePtr &planning_scene,
                       const std::string &group_name)
{
  trj_ = nullptr;
  path_ = nullptr;
  nh_ = nh;
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
  moveit_utils_ = std::make_shared<MoveitUtils>(planning_scene,group_name);
}

PathPtr Trajectory::computePath(const Eigen::VectorXd& start_conf, const Eigen::VectorXd& goal_conf, const TreeSolverPtr& solver, const bool& optimizePath, const double &max_time)
{
  NodePtr start_node = std::make_shared<Node>(start_conf);
  NodePtr goal_node = std::make_shared<Node>(goal_conf);

  return computePath(start_node,goal_node,solver,optimizePath,max_time);
}


PathPtr Trajectory::computePath(const NodePtr& start_node, const NodePtr& goal_node, const TreeSolverPtr& solver, const bool& optimize, const double &max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  CollisionCheckerPtr checker = solver->getChecker();
  SamplerPtr sampler = solver->getSampler();
  MetricsPtr metrics = solver->getMetrics();
  Eigen::VectorXd lb = sampler->getLB();
  Eigen::VectorXd ub = sampler->getUB();

  pathplan::PathPtr solution;
  bool success = solver->computePath(start_node,goal_node,nh_,solution,max_time,1000000);

  if(not success)
  {
    ROS_INFO("No solutions found");
    return nullptr;
  }

  double utopia = (start_node->getConfiguration()-goal_node->getConfiguration()).norm();

  if(optimize && success && (solution->cost()>1.05*utopia))
  {
    pathplan::PathLocalOptimizer path_solver(checker, metrics);
    path_solver.config(nh_);

    solution->setTree(solver->getStartTree());
    path_solver.setPath(solution);
    path_solver.solve(solution);

    sampler->setCost(solution->cost());
    solver->getStartTree()->addBranch(solution->getConnections());

    pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);

    for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
    {
      pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
      local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
    }
    local_sampler->setCost(solution->cost());

    pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
    opt_solver.importFromSolver(solver);

    std::vector<pathplan::NodePtr> white_list;
    white_list.push_back(goal_node);

    int stall_gen = 0;
    int max_stall_gen = 200;

    std::mt19937 gen;
    std::uniform_int_distribution<> id = std::uniform_int_distribution<>(0, max_stall_gen);

    for (unsigned int idx = 0; idx < 10000; idx++)
    {
      if ((ros::WallTime::now() - tic).toSec() > max_time)
        break;

      if(opt_solver.update(solution)) //rewire
      {
        stall_gen = 0;
        path_solver.setPath(solution);
        solution->setTree(opt_solver.getStartTree());

        local_sampler->setCost(solution->cost());
        sampler->setCost(solution->cost());
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

        local_sampler->clearBalls();
        for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
        {
          pathplan::ConnectionPtr conn = solution->getConnections().at(isol);

          local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
        }
      }
      else
      {
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);
        stall_gen++;
      }

      if (idx % 10 == 0)
      {
        if(id(gen) < stall_gen)
        {
          opt_solver.setSampler(sampler);
        }
        else
        {
          opt_solver.setSampler(local_sampler);
        }
      }

      if (stall_gen >= max_stall_gen)
        break;
    }

    path_solver.setPath(solution);
    path_solver.solve(solution);
  }

  return solution;
}

robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint &pnt)
{
  trajectory_msgs::JointTrajectoryPoint::Ptr pnt_ptr(new trajectory_msgs::JointTrajectoryPoint());

  pnt_ptr->positions       = pnt.positions      ;
  pnt_ptr->velocities      = pnt.velocities     ;
  pnt_ptr->accelerations   = pnt.accelerations  ;
  pnt_ptr->effort          = pnt.effort         ;
  pnt_ptr->time_from_start = pnt.time_from_start;

  return Trajectory::fromPath2Trj(pnt_ptr);
}


robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPointPtr &pnt)
{
  if(not path_)
    throw std::invalid_argument("Path not assigned");

  std::vector<Eigen::VectorXd> waypoints=path_->getWaypoints();
  std::vector<moveit::core::RobotState> wp_state_vector = moveit_utils_->fromWaypoints2State(waypoints);

  trj_ = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);
  for(unsigned int j=0; j<waypoints.size();j++)
  {
    if(j==0 && pnt != nullptr)
    {
      wp_state_vector.at(j).setJointGroupPositions    (group_name_,pnt->positions    );
      wp_state_vector.at(j).setJointGroupVelocities   (group_name_,pnt->velocities   );
      wp_state_vector.at(j).setJointGroupAccelerations(group_name_,pnt->accelerations);
    }
    trj_->addSuffixWayPoint(wp_state_vector.at(j),0.001);
  }

  //Time parametrization
  //trajectory_processing::IterativeSplineParameterization iptp;
  //trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  iptp.computeTimeStamps(*trj_);
  return trj_;
}

double Trajectory::getTimeFromTrjPoint(const Eigen::VectorXd &trj_point, const int& n_interval, const int &spline_order)
{
  double t = -1.0;
  if(trj_ == nullptr)
  {
    ROS_ERROR("Trj not computed");
    throw std::invalid_argument("trj not computed");
  }
  else
  {
    Eigen::VectorXd pos1,pos1_interval;
    double err,dist,dist1,dist2,dist1_interval,t1_interval,t2_interval;
    double end_time = trj_->getDuration();
    double step = end_time/n_interval;
    double min_err = std::numeric_limits<double>::infinity();
    double t1 = 0;
    double t2 = t1+step;

    moveit_msgs::RobotTrajectory tmp_trj_msg;
    trj_->getRobotTrajectoryMsg(tmp_trj_msg);

    trajectory_processing::SplineInterpolator interpolator;
    interpolator.setTrajectory(tmp_trj_msg);
    interpolator.setSplineOrder(spline_order);

    pos1 = path_->getWaypoints().at(0);
    dist1 = (trj_point-pos1).norm();

    for(unsigned int i=0;i<2;i++)
    {
      while(t2<=end_time)
      {
        trajectory_msgs::JointTrajectoryPoint pnt2;
        interpolator.interpolate(ros::Duration(t2),pnt2);
        Eigen::VectorXd pos2(pnt2.positions.size());
        for(unsigned int j=0; j<pnt2.positions.size();j++) pos2[j] = pnt2.positions.at(j);

        dist2 = (pos2-trj_point).norm();
        dist = (pos2-pos1).norm();
        err = std::abs(dist-dist1-dist2);

        if(err<min_err)
        {
          min_err = err;
          t1_interval = t1;
          t2_interval = t2;
          dist1_interval = dist1;
          pos1_interval = pos1;

          t = t1+step*(dist1)/(dist1+dist2);

          if(err<1e-06) return t;
        }

        t1 = t2;
        t2 = t1+step;
        dist1 = dist2;
        pos1 = pos2;
      }

      step = (t2_interval-t1_interval)/100;
      t1 = t1_interval;
      t2 = t1+step;
      end_time = t2_interval;
      dist1 = dist1_interval;
      pos1 = pos1_interval;
    }
  }

  return t;
}
}
