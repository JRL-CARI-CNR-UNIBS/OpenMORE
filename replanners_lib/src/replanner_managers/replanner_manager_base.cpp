#include "replanners_lib/replanner_managers/replanner_manager_base.h"

namespace pathplan
{
ReplannerManagerBase::ReplannerManagerBase(const PathPtr &current_path,
                                           const TreeSolverPtr &solver,
                                           const ros::NodeHandle &nh)
{
  current_path_replanning_ = current_path;
  solver_       = solver;
  nh_           = nh    ;

  fromParam();
  subscribeTopicsAndServices();
}

ReplannerManagerBase::~ReplannerManagerBase()
{
}

void ReplannerManagerBase::fromParam()
{
  if(!nh_.getParam("trj_execution_thread_frequency",trj_exec_thread_frequency_))
  {
    ROS_ERROR("trj_execution_thread_frequency not set, set 500");
    trj_exec_thread_frequency_ = 500;
  }
  if(!nh_.getParam("collision_checker_thread_frequency",collision_checker_thread_frequency_))
  {
    ROS_ERROR("collision_checker_thread_frequency not set, set 30");
    collision_checker_thread_frequency_ = 30;
  }
  if(!nh_.getParam("dt_replan",dt_replan_))
  {
    ROS_ERROR("dt_replan not set, set 0.100");
    dt_replan_ = 0.100;
  }
  if(!nh_.getParam("checker_resolution",checker_resolution_))
  {
    ROS_ERROR("checker_resolution not set, set 0.05");
    checker_resolution_ = 0.05;
  }
  if(!nh_.getParam("parallel_checker_n_threads",parallel_checker_n_threads_))
  {
    ROS_ERROR("parallel_checker_n_threads not set, set 10");
    parallel_checker_n_threads_ = 10;
  }
  if(!nh_.getParam("read_safe_scaling",read_safe_scaling_))
  {
    ROS_ERROR("read_safe_scaling not set, set false");
    read_safe_scaling_ = false;
  }

  if(read_safe_scaling_)
  {
    if(!nh_.getParam("overrides",scaling_topics_names_))
    {
      ROS_ERROR("overrides not set, used /speed_ovr, /safe_ovr_1, /safe_ovr_2");

      scaling_topics_names_.clear();

      scaling_topics_names_.push_back("/speed_ovr" );
      scaling_topics_names_.push_back("/safe_ovr_1");
      scaling_topics_names_.push_back("/safe_ovr_2");
    }
  }

  if(!nh_.getParam("goal_tol",goal_tol_))
    goal_tol_ = 1.0e-06;
  else
  {
    if(goal_tol_<TOLERANCE)
    {
      goal_tol_ = TOLERANCE;
      ROS_WARN("goal_tol set equal to TOLERANCE (%f), it can't be less than that value", TOLERANCE);
    }
  }

  if(!nh_.getParam("group_name",group_name_))
    ROS_ERROR("group_name not set, maybe set later with setGroupName(..)?");
  if(!nh_.getParam("scaling",scaling_from_param_))
    scaling_from_param_ = 1.0;
  if(!nh_.getParam("display_timing_warning",display_timing_warning_))
    display_timing_warning_ = false;
  if(!nh_.getParam("display_replanning_success", display_replanning_success_))
    display_replanning_success_ = false;
  if(!nh_.getParam("replanner_verbosity", replanner_verbosity_))
    replanner_verbosity_ = false;
  if(!nh_.getParam("display_replan_trj_point",display_replan_trj_point_))
    display_replan_trj_point_ = false;
  if(!nh_.getParam("display_replan_config",display_replan_config_))
    display_replan_config_ = false;
  if(!nh_.getParam("display_current_trj_point",display_current_trj_point_))
    display_current_trj_point_ = true;
  if(!nh_.getParam("display_current_config",display_current_config_))
    display_current_config_ = true;
  if(!nh_.getParam("benchmark",benchmark_))
    benchmark_ = false;
  if(!nh_.getParam("spawn_objs",spawn_objs_))
    spawn_objs_ = false;
  else
  {
    spawn_instants_.clear();
    if(!nh_.getParam("spawn_instants",spawn_instants_))
      spawn_instants_.push_back(0.5);

    if(!nh_.getParam("obj_type",obj_type_))
      obj_type_ = "little_box";

    if(!nh_.getParam("obj_max_size",obj_max_size_))
      obj_max_size_ = 0.07;
  }
}

void ReplannerManagerBase::attributeInitialization()
{
  stop_                            = false;
  terminated_                      = false;
  current_path_sync_needed_        = false;
  replanning_time_                 = 0.0  ;
  scaling_                         = 1.0  ;
  real_time_                       = 0.0  ;
  t_                               = 0.0  ;
  dt_                              = 1.0/trj_exec_thread_frequency_;
  replan_offset_                   = dt_replan_*K_OFFSET           ;
  t_replan_                        = t_+replan_offset_             ;
  replanning_thread_frequency_     = 100                           ;
  global_override_                 = 1.0                           ;

  if(group_name_.empty())
    throw std::invalid_argument("group name not set");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scn_cc_         = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scn_replanning_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  moveit_msgs::GetPlanningScene ps_srv;
  if(not plannning_scene_client_.call(ps_srv))
    throw std::runtime_error("call to planning scene srv not ok");

  planning_scene_msg_ = ps_srv.response.scene;

  if (not planning_scn_cc_        ->setPlanningSceneMsg(planning_scene_msg_))
    throw std::runtime_error("unable to update planning scene");
  if (not planning_scn_replanning_->setPlanningSceneMsg(planning_scene_msg_))
    throw std::runtime_error("unable to update planning scene");

  robot_state::RobotState state(planning_scn_cc_->getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup(group_name_);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  current_path_shared_ = current_path_replanning_->clone();

  checker_cc_         = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_cc_,        group_name_,10,checker_resolution_);
  checker_replanning_ = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_replanning_,group_name_,10,checker_resolution_);
  current_path_shared_    ->setChecker(checker_cc_        );
  current_path_replanning_->setChecker(checker_replanning_);
  solver_                 ->setChecker(checker_replanning_);

  trajectory_ = std::make_shared<pathplan::Trajectory>(current_path_shared_,nh_,planning_scn_replanning_,group_name_);
  robot_trajectory::RobotTrajectoryPtr trj = trajectory_->fromPath2Trj();

  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg) ;
  interpolator_.setTrajectory(tmp_trj_msg);
  interpolator_.setSplineOrder(1)         ;

  double scaling = 1.0;
  read_safe_scaling_? (scaling = readScalingTopics()):
                      (scaling = scaling_from_param_);

  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_  ,scaling);
  interpolator_.interpolate(ros::Duration(t_       ),pnt_         ,scaling);
  interpolator_.interpolate(ros::Duration(t_       ),pnt_unscaled_,    1.0);

  Eigen::VectorXd point2project(joint_names.size());
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++)
    point2project(i) = pnt_replan_.positions.at(i);

  configuration_replan_  = current_path_shared_->projectOnClosestConnection(point2project);
  current_configuration_ = current_path_shared_->getStartNode()->getConfiguration();

  initReplanner();
  replanner_->setVerbosity(replanner_verbosity_);

  obj_ids_.clear();

  new_joint_state_.position                 = pnt_.positions                  ;
  new_joint_state_.velocity                 = pnt_.velocities                 ;
  new_joint_state_.name                     = joint_names                     ;
  new_joint_state_.header.frame_id          = kinematic_model->getModelFrame();
  new_joint_state_.header.stamp             = ros::Time::now()                ;
  new_joint_state_unscaled_.position        = pnt_unscaled_.positions         ;
  new_joint_state_unscaled_.velocity        = pnt_unscaled_.velocities        ;
  new_joint_state_unscaled_.name            = joint_names                     ;
  new_joint_state_unscaled_.header.frame_id = kinematic_model->getModelFrame();
  new_joint_state_unscaled_.header.stamp    = ros::Time::now()                ;
}

void ReplannerManagerBase::overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name)
{
  double ovr;
  double global_override = 1.0;

  if (msg->data>100)
    ovr=1.0;
  else if (msg->data<0)
    ovr=0.0;
  else
    ovr=msg->data*0.01;

  overrides_.at(override_name)=ovr;
  for(const std::pair<std::string,double>& p: overrides_)
    global_override *= p.second;

  ovr_mtx_.lock();
  global_override_ = global_override;
  ovr_mtx_.unlock();
}

void ReplannerManagerBase::subscribeTopicsAndServices()
{
  scaling_topics_vector_.clear();
  for(const std::string &scaling_topic_name : scaling_topics_names_)
  {
    auto cb=boost::bind(&ReplannerManagerBase::overrideCallback,this,_1,scaling_topic_name);
    scaling_topics_vector_.push_back(std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Int64>>(nh_,scaling_topic_name,1,cb));

    overrides_.insert(std::pair<std::string,double>(scaling_topic_name,1.0));
    ROS_BOLDWHITE_STREAM("Subscribing speed override topic "<<scaling_topic_name.c_str());
  }

  target_pub_          = nh_.advertise<sensor_msgs::JointState>("/replanner_joint_target",         1);
  unscaled_target_pub_ = nh_.advertise<sensor_msgs::JointState>("/replanner_unscaled_joint_target",1);

  if(benchmark_)
    text_overlay_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("/rviz_text_overlay_replanner_bench",1);

  plannning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if(not plannning_scene_client_.waitForExistence(ros::Duration(10)))
    throw std::runtime_error("unable to connect to /get_planning_scene");

  if(spawn_objs_)
  {
    add_obj_    = nh_.serviceClient<object_loader_msgs::AddObjects>   ("/add_object_to_scene"     );
    remove_obj_ = nh_.serviceClient<object_loader_msgs::RemoveObjects>("/remove_object_from_scene");

    if(not add_obj_.waitForExistence(ros::Duration(10)))
    {
      ROS_ERROR("unable to connect to /add_object_to_scene");
      spawn_objs_ = false;
    }
    if(not remove_obj_.waitForExistence(ros::Duration(10)))
    {
      ROS_ERROR("unable to connect to /remove_object_to_scene");
      spawn_objs_ = false;
    }
  }

}

void ReplannerManagerBase::updateSharedPath()
{
  current_path_shared_ = current_path_replanning_->clone();
  current_path_shared_->setChecker(checker_cc_);
  current_path_sync_needed_ = true;
}

void ReplannerManagerBase::syncPathCost()
{
  paths_mtx_.lock();

  std::vector<ConnectionPtr> current_path_conn        = current_path_replanning_->getConnections();
  std::vector<ConnectionPtr> current_path_shared_conn = current_path_shared_    ->getConnections();

  std::vector<ConnectionPtr>::iterator it        = current_path_conn       .end();
  std::vector<ConnectionPtr>::iterator it_shared = current_path_shared_conn.end();

  while(it>current_path_conn.begin() && it_shared>current_path_shared_conn.begin())
  {
    it--;
    it_shared--;

    if((*it)->getParent()->getConfiguration() == (*it_shared)->getParent()->getConfiguration() &&
       (*it)->getChild() ->getConfiguration() == (*it_shared)->getChild() ->getConfiguration())
    {
      (*it)->setCost((*it_shared)->getCost());
    }
    else
      break;
  }

  current_path_replanning_->cost(); //update path cost
  paths_mtx_.unlock();
}

void ReplannerManagerBase::updatePathCost(const PathPtr& current_path_updated_copy)
{
  paths_mtx_.lock();
  if(not current_path_sync_needed_)
  {
    std::vector<ConnectionPtr> current_path_conns      = current_path_shared_     ->getConnections();
    std::vector<ConnectionPtr> current_path_copy_conns = current_path_updated_copy->getConnections();
    for(unsigned int z=0;z<current_path_conns.size();z++)
    {
      assert(current_path_conns.size() == current_path_copy_conns.size());
      assert((current_path_conns.at(z)->getParent()->getConfiguration() == current_path_copy_conns.at(z)->getParent()->getConfiguration()) &&
             (current_path_conns.at(z)->getChild() ->getConfiguration() == current_path_copy_conns.at(z)->getChild() ->getConfiguration()));

      current_path_conns.at(z)->setCost(current_path_copy_conns.at(z)->getCost());
    }
    current_path_shared_->cost();
  }

  if(current_path_shared_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity() && (display_timing_warning_ || display_replanning_success_))
    ROS_BOLDMAGENTA_STREAM("Obstacle detected!");

  paths_mtx_.unlock();
}

void ReplannerManagerBase::updateTrajectory()
{
  PathPtr trj_path = current_path_replanning_->clone();

  trj_path->removeNodes(1e-03); //remove useless nodes to speed up the trj (does not affect the tree because its a cloned path)

  assert([&]() ->bool{
           ConnectionPtr conn1;
           ConnectionPtr conn2;
           std::vector<ConnectionPtr> conns = trj_path->getConnectionsConst();
           for(unsigned int i=0;i<conns.size()-1;i++)
           {
             conn1 = trj_path->getConnections().at(i);
             conn2 = trj_path->getConnections().at(i+1);

             if(conn1->isParallel(conn2,1e-03))
             {
               return false;
             }
           }
           return true;
         }());

  trajectory_->setPath(trj_path);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj(pnt_);
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);

  interpolator_.setTrajectory(tmp_trj_msg);
  interpolator_.setSplineOrder(1);
}


void ReplannerManagerBase::replanningThread()
{
  ros::Rate lp(replanning_thread_frequency_);
  ros::WallTime tic,tic_rep,toc_rep;

  PathPtr path2project_on;
  Eigen::VectorXd current_configuration;
  Eigen::VectorXd point2project(pnt_replan_.positions.size());

  bool success = false;
  bool path_changed = false;
  bool path_obstructed = true;
  double replanning_duration = 0.0;
  Eigen::VectorXd projection = configuration_replan_;
  Eigen::VectorXd past_configuration_replan = projection;
  Eigen::VectorXd goal_conf = replanner_->getGoal()->getConfiguration();

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    trj_mtx_.lock();
    interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
    for(unsigned int i=0; i<pnt_replan_.positions.size();i++)
      point2project(i) = pnt_replan_.positions.at(i);

    current_configuration = current_configuration_;
    trj_mtx_.unlock();

    if((point2project-goal_conf).norm()>goal_tol_)
    {
      paths_mtx_.lock();
      path2project_on = current_path_shared_->clone();
      paths_mtx_.unlock();

      projection = path2project_on->projectOnPath(point2project,past_configuration_replan,false);
      past_configuration_replan = projection;

      if(not path2project_on->findConnection(projection))
        throw std::runtime_error("nn");

      replanner_mtx_.lock();
      configuration_replan_ = projection;
      replanner_mtx_.unlock();

      scene_mtx_.lock();
      checker_replanning_->setPlanningSceneMsg(planning_scene_msg_);
      syncPathCost();
      scene_mtx_.unlock();

      replanner_mtx_.lock();
      if(not (current_path_replanning_->findConnection(configuration_replan_)))
      {
        ROS_BOLDYELLOW_STREAM("configuration replan not found on path");
        trj_mtx_.lock();
        configuration_replan_ = current_configuration_;
        trj_mtx_.unlock();
      }

      replanner_->setChecker(checker_replanning_);
      replanner_->setCurrentConf(configuration_replan_);
      replanner_->setCurrentPath(current_path_replanning_);

      path_obstructed = (current_path_replanning_->getCostFromConf(configuration_replan_) == std::numeric_limits<double>::infinity());
      replanner_mtx_.unlock();

      success = false;
      path_changed = false;
      replanning_duration = 0.0;

      if(haveToReplan(path_obstructed))
      {
        int n_size_before = current_path_replanning_->getConnectionsSize();

        tic_rep=ros::WallTime::now();
        path_changed = replan();      //path may have changed even though replanning was unsuccessful
        toc_rep=ros::WallTime::now();

        replanning_duration = (toc_rep-tic_rep).toSec();
        success = replanner_->getSuccess();

//        if(replanning_duration>0.3) // ELIMINA
//          throw std::runtime_error("duration "+std::to_string(replanning_duration));

        bench_mtx_.lock();
        if(success)
          replanning_time_ = replanning_duration;
        bench_mtx_.unlock();

        assert(((not path_changed) && (n_size_before == current_path_replanning_->getConnectionsSize())) || (path_changed));
      }

      if(replanning_duration>=dt_replan_/0.9 && display_timing_warning_)
        ROS_BOLDYELLOW_STREAM("Replanning duration: "<<replanning_duration);
      if(display_replanning_success_)
        ROS_BOLDWHITE_STREAM("Success: "<< success <<" in "<< replanning_duration <<" seconds");

      if(path_changed && (not stop_))
      {
        replanner_mtx_.lock();
        trj_mtx_.lock();

        startReplannedPathFromNewCurrentConf(current_configuration_);

        current_path_replanning_ = replanner_->getReplannedPath();
        replanner_->setCurrentPath(current_path_replanning_);

        paths_mtx_.lock();
        updateSharedPath();
        paths_mtx_.unlock();

        updateTrajectory();

        past_configuration_replan = current_configuration_;

        t_=0.0;
        t_replan_=t_+replan_offset_;

        trj_mtx_.unlock();
        replanner_mtx_.unlock();
      }

      ros::WallTime toc=ros::WallTime::now();
      double duration = (toc-tic).toSec();

      if(display_timing_warning_ && duration>(dt_replan_/0.9))
      {
        ROS_BOLDYELLOW_STREAM("Replanning thread time expired: duration-> "<<duration);
        ROS_BOLDYELLOW_STREAM("replanning time-> "<<replanning_duration);
      }
    }

    lp.sleep();
  }

  ROS_BOLDCYAN_STREAM("Replanning thread is over");
}

void ReplannerManagerBase::collisionCheckThread()
{
  moveit_msgs::GetPlanningScene ps_srv;
  Eigen::VectorXd current_configuration_copy;

  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);

  ros::Rate lp(collision_checker_thread_frequency_);
  ros::WallTime tic;

  while ((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    if(not plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");

      stop_ = true;
      break;
    }

    scene_mtx_.lock();
    checker_cc_->setPlanningSceneMsg(ps_srv.response.scene);
    scene_mtx_.unlock();

    trj_mtx_.lock();

    current_configuration_copy = current_configuration_;

    paths_mtx_.lock();
    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;
    }
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    if((current_configuration_copy-replanner_->getGoal()->getConfiguration()).norm()<goal_tol_)
    {
      stop_ = true;
      break;
    }

    int conn_idx;
    current_path_copy->findConnection(current_configuration_copy,conn_idx);
    if(conn_idx<0)
      continue;
    else
      current_path_copy->isValidFromConf(current_configuration_copy,conn_idx,checker_cc_);

    scene_mtx_.lock();
    updatePathCost(current_path_copy);
    planning_scene_msg_ = ps_srv.response.scene;
    scene_mtx_.unlock();

    ros::WallTime toc=ros::WallTime::now();
    double duration = (toc-tic).toSec();

    if(duration>(1.0/collision_checker_thread_frequency_) && display_timing_warning_)
      ROS_BOLDYELLOW_STREAM("Collision checking thread time expired: total duration-> "<<duration);

    lp.sleep();
  }

  ROS_BOLDCYAN_STREAM("Collision check thread is over");
}

bool ReplannerManagerBase::replan()
{
  return replanner_->replan();
}

bool ReplannerManagerBase::stop()
{
  if(trj_exec_thread_                .joinable()) trj_exec_thread_  .join();
  if(replanning_thread_              .joinable()) replanning_thread_.join();
  if(col_check_thread_               .joinable()) col_check_thread_ .join();
  if(display_thread_                 .joinable()) display_thread_   .join();
  if(benchmark_  && benchmark_thread_.joinable()) benchmark_thread_ .join();
  if(spawn_objs_ && spawn_obj_thread_.joinable()) spawn_obj_thread_ .join();

  terminated_ = true;

  return true;
}

bool ReplannerManagerBase::cancel()
{
  stop_ = true ;
  return stop();
}

bool ReplannerManagerBase::run()
{
  ros::AsyncSpinner spinner(4);
  spinner.start();

  attributeInitialization();

  target_pub_         .publish(new_joint_state_         );
  unscaled_target_pub_.publish(new_joint_state_unscaled_);

  ROS_BOLDWHITE_STREAM("Launching threads..");

  display_thread_                   = std::thread(&ReplannerManagerBase::displayThread            ,this);  //it must be the first one launched, otherwise the first paths will be not displayed in time
  if(spawn_objs_) spawn_obj_thread_ = std::thread(&ReplannerManagerBase::spawnObjectsThread       ,this);
  if(benchmark_)  benchmark_thread_ = std::thread(&ReplannerManagerBase::benchmarkThread          ,this);
  replanning_thread_                = std::thread(&ReplannerManagerBase::replanningThread         ,this);
  col_check_thread_                 = std::thread(&ReplannerManagerBase::collisionCheckThread     ,this);
  ros::Duration(0.1).sleep()                                                                            ;
  trj_exec_thread_                  = std::thread(&ReplannerManagerBase::trajectoryExecutionThread,this);

  return true;
}

bool ReplannerManagerBase::start()
{
  run();
  stop();

  return true;
}

bool ReplannerManagerBase::startWithoutReplanning()
{
  ros::AsyncSpinner spinner(4);
  spinner.start();

  attributeInitialization();

  target_pub_         .publish(new_joint_state_         );
  unscaled_target_pub_.publish(new_joint_state_unscaled_);

  ROS_BOLDWHITE_STREAM("Launching threads..");

  display_thread_  = std::thread(&ReplannerManagerBase::displayThread             ,this);
  trj_exec_thread_ = std::thread(&ReplannerManagerBase::trajectoryExecutionThread ,this);

  if(display_thread_ .joinable()) display_thread_ .join();
  if(trj_exec_thread_.joinable()) trj_exec_thread_.join();

  return true;
}

double ReplannerManagerBase::readScalingTopics()
{
  ovr_mtx_.lock();
  double ovr = global_override_;
  ovr_mtx_.unlock();

  return ovr;
}

void ReplannerManagerBase::trajectoryExecutionThread()
{
  ros::WallTime tic,toc;
  PathPtr path2project_on;
  Eigen::VectorXd configuration_replan;
  Eigen::VectorXd goal_conf = replanner_->getGoal()->getConfiguration();
  double abscissa_replan_configuration, abscissa_current_configuration, duration;
  int conn_idx;

  ros::Rate lp(trj_exec_thread_frequency_);

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();
    real_time_ += dt_;

    replanner_mtx_.lock();
    configuration_replan = configuration_replan_;
    replanner_mtx_.unlock();

    trj_mtx_.lock();

    scaling_ = 1.0;
    read_safe_scaling_? (scaling_ = readScalingTopics()):
                        (scaling_ = scaling_from_param_);

    t_+= scaling_*dt_;
    t_replan_ = t_+replan_offset_;

    interpolator_.interpolate(ros::Duration(t_),pnt_         ,scaling_);
    interpolator_.interpolate(ros::Duration(t_),pnt_unscaled_,     1.0);

    Eigen::VectorXd point2project(pnt_.positions.size());
    for(unsigned int i=0; i<pnt_.positions.size();i++)
      point2project(i) = pnt_.positions.at(i);

    paths_mtx_.lock();
    path2project_on = current_path_shared_->clone();
    paths_mtx_.unlock();

    current_configuration_ = path2project_on->projectOnPath(point2project,current_configuration_,false);

    if(path2project_on->findConnection(configuration_replan,conn_idx))
    {
      abscissa_replan_configuration  = path2project_on->curvilinearAbscissaOfPoint(configuration_replan,conn_idx);
      abscissa_current_configuration = path2project_on->curvilinearAbscissaOfPoint(current_configuration_);
      if(abscissa_current_configuration>abscissa_replan_configuration) //the current confgiruation must not surpass that of replanning
        current_configuration_ = configuration_replan;
    }

    trj_mtx_.unlock();

    if((point2project-goal_conf).norm()<goal_tol_)
      stop_ = true;

    new_joint_state_.position              = pnt_.positions          ;
    new_joint_state_.velocity              = pnt_.velocities         ;
    new_joint_state_.header.stamp          = ros::Time::now()        ;
    new_joint_state_unscaled_.position     = pnt_unscaled_.positions ;
    new_joint_state_unscaled_.velocity     = pnt_unscaled_.velocities;
    new_joint_state_unscaled_.header.stamp = ros::Time::now()        ;

    target_pub_         .publish(new_joint_state_)         ;
    unscaled_target_pub_.publish(new_joint_state_unscaled_);

    toc = ros::WallTime::now();
    duration = (toc-tic).toSec();
    if(duration>(1/trj_exec_thread_frequency_) && display_timing_warning_)
      ROS_BOLDYELLOW_STREAM("Trj execution thread time expired: duration-> "<<duration);

    lp.sleep();
  }

  stop_ = true;
  terminated_ = true;

  ROS_BOLDCYAN_STREAM("Trajectory execution thread is over");
}

void ReplannerManagerBase::displayThread()
{
  PathPtr initial_path = current_path_shared_->clone();
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name_);

  int path_id,node_id,wp_id;
  std::vector<double> marker_scale(3,0.01);
  std::vector<double> marker_scale_sphere(3,0.02);
  std::vector<double> marker_color_initial_path   = {0.0,1.0,0.0,1.0};
  std::vector<double> marker_color_current_path   = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_color_current_config = {1.0,0.0,1.0,1.0};
  std::vector<double> marker_color_current_pnt    = {0.0,1.0,0.0,1.0};
  std::vector<double> marker_color_replan_config  = {0.0,0.0,0.0,1.0};
  std::vector<double> marker_color_replan_pnt     = {0.5,0.5,0.5,1.0};

  disp->clearMarkers();

  double display_thread_frequency = 2*trj_exec_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  int replan_id_first = 700000;
  int replan_id_last = replan_id_first+50;
  int replan_id = replan_id_first;

  while((not stop_) && ros::ok())
  {
    paths_mtx_.lock();
    pathplan::PathPtr current_path = current_path_shared_->clone();
    paths_mtx_.unlock();

    replanner_mtx_.lock();
    trj_mtx_.lock();
    Eigen::VectorXd current_configuration = current_configuration_;
    Eigen::VectorXd configuration_replan = configuration_replan_;
    trajectory_msgs::JointTrajectoryPoint pnt = pnt_;
    trajectory_msgs::JointTrajectoryPoint pnt_replan = pnt_replan_;
    trj_mtx_.unlock();
    replanner_mtx_.unlock();

    path_id = 10;
    node_id = 1000;
    wp_id = 10000;

    disp->changeConnectionSize(marker_scale);
    disp->displayPathAndWaypoints(current_path,path_id,wp_id,"pathplan",marker_color_current_path);

    disp->displayPathAndWaypoints(initial_path,path_id+2000,wp_id+2000,"pathplan",marker_color_initial_path);
    disp->defaultConnectionSize();

    disp->changeNodeSize(marker_scale_sphere);

    if(display_current_config_)
      disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),node_id,"pathplan",marker_color_current_config);

    Eigen::VectorXd point2project(pnt.positions.size());
    if(display_current_trj_point_)
    {
      for(unsigned int i=0; i<pnt.positions.size();i++)
        point2project[i] = pnt.positions.at(i);

      node_id +=1;
      disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color_current_pnt);
    }

    if(display_replan_config_)
    {
      //      node_id +=1;
      //      disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),node_id,"pathplan",marker_color_replan_config);

      disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),replan_id,"pathplan",marker_color_replan_config);
      replan_id = replan_id+1;

      if(replan_id >= replan_id_last)
      {
        replan_id = replan_id_first;
      }
    }

    if(display_replan_trj_point_)
    {
      for(unsigned int i=0; i<pnt_replan.positions.size();i++)
        point2project[i] = pnt_replan.positions.at(i);

      node_id +=1;
      disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color_replan_pnt);
    }

    disp->defaultNodeSize();

    lp.sleep();
  }

  ROS_BOLDCYAN_STREAM("Display thread is over");
}

void ReplannerManagerBase::spawnObjectsThread()
{
  object_loader_msgs::AddObjects srv_add_object;
  object_loader_msgs::RemoveObjects srv_remove_object;

  CollisionCheckerPtr checker = checker_cc_->clone();
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);

  MoveitUtils moveit_utils(planning_scene,group_name_);
  std::string last_link = planning_scene->getRobotModel()->getJointModelGroup(group_name_)->getLinkModelNames().back();

  PathPtr path_copy;
  Eigen::VectorXd conf, obj_pos, obj_conf;
  Eigen::VectorXd goal = current_path_shared_->getGoalNode()->getConfiguration();

  Eigen::Vector3d conf_3d, obj_pos_3d;
  Eigen::Vector3d goal_3d = forwardIk(goal,last_link,moveit_utils);

  geometry_msgs::Quaternion q;
  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;

  std::reverse(spawn_instants_.begin(),spawn_instants_.end());

  ros::Rate lp(trj_exec_thread_frequency_);

  while(not stop_ && ros::ok())
  {
    if(not spawn_instants_.empty())
    {
      if(real_time_>=spawn_instants_.back())
      {
        spawn_instants_.pop_back();

        object_loader_msgs::Object obj;
        obj.object_type = obj_type_;
        obj.pose.header.frame_id = "world";
        obj.pose.pose.orientation = q;

        replanner_mtx_.lock();
        paths_mtx_.lock();
        conf = configuration_replan_;
        path_copy = current_path_shared_->clone();
        path_copy->setChecker(checker);
        path_copy = path_copy->getSubpathFromConf(configuration_replan_,true);
        paths_mtx_.unlock();
        replanner_mtx_.unlock();

        conf_3d = forwardIk(conf,last_link,moveit_utils);

        double obj_abscissa = 0.0;
        std::srand(std::time(NULL));

        do
        {
          while(obj_abscissa<0.2 || obj_abscissa>0.8) //to not obstruct goal and current robot position
            obj_abscissa = double(rand())/double(RAND_MAX);

          obj_conf = path_copy->pointOnCurvilinearAbscissa(obj_abscissa);
          obj_pos_3d = forwardIk(obj_conf,last_link,moveit_utils,obj.pose.pose);

          if((obj_pos_3d-conf_3d).norm()>obj_max_size_ && (obj_conf-conf).norm()>obj_max_size_ && (obj_pos_3d-goal_3d).norm()>obj_max_size_ && (obj_conf-goal).norm()>obj_max_size_)
          {
            obj_pos = obj_pos_3d;
            break;
          }

        }while((not stop_) && ros::ok());

        if(stop_ || not ros::ok())
          break;

        srv_add_object.request.objects.clear();
        srv_add_object.request.objects.push_back(obj);

        ROS_BOLDMAGENTA_STREAM("Obstacle spawned!");
        if(not add_obj_.call(srv_add_object))
        {
          ROS_ERROR("call to add obj srv not ok");

          stop_ = true;
          break;
        }

        if(not srv_add_object.response.success)
          ROS_ERROR("add obj srv error");
        else
        {
          bench_mtx_.lock();
          obj_ids_.push_back(srv_add_object.response.ids.front());
          obj_pos_.push_back(obj_pos);
          bench_mtx_.unlock();

          for (const std::string& str:srv_add_object.response.ids)
            srv_remove_object.request.obj_ids.push_back(str);
        }
      }
    }

    lp.sleep();
  }

  if (not remove_obj_.call(srv_remove_object))
    ROS_ERROR("call to remove obj srv not ok");
  if(not srv_remove_object.response.success)
    ROS_ERROR("remove obj srv error");

  ROS_BOLDCYAN_STREAM("Spawn objects thread is over");
}

void ReplannerManagerBase::benchmarkThread()
{
  bool success = true;
  double path_length = 0.0;
  PathPtr current_path;
  ConnectionPtr current_conn;
  std::vector<std::string> obj_ids;
  std::vector<Eigen::VectorXd> obj_pos;
  std::vector<std::string>::iterator it;
  trajectory_msgs::JointTrajectoryPoint pnt;
  std::vector<double> replanning_time_vector;
  std::vector<std::string> already_collided_obj;
  Eigen::VectorXd old_current_configuration, current_configuration, current_configuration_3d, old_pnt_conf, pnt_conf;

  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scn_cc_);

  MoveitUtils moveit_utils(planning_scene,group_name_);
  std::string last_link = planning_scene->getRobotModel()->getJointModelGroup(group_name_)->getLinkModelNames().back();
  CollisionCheckerPtr checker = std::make_shared<MoveitCollisionChecker>(planning_scene,group_name_);

  paths_mtx_.lock();
  Eigen::VectorXd start = current_path_shared_->getStartNode()->getConfiguration();
  Eigen::VectorXd goal  = current_path_shared_->getGoalNode ()->getConfiguration();
  double initial_path_length = current_path_shared_->length();
  paths_mtx_.unlock();

  Eigen::VectorXd goal_3d = forwardIk(goal,last_link,moveit_utils);

  pnt_conf = start;
  current_configuration = start;

  double distance;
  double distance_start_goal = (goal-start).norm();

  int n_collisions = 0;

  std_msgs::ColorRGBA fg_color_green, fg_color_red, bg_color;
  fg_color_green.r = 0;   fg_color_red.r = 1;   bg_color.r = 0;
  fg_color_green.g = 0.8; fg_color_red.g = 0;   bg_color.g = 0;
  fg_color_green.b = 0.2; fg_color_red.b = 0;   bg_color.b = 0;
  fg_color_green.a = 0.8; fg_color_red.a = 0.8; bg_color.a = 0;

  jsk_rviz_plugins::OverlayText overlayed_text;
  overlayed_text.font = "FreeSans";
  overlayed_text.bg_color = bg_color;
  overlayed_text.height = 70;
  overlayed_text.left = 10;
  overlayed_text.top = 80;
  overlayed_text.line_width = 2;
  overlayed_text.text_size = 15;
  overlayed_text.width = 1000;

  overlayed_text.action = overlayed_text.DELETE;
  text_overlay_pub_.publish(overlayed_text);

  overlayed_text.action = overlayed_text.ADD;

  double cycle_duration;
  ros::WallTime tic, toc;
  double freq = 2*trj_exec_thread_frequency_;
  ros::Rate lp(freq);

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    if(success)
    {
      std::string text = "Success: TRUE \nCollided objects: 0";
      overlayed_text.text = text;
      overlayed_text.fg_color = fg_color_green;
      text_overlay_pub_.publish(overlayed_text);
    }

    old_pnt_conf = pnt_conf;
    old_current_configuration = current_configuration;

    trj_mtx_.lock();
    paths_mtx_.lock();
    pnt = pnt_;
    current_path = current_path_shared_->clone();
    current_configuration = current_configuration_;
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    current_configuration_3d = forwardIk(current_configuration,last_link,moveit_utils);

    for(unsigned int i=0; i<pnt.positions.size();i++)
      pnt_conf(i) = pnt.positions[i];

    /* Replanning time */
    bench_mtx_.lock();
    if(replanning_time_ != 0.0)
      replanning_time_vector.push_back(replanning_time_);

    replanning_time_ = 0.0;
    bench_mtx_.unlock();

    /* Path length */
    //    distance = (current_configuration-old_current_configuration).norm();
    distance = (pnt_conf-old_pnt_conf).norm();
    if(distance>0.3)
    {
      //current_configuration = old_current_configuration;
      ROS_BOLDRED_STREAM("Skipping path length increment! Distance: "<<distance);
    }
    else
      path_length += distance;

    /* Collisions with mobile obstacles */
    bench_mtx_.lock();
    obj_ids = obj_ids_;
    obj_pos = obj_pos_;
    bench_mtx_.unlock();

    for(unsigned int i=0;i<obj_pos.size();i++)
    {
      if((current_configuration_3d-obj_pos[i]).norm()<obj_max_size_ && ((goal_3d-obj_pos[i]).norm()>obj_max_size_))
      {
        it = std::find(already_collided_obj.begin(),already_collided_obj.end(),obj_ids[i]);

        scene_mtx_.lock();
        checker->setPlanningSceneMsg(planning_scene_msg_);
        scene_mtx_.unlock();

        if(checker->check(current_configuration))
        {
          throw std::runtime_error("current conf not in collision, dist "+std::to_string((current_configuration_3d-obj_pos[i]).norm())+" max size "+std::to_string(obj_max_size_));
        }

        if(it>=already_collided_obj.end())
        {
          current_conn = current_path->findConnection(current_configuration);
          if(current_conn && (current_conn->getCost() == std::numeric_limits<double>::infinity()))
          {
            n_collisions++;
            already_collided_obj.push_back(obj_ids[i]);
            success = false;

            std::string txt = "Success: FALSE \nCollided objects: "+std::to_string(n_collisions);
            overlayed_text.text = txt;
            overlayed_text.fg_color = fg_color_red;
            text_overlay_pub_.publish(overlayed_text);
          }
        }
        break;
      }
    }

    toc = ros::WallTime::now();
    cycle_duration = (toc-tic).toSec();
    if(cycle_duration>(1/freq) && display_timing_warning_)
      ROS_BOLDYELLOW_STREAM("Benchmark thread time expired: duration-> "<<cycle_duration);

    lp.sleep();
  }

  double sum, mean, std_dev, variance, max_replanning_time;
  sum = std::accumulate(replanning_time_vector.begin(),replanning_time_vector.end(),0.0);
  mean = sum/replanning_time_vector.size();

  variance = 0.0;
  for(const double& d:replanning_time_vector)
    variance += std::pow(d - mean, 2);

  std_dev = std::sqrt(variance / replanning_time_vector.size());

  if(not replanning_time_vector.empty())
    max_replanning_time = *(std::max_element(replanning_time_vector.begin(),replanning_time_vector.end()));
  else
    max_replanning_time = 0.0;

  bench_mtx_.lock();
  unsigned int number_of_objects = obj_ids_.size();
  bench_mtx_.unlock();

  std::string replanner_type = "replanner";
  nh_.getParam("replanner_type",replanner_type);

  std::string test_name = "test";
  nh_.getParam("test_name",test_name);

  std::string bench_name = "bench";
  nh_.getParam("bench_name",bench_name);

  std::string path = "./replanners_benchmark";
  std::string file_name = path+"/"+bench_name+"/"+replanner_type+"/"+test_name+".bin";

  boost::filesystem::path dir(path);
  if(not (boost::filesystem::exists(dir)))
    boost::filesystem::create_directory(dir);

  boost::filesystem::path dir2(path+"/"+bench_name);
  if(not (boost::filesystem::exists(dir2)))
    boost::filesystem::create_directory(dir2);

  boost::filesystem::path dir3(path+"/"+bench_name+"/"+replanner_type);
  if(not (boost::filesystem::exists(dir3)))
    boost::filesystem::create_directory(dir3);

  std::ofstream file;
  file.open(file_name,std::ios::out | std::ios::binary);

  const size_t bufsize = 1024 * 1024;
  std::unique_ptr<char[]> buf;
  buf.reset(new char[bufsize]);

  file.rdbuf()->pubsetbuf(buf.get(), bufsize);

  file.write((char*) &success,             sizeof(success            ));
  file.write((char*) &number_of_objects,   sizeof(number_of_objects  ));
  file.write((char*) &n_collisions,        sizeof(n_collisions       ));
  file.write((char*) &path_length,         sizeof(path_length        ));
  file.write((char*) &distance_start_goal, sizeof(distance_start_goal));
  file.write((char*) &initial_path_length, sizeof(initial_path_length));
  file.write((char*) &real_time_,          sizeof(real_time_         ));
  file.write((char*) &mean,                sizeof(mean               ));
  file.write((char*) &std_dev,             sizeof(std_dev            ));
  file.write((char*) &max_replanning_time, sizeof(max_replanning_time));

  file.flush();
  file.close();

  ROS_BOLDBLUE_STREAM("\nFile "<<file_name<<" saved!\n* success: "<<success
                      <<"\n* number_of_objects: "<<number_of_objects
                      <<"\n* number_of_collisions: "<<n_collisions
                      <<"\n* path length: "<<path_length
                      <<"\n* distance start-goal: "<<distance_start_goal
                      <<"\n* initial path length: "<<initial_path_length
                      <<"\n* time: "<<real_time_
                      <<"\n* replanning time mean: "<<mean
                      <<"\n* replanning time std dev: "<<std_dev
                      <<"\n* max replanning time: "<<max_replanning_time);

  if(n_collisions == 0 && not success)
    throw std::runtime_error("no collisions but success false!");

  ROS_BOLDCYAN_STREAM("Benchamrk thread is over");
}

Eigen::Vector3d ReplannerManagerBase::forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const MoveitUtils& util)
{
  geometry_msgs::Pose pose;
  return forwardIk(conf,last_link,util,pose);
}


Eigen::Vector3d ReplannerManagerBase::forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const MoveitUtils& util,geometry_msgs::Pose& pose)
{
  moveit::core::RobotState state = util.fromWaypoints2State(conf);
  tf::poseEigenToMsg(state.getGlobalLinkTransform(last_link),pose);

  std::vector<double> v(3);
  v[0] = pose.position.x;
  v[1] = pose.position.y;
  v[2] = pose.position.z;

  Eigen::Vector3d position(v.size());
  for(unsigned int i=0;i<v.size();i++)
    position[i] = v[i];

  return position;
}
}
