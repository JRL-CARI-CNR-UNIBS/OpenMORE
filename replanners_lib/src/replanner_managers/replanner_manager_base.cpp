#include "replanners_lib/replanner_managers/replanner_manager_base.h"

namespace pathplan
{
ReplannerManagerBase::ReplannerManagerBase(PathPtr &current_path,
                                           TreeSolverPtr &solver,
                                           ros::NodeHandle &nh)
{
  current_path_replanning_ = current_path;
  solver_       = solver      ;
  nh_           = nh          ;

  fromParam();
  subscribeTopicsAndServices();
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReplannerManagerBase::setChainProperties(std::string &group_name, std::string &base_link, std::string &last_link)
{
  group_name_ = group_name;
  base_link_  = base_link ;
  last_link_  = last_link ;

  ROS_INFO("Chain propierties set -> group name: %s, base link: %s, tool link: %s.",group_name_.c_str(),base_link_.c_str(),last_link_.c_str());
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
  if(!nh_.getParam("checker_resolution",checker_resol_))
  {
    ROS_ERROR("checker_resolution not set, set 0.05");
    checker_resol_ = 0.05;
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

  if(!nh_.getParam("group_name", group_name_)) ROS_ERROR("group_name not set, maybe set later with setChainProperties(..)?");
  if(!nh_.getParam("base_link",  base_link_ )) ROS_ERROR("base_link  not set, maybe set later with setChainProperties(..)?");
  if(!nh_.getParam("last_link",  last_link_ )) ROS_ERROR("last_link  not set, maybe set later with setChainProperties(..)?");

  if(!nh_.getParam("goal_toll",                  goal_toll_                 )) goal_toll_                  = 1.0e-03  ;
  if(!nh_.getParam("scaling",                    scaling_from_param_        )) scaling_from_param_         = 1.0      ;
  if(!nh_.getParam("spawn_objs",                 spawn_objs_                )) spawn_objs_                 = false    ;
  if(!nh_.getParam("display_timing_warning",     display_timing_warning_    )) display_timing_warning_     = false    ;
  if(!nh_.getParam("display_replanning_success", display_replanning_success_)) display_replanning_success_ = false    ;
}

void ReplannerManagerBase::attributeInitialization()
{
  stop_                            = false;
  replanning_                      = false;
  path_obstructed_                 = false;
  computing_avoiding_path_         = false;
  current_path_changed_            = false;
  n_conn_                          = 0    ;
  real_time_                       = 0.0  ;
  t_                               = 0.0  ;
  dt_                              = 1/trj_exec_thread_frequency_;
  replan_offset_                   = (dt_replan_-dt_)*K_OFFSET   ;
  t_replan_                        = t_+replan_offset_           ;
  //  replanning_thread_frequency_     = 1/dt_replan_            ;
  replanning_thread_frequency_     = 100                         ;
  global_override_                 = 1.0                         ;

  if(group_name_.empty()) throw std::invalid_argument("group name not set");
  if(base_link_ .empty()) throw std::invalid_argument("base link not set" );
  if(last_link_ .empty()) throw std::invalid_argument("last link not set" );

  robot_model_loader::RobotModelLoader             robot_model_loader("robot_description")                         ;
  robot_model::RobotModelPtr kinematic_model     = robot_model_loader.getModel()                                   ;
  planning_scn_cc_                                  = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scn_replanning_                       = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  moveit_msgs::GetPlanningScene ps_srv;
  if (!plannning_scene_client_.call(ps_srv)                                ) ROS_ERROR("call to srv not ok"             );
  if (!planning_scn_cc_->setPlanningSceneMsg(ps_srv.response.scene)           ) ROS_ERROR("unable to update planning scene");
  if (!planning_scn_replanning_->setPlanningSceneMsg(ps_srv.response.scene)) ROS_ERROR("unable to update planning scene");

  robot_state::RobotState state(planning_scn_cc_->getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup(group_name_)        ;
  std::vector<std::string> joint_names                  = joint_model_group->getActiveJointModelNames();
  unsigned int dof                                      = joint_names.size()                           ;

  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  lb_ = lb;
  ub_ = ub;

  current_path_shared_ = current_path_replanning_->clone();

  checker_cc_ = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_cc_, group_name_,5, checker_resol_           );
  checker_replanning_           = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_replanning_, group_name_,5, checker_resol_);
  current_path_replanning_->setChecker(checker_replanning_);
  current_path_shared_->setChecker(checker_cc_);
  solver_->setChecker(checker_replanning_);

  root_for_detach_ = current_path_replanning_->getTree()->getRoot();

  trajectory_                              = std::make_shared<pathplan::Trajectory>(current_path_shared_,nh_,planning_scn_replanning_,group_name_);
  robot_trajectory::RobotTrajectoryPtr trj = trajectory_->fromPath2Trj()                                                                   ;

  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg) ;
  interpolator_.setTrajectory(tmp_trj_msg);
  interpolator_.setSplineOrder(1)         ;

  double scaling = 1.0;
  if(read_safe_scaling_) scaling = readScalingTopics();
  else                   scaling = scaling_from_param_;

  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_  ,scaling);
  interpolator_.interpolate(ros::Duration(t_       ),pnt_         ,scaling);
  interpolator_.interpolate(ros::Duration(t_       ),pnt_unscaled_,1.0    );

  Eigen::VectorXd point2project(dof);
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);

  configuration_replan_  = current_path_shared_->projectOnClosestConnection(point2project);
  current_configuration_ = current_path_shared_->getWaypoints().front()                   ;

  n_conn_ = 0;

  initReplanner();

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
  if (msg->data>100)
    ovr=1.0;
  else if (msg->data<0)
    ovr=0.0;
  else
    ovr=msg->data*0.01;
  overrides_.at(override_name)=ovr;
  double global_override=1;
  for (const std::pair<std::string,double>& p: overrides_)
    global_override *= p.second;

  ovr_mtx_.lock();
  global_override_   = global_override;
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
    ROS_FATAL("subscribe topic %s",scaling_topic_name.c_str());
  }

  target_pub_              = nh_.advertise<sensor_msgs::JointState>              ("/joint_target",          1);
  unscaled_target_pub_     = nh_.advertise<sensor_msgs::JointState>              ("/unscaled_joint_target", 1);
  plannning_scene_client_  = nh_.serviceClient<moveit_msgs::GetPlanningScene>    ("/get_planning_scene"      );
  add_obj_                 = nh_.serviceClient<object_loader_msgs::AddObjects>   ("/add_object_to_scene"     );
  remove_obj_              = nh_.serviceClient<object_loader_msgs::RemoveObjects>("/remove_object_from_scene");

  if(!plannning_scene_client_.waitForExistence(ros::Duration(10))) throw std::invalid_argument("unable to connect to /get_planning_scene");
  if(!add_obj_               .waitForExistence(ros::Duration(10))) ROS_ERROR("unable to connect to /add_object_to_scene"                 );
  if(!remove_obj_            .waitForExistence(ros::Duration(10))) ROS_ERROR("unable to connect to /remove_object_to_scene"              );
}

void ReplannerManagerBase::replanningThread()
{
  ros::Rate lp(replanning_thread_frequency_);

  bool do_replan = true;
  bool success = false;
  bool path_changed = false;
  bool path_obstructed = true;
  //  bool old_path_obstructed = path_obstructed_;
  double replanning_duration = 0;
  int n_conn_replan = 0;
  Eigen::VectorXd past_configuration_replan = configuration_replan_;
  Eigen::VectorXd goal = replanner_->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();

  //  double abscissa;
  //  double past_abscissa;
  PathPtr path2project_on;
  Eigen::VectorXd projection;
  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  ros::WallTime tic_rep,toc_rep;

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while(!stop && ros::ok())
  {
    ros::WallTime tic=ros::WallTime::now();

    trj_mtx_.lock();
    //t_replan_ = t_+replan_offset_; updated in trj_thread

    interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
    for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);
    trj_mtx_.unlock();

    do_replan =  (point2project-goal).norm()>goal_toll_;

    if(do_replan)
    {
      scene_mtx_.lock();
      moveit_msgs::PlanningScene scn;
      planning_scn_cc_->getPlanningSceneMsg(scn);
      checker_replanning_->setPlanningSceneMsg(scn);
      scene_mtx_.unlock();

      replanner_mtx_.lock();
      past_configuration_replan = configuration_replan_;
      path2project_on = replanner_->getCurrentPath()->clone();
      replanner_mtx_.unlock();

      projection = path2project_on->projectOnClosestConnectionKeepingPastPrj(point2project,past_configuration_replan,n_conn_replan);

      replanner_mtx_.lock();
      configuration_replan_ = projection;
      replanner_mtx_.unlock();

      paths_mtx_.lock(); //replanningThread should work with the original current path, so its copy is used for collision check and then the cost is updated here
      if(!current_path_changed_) //se fallisce il replan, rimane il nodo nella config del replan precedente e qua non va bene, il nodo andrebbe tolto
      {
        if(current_path_replanning_->getConnections().size() != current_path_shared_->getConnections().size())
        {
          ROS_INFO_STREAM("current path replanning size: "<<current_path_replanning_->getConnections().size()<<" current path shared size: "<<current_path_shared_->getConnections().size());

          for(const ConnectionPtr conn:current_path_replanning_->getConnections())
          {
            ROS_INFO_STREAM("r parent: "<<conn->getParent()->getConfiguration().transpose());
            ROS_INFO_STREAM("r child : "<<conn->getChild()->getConfiguration().transpose());
          }

          for(const ConnectionPtr conn:current_path_shared_->getConnections())
          {
            ROS_INFO_STREAM("s parent: "<<conn->getParent()->getConfiguration().transpose());
            ROS_INFO_STREAM("s child : "<<conn->getChild()->getConfiguration().transpose());
          }

          assert(0);
        }

        for(unsigned int i=0;i<current_path_replanning_->getConnections().size();i++)
        {
          current_path_replanning_->getConnections().at(i)->setCost(current_path_shared_->getConnections().at(i)->getCost());
        }
        current_path_replanning_->cost(); //update path cost
      }
      paths_mtx_.unlock();

      checker_mtx_.lock();
      replanner_mtx_.lock();

      if(current_path_replanning_->findConnection(configuration_replan_) == NULL)
      {
        trj_mtx_.lock();
        configuration_replan_ = current_configuration_;
        trj_mtx_.unlock();
      }

      replanner_->setCurrentConf(configuration_replan_);
      replanner_->setCurrentPath(current_path_replanning_);

      path_obstructed = path_obstructed_;

      replanner_mtx_.unlock();
      checker_mtx_.unlock();

      success = false;
      path_changed = false;
      replanning_duration = 0;
      if(haveToReplan(path_obstructed))
      {
        ROS_WARN("DEVE RIPIANIFICARE");

        checker_mtx_.lock();
        computing_avoiding_path_ = true;
        checker_mtx_.unlock();

        planning_mtx_.lock();

        replanning_ = true;

        tic_rep=ros::WallTime::now();
        path_changed = replan();
        toc_rep=ros::WallTime::now();

        success = replanner_->getSuccess();
        replanning_ = false;

        planning_mtx_.unlock();

        replanning_duration = (toc_rep-tic_rep).toSec();
      }

      if(replanning_duration>=dt_replan_/0.9 && display_timing_warning_) ROS_WARN("Replanning duration: %f",(toc_rep-tic_rep).toSec());
      if(display_replanning_success_) ROS_INFO_STREAM("Success: "<< success <<" in "<< replanning_duration <<" seconds");

      stop_mtx_.lock();
      bool stop = stop_;
      stop_mtx_.unlock();

      if(path_changed && !stop)
      {
        replanner_mtx_.lock();
        trj_mtx_.lock();

        connectToReplannedPath();
        replanner_->getReplannedPath()->setTree(current_path_replanning_->getTree());
        current_path_replanning_ = replanner_->getReplannedPath();
        replanner_->setCurrentPath(current_path_replanning_);

        paths_mtx_.lock();
        current_path_shared_ = current_path_replanning_->clone();
        current_path_changed_ = true;
        paths_mtx_.unlock();

        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory_->setPath(current_path_replanning_);
        robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj(pnt_);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);
        interpolator_.setTrajectory(tmp_trj_msg);
        interpolator_.setSplineOrder(1);

        t_=0;
        t_replan_=0;
        n_conn_ = 0;
        n_conn_replan = 0;

        trj_mtx_.unlock();
        replanner_mtx_.unlock();

        checker_mtx_.lock();

        path_obstructed_ = false;
        computing_avoiding_path_ = false;

        checker_mtx_.unlock();
      }
      else
      {
        checker_mtx_.lock();
        computing_avoiding_path_ = false;
        checker_mtx_.unlock();
      }

      ros::WallTime toc=ros::WallTime::now();
      double duration = (toc-tic).toSec();

      if(display_timing_warning_ && duration>(dt_replan_/0.9))
      {
        ROS_WARN("Replanning thread time expired: duration-> %f",duration);
        ROS_WARN("replanning time-> %f",(toc_rep-tic_rep).toSec());
      }
    }

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

void ReplannerManagerBase::collisionCheckThread()
{
  ros::Rate lp(collision_checker_thread_frequency_);

  Eigen::VectorXd current_configuration_copy;
  PathPtr current_path_copy = current_path_shared_->clone();
  moveit_msgs::GetPlanningScene ps_srv;

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while (!stop && ros::ok())
  {
    ros::WallTime tic = ros::WallTime::now();

    scene_mtx_.lock();
    if (!plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }
    checker_cc_->setPlanningSceneMsg(ps_srv.response.scene);
    scene_mtx_.unlock();

    paths_mtx_.lock();
    if(current_path_changed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_changed_ = false;
    }
    paths_mtx_.unlock();

    trj_mtx_.lock();
    current_configuration_copy = current_configuration_;
    trj_mtx_.unlock();

    bool path_obstructed = !(current_path_copy->isValidFromConf(current_configuration_copy,checker_cc_));

    paths_mtx_.lock();
    if(!current_path_changed_)  //if changed, it is useless checking current_path_copy
    {
      for(unsigned int z=0;z<current_path_shared_->getConnections().size();z++)
      {
        current_path_shared_->getConnections().at(z)->setCost(current_path_copy->getConnections().at(z)->getCost());
      }

      if(current_path_shared_->cost() == std::numeric_limits<double>::infinity())
        ROS_WARN("path obstructed");
      else
        ROS_INFO("path free");
    }
    paths_mtx_.unlock();

    checker_mtx_.lock();

    if(!computing_avoiding_path_)
      path_obstructed_ = path_obstructed;


    checker_mtx_.unlock();

    ros::WallTime toc=ros::WallTime::now();
    double duration = (toc-tic).toSec();

    if(duration>(1/collision_checker_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Collision checking thread time expired: duration-> %f",duration);
    }

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

bool ReplannerManagerBase::stop()
{
  if(trj_exec_thread_                .joinable()) trj_exec_thread_  .join();
  if(replanning_thread_              .joinable()) replanning_thread_.join();
  if(col_check_thread_               .joinable()) col_check_thread_ .join();
  if(display_thread_                 .joinable()) display_thread_   .join();
  if(spawn_objs_ && spawn_obj_thread_.joinable()) spawn_obj_thread_ .join();
  return true;
}

bool ReplannerManagerBase::cancel()
{
  stop_mtx_.lock();
  stop_ = true ;
  stop_mtx_.unlock();

  return stop();
}

bool ReplannerManagerBase::run()
{
  ros::AsyncSpinner spinner(4);
  spinner.start();

  attributeInitialization();

  target_pub_         .publish(new_joint_state_         );
  unscaled_target_pub_.publish(new_joint_state_unscaled_);

  ROS_WARN("Launching threads..");

  display_thread_                   = std::thread(&ReplannerManagerBase::displayThread             ,this);  //it must be the first one launched, otherwise the first paths will be not displayed in time
  if(spawn_objs_) spawn_obj_thread_ = std::thread(&ReplannerManagerBase::spawnObjects              ,this);
  replanning_thread_                = std::thread(&ReplannerManagerBase::replanningThread          ,this);
  col_check_thread_                 = std::thread(&ReplannerManagerBase::collisionCheckThread      ,this);
  ros::Duration(2).sleep()                                                                               ;
  trj_exec_thread_                  = std::thread(&ReplannerManagerBase::trajectoryExecutionThread ,this);

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

  ROS_WARN("Launching threads..");

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
  PathPtr path2project_on;
  Eigen::VectorXd goal_conf = current_path_shared_->getWaypoints().back();
  Eigen::VectorXd past_current_configuration = current_configuration_;
  trajectory_msgs::JointTrajectoryPoint pnt;
  trajectory_msgs::JointTrajectoryPoint pnt_unscaled;

  ros::Rate lp(trj_exec_thread_frequency_);

  stop_mtx_.lock()  ;
  bool stop = stop_ ;
  stop_mtx_.unlock();

  while(!stop && ros::ok())
  {
    ros::WallTime tic = ros::WallTime::now();
    real_time_ += dt_;

    trj_mtx_.lock();

    double scaling = 1.0;
    if(read_safe_scaling_) scaling = readScalingTopics();
    else                   scaling = scaling_from_param_;

    t_+= scaling*dt_;
    t_replan_ = t_+(replan_offset_*scaling);

    paths_mtx_.lock();
    path2project_on = current_path_shared_->clone();
    paths_mtx_.unlock();

    interpolator_.interpolate(ros::Duration(t_),pnt_         ,scaling);
    interpolator_.interpolate(ros::Duration(t_),pnt_unscaled_,    1.0);

    pnt          = pnt_         ;
    pnt_unscaled = pnt_unscaled_;

    Eigen::VectorXd point2project(pnt_.positions.size());
    for(unsigned int i=0; i<pnt_.positions.size();i++) point2project[i] = pnt_.positions.at(i);

    past_current_configuration = current_configuration_;
    current_configuration_ = path2project_on->projectOnClosestConnectionKeepingPastPrj(point2project,past_current_configuration,n_conn_);

    trj_mtx_.unlock();

    if((point2project-goal_conf).norm()<goal_toll_)
    {
      stop_mtx_.lock()  ;
      stop_     = true  ;
      stop_mtx_.unlock();
    }

    new_joint_state_unscaled_.position     = pnt_unscaled.positions ;
    new_joint_state_unscaled_.velocity     = pnt_unscaled.velocities;
    new_joint_state_unscaled_.header.stamp = ros::Time::now()       ;
    new_joint_state_.position              = pnt.positions          ;
    new_joint_state_.velocity              = pnt.velocities         ;
    new_joint_state_.header.stamp          = ros::Time::now()       ;

    unscaled_target_pub_.publish(new_joint_state_unscaled_)         ;
    target_pub_         .publish(new_joint_state_)                  ;

    ros::WallTime toc = ros::WallTime::now();
    double duration = (toc-tic).toSec();

    if(duration>(1/trj_exec_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Trj execution thread time expired: duration-> %f",duration);
    }

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }

  stop_mtx_.lock();
  stop_ = true;
  stop_mtx_.unlock();

  ROS_ERROR("STOP");
}

void ReplannerManagerBase::displayThread()
{
  PathPtr initial_path = current_path_shared_->clone();

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_,last_link_);
  ros::Duration(0.5).sleep();

  std::vector<double> marker_color;
  std::vector<double> marker_scale;
  std::vector<double> marker_scale_sphere(3,0.02);

  disp->clearMarkers();

  double display_thread_frequency = 0.75*trj_exec_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while(!stop && ros::ok())
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

    int path_id = 10;
    int node_id = 1000;
    int wp_id = 10000;

    marker_scale = {0.01,0.01,0.01};
    marker_color =  {1.0,1.0,0.0,1.0};
    disp->changeConnectionSize(marker_scale);
    disp->displayPathAndWaypoints(current_path,path_id,wp_id,"pathplan",marker_color);

    marker_color =  {0.0,1.0,0.0,1.0};
    disp->displayPathAndWaypoints(initial_path,path_id+2000,wp_id+2000,"pathplan",marker_color);
    disp->defaultConnectionSize();

    //    disp->changeNodeSize(marker_scale_sphere);
    //    marker_color = {1.0,0.0,1.0,1.0};
    //    disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),node_id,"pathplan",marker_color);

    Eigen::VectorXd point2project(pnt.positions.size());
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    node_id +=1;
    marker_color = {0.0,1.0,0.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);

    //    node_id +=1;
    //    marker_color = {0.0,0.0,0.0,1.0};
    //    disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),node_id,"pathplan",marker_color);

    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
    node_id +=1;
    marker_color = {0.5,0.5,0.5,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);

    disp->defaultNodeSize();

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

void ReplannerManagerBase::spawnObjects()
{
  object_loader_msgs::AddObjects srv_add_object;
  object_loader_msgs::RemoveObjects srv_remove_object;
  MoveitUtils moveit_utils(planning_scn_cc_,group_name_);

  ros::Rate lp(0.5*trj_exec_thread_frequency_);

  bool object_spawned = false;
  bool second_object_spawned = false;

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while(!stop && ros::ok())
  {
    //    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////
    if(real_time_>=2.0 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && real_time_>=1.0)
    {
      if (!add_obj_.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }
      object_loader_msgs::Object obj;
      obj.object_type="scatola";

      int obj_conn_pos;
      int idx_current_conn;

      replanner_mtx_.lock();
      trj_mtx_.lock();
      replanner_->getCurrentPath()->findConnection(current_configuration_,idx_current_conn);
      trj_mtx_.unlock();
      replanner_mtx_.unlock();

      replanner_mtx_.lock();
      int size = replanner_->getCurrentPath()->getConnections().size();
      replanner_mtx_.unlock();

      std::srand(time(NULL));
      obj_conn_pos = (rand() % (size-idx_current_conn)) + idx_current_conn;

      pathplan::ConnectionPtr obj_conn;
      pathplan::NodePtr obj_parent;
      pathplan::NodePtr obj_child;
      Eigen::VectorXd obj_pos;

      replanner_mtx_.lock();
      trj_mtx_.lock();
      if(obj_conn_pos != size-1)
      {
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_parent = obj_conn->getParent();
        obj_child = obj_conn->getChild();
        obj_pos = obj_parent->getConfiguration() + 0.8*(obj_child->getConfiguration()-obj_parent->getConfiguration());

        if((obj_pos-configuration_replan_).norm()<0.20 && ((obj_conn_pos+1)<replanner_->getCurrentPath()->getConnections().size()))
        {
          //ROS_WARN("Shifting the object..");
          obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos+1);
          obj_parent = obj_conn->getParent();
          obj_child = obj_conn->getChild();

          obj_pos = obj_parent->getConfiguration() + 0.8*(obj_child->getConfiguration()-obj_parent->getConfiguration());
        }
      }
      else
      {
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_parent = obj_conn->getParent();
        obj_pos = obj_parent->getConfiguration();
      }
      trj_mtx_.unlock();
      replanner_mtx_.unlock();

      moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);

      tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link_),obj.pose.pose);

      obj.pose.header.frame_id="world";

      srv_add_object.request.objects.clear();
      srv_add_object.request.objects.push_back(obj);

      scene_mtx_.lock();
      if (!add_obj_.call(srv_add_object))
      {
        ROS_ERROR("call to srv not ok");
      }
      if (!srv_add_object.response.success)
      {
        ROS_ERROR("srv error");
      }
      else
      {
        for (const std::string& str: srv_add_object.response.ids)
        {
          srv_remove_object.request.obj_ids.push_back(str);   //per rimuovere gli oggetti alla fine
        }
      }

      ROS_WARN("OBJECT SPAWNED");
      scene_mtx_.unlock();

      object_spawned = true;
    }

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }

  ros::Duration(1).sleep();

  scene_mtx_.lock();
  if (!remove_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }
  if (!remove_obj_.call(srv_remove_object))
  {
    ROS_ERROR("call to srv not ok");
  }
  if (!srv_remove_object.response.success)
  {
    ROS_ERROR("srv error");
  }
  scene_mtx_.unlock();
}

void ReplannerManagerBase::connectCurrentConfToTree()
{
  paths_mtx_.lock();
  PathPtr current_path_copy = current_path_shared_->clone();
  paths_mtx_.unlock();

  ROS_INFO_STREAM("ROOT IN CONNECT TO TREE: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());

  root_for_detach_ = replanner_->getReplannedPath()->getTree()->getRoot();

  std::vector<ConnectionPtr> new_branch;
  new_branch = replanner_->startReplannedTreeFromNewCurrentConf(current_configuration_,current_path_copy);

  ROS_INFO_STREAM("NEW ROOT IN CONNECT TO TREE: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());

  added_branch_.clear();  //removed before replanning
  if(!new_branch.empty())
    added_branch_ = new_branch;
}

bool ReplannerManagerBase::detachAddedBranch(std::vector<NodePtr>& nodes,
                                             std::vector<double>& costs)
{
  if(added_branch_.empty())
    return true;

  ROS_WARN("INIZIO DETACH");

  //Saving the branch
  nodes.clear();
  nodes.push_back(added_branch_.at(0)->getParent());
  ROS_INFO_STREAM("node: "<<added_branch_.at(0)->getParent()->getConfiguration().transpose());
  for(const ConnectionPtr& conn:added_branch_)
  {
    ROS_INFO_STREAM("node: "<<conn->getChild()->getConfiguration().transpose());
    nodes.push_back(conn->getChild());
    costs.push_back(conn->getCost());
  }

  //Removing the branch
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;
  NodePtr node_purge_from = added_branch_.back()->getParent(); //added_banch goes from current conf to replanned node

//  NodePtr new_root = added_branch_.back()->getChild();
  NodePtr new_root = root_for_detach_;
  root_for_attach_ = replanner_->getReplannedPath()->getTree()->getRoot();
  ROS_INFO_STREAM("new root: "<<*new_root);

  ROS_INFO_STREAM("ROOT IN DETACH: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());
  bool changed = replanner_->getReplannedPath()->getTree()->changeRoot(new_root); //the node through which the branch is attached to the tree
  ROS_INFO_STREAM("CHANGED "<<changed<< "\nNUOVA ROOT IN DETACH: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());

  bool purged = replanner_->getReplannedPath()->getTree()->purgeFromHere(node_purge_from,white_list,removed_nodes);

  ROS_WARN("FINE DETACH");

  return purged;

  //  return replanner_->getReplannedPath()->getTree()->purgeFromHere(node_purge_from,white_list,removed_nodes);
}

bool ReplannerManagerBase::attachAddedBranch(const std::vector<NodePtr>& nodes,
                                             const std::vector<double>& costs)
{
  if(added_branch_.empty())
    return true;

  ROS_WARN("INIZIO ATTACH");
  ROS_INFO_STREAM("ROOT IN ATTACH: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());

  //Re-build the branch
  added_branch_.clear();
  for(unsigned int i=0;i<costs.size();i++)
  {
    ConnectionPtr conn = std::make_shared<Connection>(nodes.at(i+1),nodes.at(i));
    conn->setCost(costs.at(i));
    conn->add();

    added_branch_.push_back(conn);
  }
  std::reverse(added_branch_.begin(),added_branch_.end());

  //Attach the branch
  bool attached = replanner_->getReplannedPath()->getTree()->addBranch(added_branch_);

  replanner_->getReplannedPath()->getTree()->changeRoot(root_for_attach_);
  std::reverse(added_branch_.begin(),added_branch_.end());

//  current_path_replanning_->setConnections()

  ROS_INFO_STREAM("NUOVA ROOT IN ATTACH: "<< *(replanner_->getReplannedPath()->getTree()->getRoot())<<"\n"<<replanner_->getReplannedPath()->getTree()->getRoot());

  ROS_INFO_STREAM("current path replanning size: "<<current_path_replanning_->getConnections().size()<<" current path shared size: "<<current_path_shared_->getConnections().size());

  for(const ConnectionPtr conn:current_path_replanning_->getConnections())
  {
    ROS_INFO_STREAM("r parent: "<<conn->getParent()->getConfiguration().transpose());
    ROS_INFO_STREAM("r child : "<<conn->getChild()->getConfiguration().transpose());
  }

  ROS_WARN("FINE ATTACH");

  return attached;

  //  return replanner_->getReplannedPath()->getTree()->addBranch(added_branch_);
}

}
