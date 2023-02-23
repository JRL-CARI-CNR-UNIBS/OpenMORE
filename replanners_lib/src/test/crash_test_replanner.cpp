#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanners_lib/trajectory.h>
#include <replanners_lib/replanners/MPRRT.h>
#include <replanners_lib/replanners/DRRTStar.h>
#include <replanners_lib/replanners/DRRT.h>
#include <replanners_lib/replanners/anytimeDRRT.h>
#include <replanners_lib/replanners/MARS.h>
#include <replanners_lib/replanners/MARSHA.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <ssm15066_estimators/parallel_ssm15066_estimator2D.h>
#include <length_penalty_metrics.h>
#include <graph_core/solvers/birrt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crash_test_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");

  //  ////////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  int n_iter = 1;
  nh.getParam("n_iter",n_iter);

  std::string replanner_type;
  if (!nh.getParam("replanner_type",replanner_type))
  {
    ROS_INFO("replanner_type not set");
    return 0;
  }

  double max_time;
  if (!nh.getParam("max_time",max_time))
  {
    ROS_INFO("max_time not set, use inf");
    max_time=std::numeric_limits<double>::infinity();
  }

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }

  std::vector<double> start_configuration;
  if (!nh.getParam("start_configuration",start_configuration))
  {
    ROS_ERROR("start_configuration not set, exit");
    return 0;
  }

  std::vector<double> stop_configuration;
  if (!nh.getParam("stop_configuration",stop_configuration))
  {
    ROS_ERROR("stop_configuration not set, exit");
    return 0;
  }

  double max_distance;
  if(!nh.getParam("max_distance",max_distance))
  {
    ROS_ERROR("max_distance not set, set 0.5");
    max_distance = 0.5;
  }

  bool display;
  if (!nh.getParam("display",display))
  {
    display = false;
  }

  bool verbosity;
  if (!nh.getParam("verbosity",verbosity))
  {
    verbosity = false;
  }

  int ssm_threads;
  std::string base_frame, tool_frame;
  std::vector<std::string> poi_names;
  double ssm_max_step_size, max_cart_acc, tr, min_distance, v_h;
  if(replanner_type == "MARSHA")
  {
    nh.getParam("MARSHA/base_frame",base_frame);
    nh.getParam("MARSHA/tool_frame",tool_frame);
    nh.getParam("MARSHA/ssm_threads",ssm_threads);
    nh.getParam("MARSHA/ssm_max_step_size",ssm_max_step_size);
    nh.getParam("MARSHA/max_cart_acc",max_cart_acc);
    nh.getParam("MARSHA/Tr",tr);
    nh.getParam("MARSHA/min_distance",min_distance);
    nh.getParam("MARSHA/v_h",v_h);
    nh.getParam("MARSHA/poi_names",poi_names);
  }

  //  ///////////////////////////////////UPLOADING THE ROBOT ARM/////////////////////////////////////////////////////////////
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
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

  //  /////////////////////////////////////UPDATING THE PLANNING STATIC SCENE////////////////////////////////////
  moveit_msgs::GetPlanningScene ps_srv;
  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 1;
  }

  // /////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::string last_link=planning_scene->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);

  ros::Duration(0.1).sleep();

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  Eigen::VectorXd delta = (goal_conf-start_conf)/(n_iter);
  delta[2] = 0.0;

  int id_start,id_goal;
  double distance;
  for(int i=0; i<n_iter; i++)
  {
    ROS_INFO("---------------------------------------------------------------------------------------------------------");
    distance = (goal_conf-start_conf).norm();
    ROS_INFO_STREAM("Iter n: "<<std::to_string(i)<<" start: "<<start_conf.transpose()<< " goal: "<<goal_conf.transpose()<< " distance: "<<distance);

    disp->clearMarker(id_start);
    disp->clearMarker(id_goal);

    disp->changeNodeSize();
    id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
    id_goal = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
    disp->defaultNodeSize();

    if(display)
      disp->nextButton();

    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }

    if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }

    pathplan::MetricsPtr metrics;

    Eigen::Vector3d grav; grav << 0, 0, -9.806;
    rosdyn::ChainPtr chain;
    ssm15066_estimator::SSM15066EstimatorPtr ssm;
    if(replanner_type == "MARSHA")
    {
      chain = rosdyn::createChain(*robot_model_loader.getURDF(),base_frame,tool_frame,grav);
      // ssm = std::make_shared<ssm15066_estimator::ParallelSSM15066Estimator2D>(chain,ssm_max_step_size,ssm_threads);
      ssm = std::make_shared<ssm15066_estimator::SSM15066Estimator2D>(chain,ssm_max_step_size);

      ssm->setHumanVelocity(v_h);
      ssm->setMaxCartAcc(max_cart_acc);
      ssm->setReactionTime(tr);
      ssm->setMinDistance(min_distance);
      ssm->setPoiNames(poi_names);

      metrics = std::make_shared<pathplan::LengthPenaltyMetrics>(ssm);
    }
    else
      metrics = std::make_shared<pathplan::Metrics>();

    pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
    pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics,checker,sampler);
    solver->setMaxDistance(max_distance);

    std::srand(std::time(NULL));
    pathplan::PathPtr current_path = trajectory.computePath(start_conf,goal_conf,solver,true);

    if(!current_path)
    {
      start_conf = start_conf+delta;
      goal_conf = goal_conf-delta;

      continue;
    }

    if(display)
      disp->displayPath(current_path,"pathplan",{0.0,1.0,0.0,1.0});

    std::vector<pathplan::PathPtr> all_paths;
    //    all_paths.push_back(current_path);

    std::srand(std::time(NULL));
    int n_conn = std::rand() % ((int) std::ceil(current_path->getConnections().size()/2.0));
    Eigen::VectorXd parent = current_path->getConnections().at(n_conn)->getParent()->getConfiguration();
    Eigen::VectorXd child = current_path->getConnections().at(n_conn)->getChild()->getConfiguration();

    Eigen::VectorXd current_configuration = parent + (child-parent)*0.1;
    ROS_INFO_STREAM("Current configuration: "<<current_configuration.transpose());

    if(display)
    {
      disp->changeNodeSize();
      disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),"pathplan",{1.0,0.5,0.5,1.0});
      disp->defaultNodeSize();

      disp->nextButton();
    }

    // //////////////////////////////////////////DEFINING THE REPLANNER//////////////////////////////////////////////
    pathplan::ReplannerBasePtr replanner = nullptr;
    if(replanner_type == "MPRRT")
    {
      replanner = std::make_shared<pathplan::MPRRT>(current_configuration,current_path,max_time,solver,5);
    }
    else if(replanner_type ==  "DRRT*")
    {
      replanner =  std::make_shared<pathplan::DynamicRRTStar>(current_configuration,current_path,max_time,solver);
    }
    else if(replanner_type == "DRRT")
    {
      replanner =  std::make_shared<pathplan::DynamicRRT>(current_configuration,current_path,max_time,solver);
    }
    else if(replanner_type == "anytimeDRRT")
    {
      replanner =  std::make_shared<pathplan::AnytimeDynamicRRT>(current_configuration,current_path,max_time,solver);
    }
    else if(replanner_type == "MARS" || replanner_type == "MARSHA")
    {
      int n_other_paths;
      if (!nh.getParam("/MARS/n_other_paths",n_other_paths))
      {
        ROS_ERROR("n_other_paths not set, set 1");
        n_other_paths = 1;
      }

      bool reverse;
      if (!nh.getParam("/MARS/reverse_start_nodes",reverse))
      {
        ROS_ERROR("reverse_start_nodes not set, set false");
        reverse = false;
      }
      bool full_search;
      if (!nh.getParam("/MARS/full_net_search",full_search))
      {
        ROS_ERROR("full_net_search not set, set true");
        full_search = true;
      }

      bool opt;
      if (!nh.getParam("/MARS/opt_paths",opt))
      {
        ROS_ERROR("opt_paths not set, set false");
        opt = false;
      }

      pathplan::PathPtr new_path;
      for(unsigned int i=0;i<n_other_paths;i++)
      {
        std::srand(std::time(NULL));
        pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
        pathplan::NodePtr goal_node  = std::make_shared<pathplan::Node>(goal_conf );

        solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
        //        new_path = trajectory.computePath(start_conf,goal_conf,solver,opt);

        if(solver->computePath(start_node,goal_node,nh,new_path))
        {
          all_paths.push_back(new_path);
          if(!new_path->getTree())
            assert(0);

          if(display)
            disp->displayPath(current_path,"pathplan",{0.0,0.0,1.0,1.0});
        }
      }

      if(replanner_type == "MARSHA")
      {
        pathplan::LengthPenaltyMetricsPtr ha_metrics = std::dynamic_pointer_cast<pathplan::LengthPenaltyMetrics>(metrics);

        pathplan::MARSHAPtr MARSHA_replanner = std::make_shared<pathplan::MARSHA>(current_configuration,current_path,max_time,solver,all_paths,ha_metrics);
        replanner = MARSHA_replanner;
      }
      else
      {
        pathplan::MARSPtr MARS_replanner = std::make_shared<pathplan::MARS>(current_configuration,current_path,max_time,solver);
        MARS_replanner->setOtherPaths(all_paths);
        MARS_replanner->reverseStartNodes(reverse);
        MARS_replanner->setFullNetSearch(full_search);

        replanner = MARS_replanner;
      }
    }
    else
    {
      ROS_ERROR("Replanner %s does not exist",replanner_type.c_str());
      return 0;
    }

    //      /////////////////////////////////////REPLANNING ////////////////////////////////////////////////////////
    replanner->setDisp(disp);
    replanner->setVerbosity(verbosity);

    if(replanner_type == "MARS" || replanner_type == "MARSHA")
    {
      if(verbosity)
      {
        int verbosity_level;
        if (!nh.getParam("/MARS/verbosity_level",verbosity_level))
        {
          ROS_ERROR("verbosity_level not set, set 0");
          verbosity_level = 0;
        }

        pathplan::MARSPtr mars = std::dynamic_pointer_cast<pathplan::MARS>(replanner);
        mars->setVerbosityLevel(verbosity_level);
      }
    }

    pathplan::MoveitUtils moveit_utils(planning_scene,group_name);
    if (!add_obj.waitForExistence(ros::Duration(10)))
    {
      ROS_FATAL("srv not found");
      return 1;
    }

    pathplan::PathPtr current_path_copy = current_path->clone();
    bool in_collision = false;
    bool success;
    for(int j=0;j<3;j++)
    {
      object_loader_msgs::AddObjects add_srv;
      object_loader_msgs::RemoveObjects remove_srv;
      object_loader_msgs::Object obj;
      obj.object_type="little_box";

      if(j!=2)
      {
        pathplan::PathPtr subpath = current_path_copy->getSubpathFromConf(current_configuration,true);
        std::vector<pathplan::ConnectionPtr> subpath_conns = subpath->getConnections();

        std::srand(std::time(NULL));
        int obj_conn_pos;
        if(subpath_conns.size() == 1)
          obj_conn_pos = 0;
        else
          obj_conn_pos = (std::rand() % ((int)std::ceil(subpath_conns.size()/1.7)));

        pathplan::ConnectionPtr obj_conn = subpath_conns.at(obj_conn_pos);
        pathplan::NodePtr obj_child = obj_conn->getChild();
        pathplan::NodePtr obj_parent = obj_conn->getParent();
        Eigen::VectorXd obj_pos = obj_parent->getConfiguration() + 0.75*(obj_child->getConfiguration()-obj_parent->getConfiguration());

        moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
        tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
        obj.pose.header.frame_id="world";

        add_srv.request.objects.push_back(obj);
        if (!add_obj.call(add_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }

        if (!add_srv.response.success)
        {
          ROS_ERROR("srv error");
          return 1;
        }
        else
        {
          if(ssm)
          {
            Eigen::Vector3d obs_location; obs_location<<obj.pose.pose.position.x,obj.pose.pose.position.y,obj.pose.pose.position.z;
            ssm->addObstaclePosition(obs_location);
          }

          remove_srv.request.obj_ids.clear();
          for (const std::string& str: add_srv.response.ids)
          {
            remove_srv.request.obj_ids.push_back(str);
          }

          ROS_BOLDMAGENTA_STREAM("Obstacle spawned");
        }

        ros::Duration(1).sleep();
      }
      //  /////////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////
      if (!ps_client.call(ps_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }

      if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
      {
        ROS_ERROR("unable to update planning scene");
        return 1;
      }

      checker->setPlanningSceneMsg(ps_srv.response.scene);

      in_collision = false;
      if(!checker->check(current_configuration))
      {
        start_conf = start_conf+delta;
        goal_conf = goal_conf-delta;

        if(j!=2)
        {
          if(!remove_obj.call(remove_srv))
          {
            ROS_ERROR("call to srv not ok");
            return 1;
          }
        }

        in_collision = true;
        ROS_INFO("Current configuration in collision!");
        break;
      }

      if(in_collision)
        continue;

      bool valid = current_path->isValid();
      if(j != 2 && valid)
        assert(0);

      ROS_INFO_STREAM("current path "<<*current_path);

      for(const pathplan::PathPtr& p:all_paths)
        p->isValid();

      // //////////////////////////////REPLANNING///////////////////////////////////////////////////
      replanner->setMaxTime(max_time);

      ros::WallTime tic = ros::WallTime::now();
      replanner->replan();
      success = replanner->getSuccess();
      ros::WallTime toc = ros::WallTime::now();

      if(success)
      {
        ROS_INFO_STREAM("- - - - Cycle n: "<<std::to_string(j)<<" success: "<<success<<" duration: "<<(toc-tic).toSec()<<" cost: "<<replanner->getReplannedPath()->cost()<<" - - - - ");
        current_path = replanner->getReplannedPath();
        replanner->setCurrentPath(current_path);
        ROS_INFO_STREAM("Path found "<<*current_path);
        current_path_copy = current_path->clone();

        current_configuration = current_path->getWaypoints().front();
        replanner->setCurrentConf(current_configuration);

        if(display)
          disp->displayPathAndWaypoints(current_path,"pathplan",{1.0,1.0,0.0,1.0});

      }
      else
      {
        ROS_INFO_STREAM("- - - - Cycle n: "<<std::to_string(j)<<" success: "<<success<<" duration: "<<(toc-tic).toSec()<<" - - - - ");
      }

      if(ssm)
      {
        ssm->setVerbose(1);
        for(const pathplan::ConnectionPtr& c:current_path->getConnections())
        {
          ROS_INFO("--------");
          c->setCost(metrics->cost(c->getParent(),c->getChild()));
          ROS_INFO_STREAM(*c);
        }
        ssm->setVerbose(0);
      }

      ROS_INFO_STREAM("Path cost "<< current_path->cost());

      if(success && not current_path->isValid())
        throw std::runtime_error("not valid");

      if(display)
      {
        disp->nextButton();
        disp->clearMarkers();
      }

      if(j != 2)
      {
        if(!remove_obj.call(remove_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }

        if(ssm)
          ssm->clearObstaclesPositions();

        ros::Duration(1).sleep();
      }
    }

    start_conf = start_conf+delta;
    goal_conf = goal_conf-delta;
  }

  return 0;
}

