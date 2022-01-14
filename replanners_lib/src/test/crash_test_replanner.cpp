#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanners_lib/replanners/replanner_to_goal.h>
#include <replanners_lib/replanners/DRRTStar.h>
#include <replanners_lib/replanners/DRRT.h>
#include <replanners_lib/replanners/anytimeDRRT.h>
#include <replanners_lib/replanners/AIPRO.h>

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

  bool display;
  if (!nh.getParam("display",display))
  {
    display = false;
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

  ros::Duration(1.0).sleep();

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  Eigen::VectorXd delta = (goal_conf-start_conf)/(n_iter);
  delta[2] = 0.0;

  double distance;
  for(int i=0; i<n_iter; i++)
  {
    ROS_INFO("---------------------------------------------------------------------------------------------------------");
    distance = (goal_conf-start_conf).norm();
    ROS_INFO_STREAM("Iter n: "<<std::to_string(i)<<" start: "<<start_conf.transpose()<< " goal: "<<goal_conf.transpose()<< " distance: "<<distance);

    if(display)
    {
      disp->changeNodeSize();
      disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
      disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
      disp->defaultNodeSize();

      disp->nextButton();
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

    pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
    pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
    pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics,checker,sampler);

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
    all_paths.push_back(current_path);

    std::srand(std::time(NULL));
    int n_conn = std::rand() % (current_path->getConnections().size());
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

    // //////////////////////////////////////////0.794DEFINING THE REPLANNER//////////////////////////////////////////////
    pathplan::ReplannerBasePtr replanner = NULL;
    if(replanner_type == "replanner_to_goal")
    {
      replanner = std::make_shared<pathplan::ReplannerToGoal>(current_configuration,current_path,max_time,solver,5);
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
    else if(replanner_type == "AIPRO")
    {
      int n_other_paths;
      if (!nh.getParam("/aipro/n_other_paths",n_other_paths))
      {
        ROS_ERROR("n_other_paths not set, set 1");
        n_other_paths = 1;
      }

      std::vector<pathplan::PathPtr> other_paths;
      for(unsigned int i=0;i<n_other_paths;i++)
      {
        solver->resetProblem();
        pathplan::PathPtr path = trajectory.computePath(start_conf,goal_conf,solver,false);

        if(path)
        {
          other_paths.push_back(path);
          if(!path->getTree())
            assert(0);
        }
      }

      all_paths.insert(all_paths.end(),other_paths.begin(),other_paths.end());

      pathplan::AIPROPtr aipro_replanner =  std::make_shared<pathplan::AIPRO>(current_configuration,current_path,max_time,solver);
      aipro_replanner->setOtherPaths(other_paths);

      replanner = aipro_replanner;
    }
    else
    {
      ROS_ERROR("Replanner %s does not exist",replanner_type.c_str());
      return 0;
    }

    //    // ///////////////////////////ADDING THE OBSTACLE ////////////////////////////////////////////////
    pathplan::MoveitUtils moveit_utils(planning_scene,group_name);
    if (!add_obj.waitForExistence(ros::Duration(10)))
    {
      ROS_FATAL("srv not found");
      return 1;
    }

    object_loader_msgs::AddObjects add_srv;
    object_loader_msgs::RemoveObjects remove_srv;
    object_loader_msgs::Object obj;
    obj.object_type="scatola";

    std::srand(std::time(NULL));
    int obj_conn_pos;
    if(n_conn == (current_path->getConnections().size()-1))
      obj_conn_pos = n_conn;
    else
      obj_conn_pos = (std::rand() % (current_path->getConnections().size()-n_conn))+n_conn;

    pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
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
      remove_srv.request.obj_ids.clear();
      for (const std::string& str: add_srv.response.ids)
      {
        remove_srv.request.obj_ids.push_back(str);
      }
    }

    ros::Duration(1).sleep();

    //      /////////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////
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

    if(!checker->check(current_configuration))
    {
      start_conf = start_conf+delta;
      goal_conf = goal_conf-delta;
      continue;
    }

    bool valid = current_path->isValid();
    ROS_INFO_STREAM("Path valid: "<<valid);

    //      /////////////////////////////////////REPLANNING ////////////////////////////////////////////////////////
    bool success;
    for(int j=0;j<2;j++)
    {
      ros::WallTime tic = ros::WallTime::now();
      success =  replanner->replan();
      ros::WallTime toc = ros::WallTime::now();

      if(success)
      {
        ROS_INFO_STREAM("-Cycle n: "<<std::to_string(j)<<" success: "<<success<<" duration: "<<(toc-tic).toSec()<<" cost: "<<replanner->getReplannedPath()->cost());

        current_path = replanner->getReplannedPath();
        replanner->setCurrentPath(current_path);

        current_configuration = current_path->getWaypoints().front();
        replanner->setCurrentConf(current_configuration);

        if(display)
          disp->displayPath(current_path,"pathplan",{1.0,1.0,0.0,1.0});
      }
      else
      {
        pathplan::PathPtr subpath1 = current_path->getSubpathFromConf(current_configuration,true);

        ROS_INFO_STREAM("-Cycle n: "<<std::to_string(j)<<" success: "<<success<<" duration: "<<(toc-tic).toSec()<< " subpath1 cost: "<<subpath1->cost()<<" subpath1 norm: "<<subpath1->getNormFromConf(current_configuration));
        break;
      }

      if(j == 0)
      {
        if(display)
          disp->nextButton();

        if (!remove_obj.call(remove_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }
      }
    }

    if(display)
    {
      disp->nextButton();
      disp->clearMarkers();
    }

    start_conf = start_conf+delta;
    goal_conf = goal_conf-delta;
  }

  return 0;
}

