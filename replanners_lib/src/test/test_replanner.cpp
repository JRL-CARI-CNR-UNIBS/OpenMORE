#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanners_lib/trajectory.h>
#include <replanners_lib/replanners/MPRRT.h>
#include <replanners_lib/replanners/DRRTStar.h>
#include <replanners_lib/replanners/DRRT.h>
#include <replanners_lib/replanners/anytimeDRRT.h>
#include <replanners_lib/replanners/AIPRO.h>
#include <graph_core/parallel_moveit_collision_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_replanning_strategies");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

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
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
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

  //  ////////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  std::string last_link=planning_scene->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();

  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler);
  //  pathplan::AnytimeRRTPtr solver = std::make_shared<pathplan::AnytimeRRT>(metrics, checker, sampler);

  pathplan::PathPtr current_path = trajectory.computePath(start_conf,goal_conf,solver,true);

  disp->displayPathAndWaypoints(current_path,1,1000,"pathplan",{0.5,0.5,0.0,1.0});

  std::vector<pathplan::PathPtr> all_paths;
  all_paths.push_back(current_path);

  int n_conn = 1;
  Eigen::VectorXd parent = current_path->getConnections().at(n_conn)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(n_conn)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration = parent + (child-parent)*0.1;

  //    ////////////////////////////////////////// REPLAN ////////////////////////////////////////////////////////////////
  bool success = false;
  ros::WallTime tic;
  ros::WallTime toc;

  pathplan::ReplannerBasePtr replanner = NULL;

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
  else if(replanner_type == "AIPRO")
  {
    int n_other_paths;
    if (!nh.getParam("/aipro/n_other_paths",n_other_paths))
    {
      ROS_ERROR("n_other_paths not set, set 1");
      n_other_paths = 1;
    }

    bool verbose,display;
    if(!nh.getParam("/aipro/verbose",verbose))
    {
      return 0;
      verbose = false;
    }
    if(!nh.getParam("/aipro/display",display))
    {
      return 0;
      display = false;
    }

    int id_path = 500;
    int id_wp = 20000;
    std::vector<pathplan::PathPtr> other_paths;
    for(unsigned int i=0;i<n_other_paths;i++)
    {
      ROS_INFO_STREAM("Computing path number: "<<i+1);
      ROS_INFO_STREAM("MAX DIST: "<<solver->getMaxDistance());
      pathplan::PathPtr path = trajectory.computePath(start_conf,goal_conf,solver,false);
      disp->displayPathAndWaypoints(path,id_path,id_wp,"pathplan",{0.0,0.0,1.0,1.0});

      id_path += 50;
      id_wp += 1000;

      if(path)
      {
        other_paths.push_back(path);
        if(!path->getTree())
          assert(0);
      }
    }

    all_paths.insert(all_paths.end(),other_paths.begin(),other_paths.end());

    pathplan::AIPROPtr aipro_replanner =  std::make_shared<pathplan::AIPRO>(current_configuration,current_path,max_time,solver);
    aipro_replanner->setDisp(disp);

    aipro_replanner->setInformedOnlineReplanningDisp(display);
    aipro_replanner->setPathSwitchDisp(display);
    aipro_replanner->setVerbosity(verbose);

    aipro_replanner->setOtherPaths(other_paths);

    replanner = aipro_replanner;
  }
  else
  {
    ROS_ERROR("Replanner %s does not exist",replanner_type.c_str());
    return 0;
  }

  if(!replanner->getDisp())
    replanner->setDisp(disp);

  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");
  pathplan::MoveitUtils moveit_utils(planning_scene,group_name);

  object_loader_msgs::AddObjects add_srv;
  object_loader_msgs::RemoveObjects remove_srv;
  for(unsigned int i=0; i<(unsigned int)n_iter; i++)
  {
    //    /////////////////////////////////////////// VISUALIZATION OF CURRENT NODE ////////////////////////
    disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),5000,"pathplan",{1.0,0.0,1.0,1.0});

    //    // ///////////////////////////ADDING THE OBSTACLE ////////////////////////////////////////////////
    if (!add_obj.waitForExistence(ros::Duration(10)))
    {
      ROS_FATAL("srv not found");
      return 1;
    }

    object_loader_msgs::Object obj;
    obj.object_type="scatola";

    int obj_conn_pos = n_conn+1;
    pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
    pathplan::NodePtr obj_child = obj_conn->getChild();
    pathplan::NodePtr obj_parent = obj_conn->getParent();
    Eigen::VectorXd obj_pos = obj_parent->getConfiguration() + 0.75*(obj_child->getConfiguration()-obj_parent->getConfiguration());

    moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
    tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
    obj.pose.header.frame_id="world";

    if((i%2) == 0)
    {
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
    }
    else
      if (!remove_obj.call(remove_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }

    //      /////////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////
    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }

    checker->setPlanningSceneMsg(ps_srv.response.scene);

    //    /////////////////////////////////////////////////////PATH CHECKING & REPLANNING////////////////////////////////////
    bool valid;
    valid =current_path->isValid();
    ROS_INFO_STREAM("current path valid: "<<valid<<", cost: "<<current_path->cost());

    disp->nextButton();

    //      /////////////////////////////////////REPLANNING ////////////////////////////////////////
    tic = ros::WallTime::now();
    success =  replanner->replan();
    toc = ros::WallTime::now();
    if((i%2 != 0))
    {
      if (!remove_obj.call(remove_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }
    }
    ROS_INFO_STREAM("Replanner -> "<<replanner_type<<" Duration: "<<(toc-tic).toSec()<<" success: "<<success);

    if(success)
    {
      disp->nextButton("----------------------------------------------------------------");
      disp->clearMarkers();

      int id_path = 500;
      int id_wp = 2000;
      std::vector<double> marker_color;
      for(unsigned int ii=0;ii<all_paths.size();ii++)
      {
        pathplan::PathPtr p = all_paths.at(ii);

        ii==0? marker_color = {0.5,0.5,0.0,1.0} : marker_color = {1.0,0.0,0.0,1.0};

        disp->displayPathAndWaypoints(p,id_path,id_wp,"pathplan",{0.0,0.0,1.0,1.0});

        id_path += 50;
        id_wp += 1000;
      }

      marker_color = {1.0,1.0,0.0,1.0};

      std::vector<double> marker_scale(3,0.01);
      disp->changeConnectionSize(marker_scale);
      disp->displayPath(replanner->getReplannedPath(),6000,"pathplan",marker_color);

      valid =replanner->getReplannedPath()->isValid();
      ROS_INFO_STREAM("replanned path valid: "<<valid);
    }
    else
      break;

    current_path = replanner->getReplannedPath();

//    Eigen::VectorXd parent = current_path->getConnections().at(n_conn)->getParent()->getConfiguration();
//    Eigen::VectorXd child = current_path->getConnections().at(n_conn)->getChild()->getConfiguration();
//    current_configuration = parent + (child-parent)*0.1;

    replanner->setCurrentConf(current_configuration);
    replanner->setCurrentPath(current_path);
  }

  disp->nextButton("Display final tree");
  disp->clearMarkers();
  disp->displayTree(current_path->getTree());

  return 0;
}

