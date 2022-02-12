#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include<replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h>
#include<replanners_lib/replanner_managers/replanner_manager_DRRTStar.h>
#include<replanners_lib/replanner_managers/replanner_manager_to_goal.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "crash_test_replanner_manager");
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

    std::vector<pathplan::PathPtr> all_paths;
    all_paths.push_back(current_path);

    // //////////////////////////////////////////DEFINING THE REPLANNER//////////////////////////////////////////////
    pathplan::ReplannerManagerBasePtr replanner_manager = nullptr;
    if(replanner_type == "replanner_to_goal")
    {
      replanner_manager = std::make_shared<pathplan::ReplannerManagerToGoal>(current_path,solver,nh);
    }
    else if(replanner_type ==  "DRRT*")
    {
      replanner_manager =  std::make_shared<pathplan::ReplannerManagerDRRTStar>(current_path,solver,nh);
    }
    else if(replanner_type == "DRRT")
    {
      replanner_manager =  std::make_shared<pathplan::ReplannerManagerDRRT>(current_path,solver,nh);
    }
    else if(replanner_type == "anytimeDRRT")
    {
      replanner_manager =  std::make_shared<pathplan::ReplannerManagerAnytimeDRRT>(current_path,solver,nh);
    }
    else
    {
      ROS_ERROR("Replanner manager %s does not exist",replanner_type.c_str());
      return 0;
    }

    // //////////////////////////////REPLANNING///////////////////////////////////////////////////
    replanner_manager->start();

    start_conf = start_conf+delta;
    goal_conf = goal_conf-delta;
  }

  return 0;
}

