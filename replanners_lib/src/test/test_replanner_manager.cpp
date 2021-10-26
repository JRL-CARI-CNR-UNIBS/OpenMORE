#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <replanners_lib/replanner_managers/replanner_manager_to_goal.h>
#include <replanners_lib/replanner_managers/replanner_manager_DRRTStar.h>
#include <replanners_lib/replanner_managers/replanner_manager_DRRT.h>
#include <replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_replanning_strategies");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  //  ////////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////

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

  std::string base_link;
  if (!nh.getParam("base_link",base_link))
  {
    ROS_ERROR("base_link not set, exit");
    return 0;
  }

  std::string last_link;
  if (!nh.getParam("last_link",last_link))
  {
    ROS_ERROR("last_link not set, exit");
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

  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler);

  pathplan::PathPtr current_path = trajectory.computePath(start_conf,goal_conf,solver,true);

  //    ////////////////////////////////////////// REPLAN ////////////////////////////////////////////////////////////////

  pathplan::ReplannerManagerBasePtr replanner_manager = NULL;

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
    ROS_ERROR("Replanner %s does not exist",replanner_type.c_str());
    return 0;
  }

  replanner_manager->start();

  return 0;
}

