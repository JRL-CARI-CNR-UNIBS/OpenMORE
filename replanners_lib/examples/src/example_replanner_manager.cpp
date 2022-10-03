#include <ros/ros.h>
#include <replanners_lib/replanner_managers/replanner_manager_DRRT.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_replanning_strategies");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  /* GET ROS PARAM */

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

  /* UPLOAD ROBOT DESCRIPTION */

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

  /* UPDATE PLANNING STATIC SCENE */

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene ps_srv;

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 0;
  }

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 0;
  }


  /* PATH PLAN & VISUALIZATION */

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler);

  pathplan::PathPtr current_path = trajectory.computePath(start_conf,goal_conf,solver,true); //optimize = true to optimize with RRT* rewire and shortcutting

  /* REPLAN */

  pathplan::ReplannerManagerBasePtr replanner_manager =  std::make_shared<pathplan::ReplannerManagerDRRT>(current_path,solver,nh);
  replanner_manager->start();

  return 0;
}

