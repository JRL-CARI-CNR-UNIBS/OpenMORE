#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanners_lib/trajectory.h>
#include <replanners_lib/replanners/DRRT.h>
#include <graph_core/parallel_moveit_collision_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_example_test_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  /* GET ROS PARAM */

  int n_iter = 1;
  nh.getParam("n_iter",n_iter);

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

  bool verbose;
  if (!nh.getParam("verbose",verbose))
  {
    ROS_ERROR("verbose not set, exit");
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

  /* UPDATE PLANNING SCENE */

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

  std::string last_link=planning_scene->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler);
  pathplan::PathPtr current_path = trajectory.computePath(start_conf,goal_conf,solver,true); //optimize = true to optimize with RRT* rewire and shortcutting

  disp->displayPathAndWaypoints(current_path,1,1000,"pathplan",{0,1,0.0,1.0});

  double abscissa = 0.0;
  std::srand(std::time(NULL));

  do
  {
    abscissa = double(rand())/double(RAND_MAX);
  }while(abscissa>0.8);

  Eigen::VectorXd current_configuration = current_path->pointOnCurvilinearAbscissa(abscissa);
  disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),5000,"pathplan",{1.0,0.0,1.0,1.0});

  /* ADD A NEW OBSTACLE ON CURRENT PATH */

  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  pathplan::MoveitUtils moveit_utils(planning_scene,group_name);

  object_loader_msgs::AddObjects add_srv;
  if (!add_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
    return 0;
  }

  object_loader_msgs::Object obj;
  obj.object_type="red_box";

  double obs_abscissa = 0.0;
  do
  {
    obs_abscissa = double(rand())/double(RAND_MAX);
  }while(obs_abscissa<(abscissa+0.2) || obs_abscissa>0.8);

  Eigen::VectorXd obj_pos = current_path->pointOnCurvilinearAbscissa(obs_abscissa);

  moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
  tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
  obj.pose.header.frame_id="world";

  add_srv.request.objects.push_back(obj);
  if (!add_obj.call(add_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
  }
  if (!add_srv.response.success)
  {
    ROS_ERROR("srv error");
    return 0;
  }

  /* UPDATE PLANNING SCENE */

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
  }

  checker->setPlanningSceneMsg(ps_srv.response.scene);

  /* PATH CHECK & REPLAN */

  bool valid;
  valid =current_path->isValid();
  ROS_INFO_STREAM("current path valid: "<<valid<<", cost: "<<current_path->cost());

  int seconds = 10;
  ros::Rate lr(1);
  while(seconds>0)
  {
    ROS_BOLDWHITE_STREAM("Replanning will take place in "<<seconds<<" s");
    seconds = seconds-1;

    lr.sleep();
  }

  bool success = false;
  ros::WallTime tic;
  ros::WallTime toc;

  pathplan::ReplannerBasePtr replanner =  std::make_shared<pathplan::DynamicRRT>(current_configuration,current_path,max_time,solver);
  if(verbose)
    replanner->setVerbosity(true);

  tic = ros::WallTime::now();
  replanner->replan();
  toc = ros::WallTime::now();
  success = replanner->getSuccess();

  ROS_INFO_STREAM("Replanning duration: "<<(toc-tic).toSec()<<", success: "<<success);

  if(success)
    disp->displayPath(replanner->getReplannedPath(),6000,"pathplan",{0.5,0.5,0.0,1.0});

  return 0;
}

