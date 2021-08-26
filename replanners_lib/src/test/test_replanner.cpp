#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanner_to_goal.h>
#include <DRRTstar.h>

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
    return false;
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

  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");
  object_loader_msgs::AddObjects add_srv;
  object_loader_msgs::RemoveObjects remove_srv;

  //  // ///////////////////////////////////UPDATING THE PLANNING STATIC SCENE////////////////////////////////////
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

  //  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();
  ros::Duration(1).sleep();

  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
  pathplan::PathPtr current_path = trajectory.computePath(start_conf, goal_conf,solver,1);

  disp->displayPathAndWaypoints(current_path,1,1000,"pathplan",{0.5,0.5,0.0,1.0});

  int n_conn = 1;
  Eigen::VectorXd parent = current_path->getConnections().at(n_conn)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(n_conn)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration = parent + (child-parent)*0.5;

  //    // ///////////////////////////////////////////////////////////////////////////

  if (!add_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
    return 1;
  }

  object_loader_msgs::AddObjects srv;
  object_loader_msgs::Object obj;
  obj.object_type="scatola";

  int obj_conn_pos = n_conn+1;
  pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
  pathplan::NodePtr obj_parent = obj_conn->getParent();
  pathplan::NodePtr obj_child = obj_conn->getChild();
  Eigen::VectorXd obj_pos = (obj_child->getConfiguration()+obj_parent->getConfiguration())/2;

  pathplan::MoveitUtils moveit_utils(planning_scene,group_name);
  moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
  tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
  obj.pose.header.frame_id="world";

  srv.request.objects.push_back(obj);
  if (!add_obj.call(srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }
  if (!srv.response.success)
  {
    ROS_ERROR("srv error");
    return 1;
  }
  else
  {
    remove_srv.request.obj_ids.clear();
    for (const std::string& str: srv.response.ids)
    {
      remove_srv.request.obj_ids.push_back(str);
    }
  }
  //      // ///////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  checker->setPlanningSceneMsg(ps_srv.response.scene);


  //    // ///////////////////////////////////////////////////PATH CHECKING & REPLANNING/////////////////////////////////////////////////////

  bool valid;
  valid =current_path->isValid();
  ROS_INFO_STREAM("current path valid: "<<valid);

  //    // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
  std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
  disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),5000,"pathplan",marker_color_sphere_actual);
  //    // //////////////////////////////////////// ADDING A MOBILE OBSTACLE ////////////////////////////////////////////////////////////////

  bool success = false;
  ros::WallTime tic;
  ros::WallTime toc;

  pathplan::ReplannerBasePtr replanner;

  if(replanner_type == "replanner_to_goal")
  {
    replanner = std::make_shared<pathplan::ReplannerToGoal>(current_configuration,current_path,max_time,solver);

    tic = ros::WallTime::now();
    success =  replanner->replan();
    toc = ros::WallTime::now();

  }
  else if(replanner_type ==  "DRRT*")
  {
    replanner =  std::make_shared<pathplan::DynamicRRTstar>(current_configuration,current_path,max_time,solver);

    tic = ros::WallTime::now();
    success =  replanner->replan();
    toc = ros::WallTime::now();
  }
  else
  {
    ROS_ERROR("Replanner %s does not exist",replanner_type.c_str());
  }

  if((toc-tic).toSec()>max_time) ROS_ERROR("TIME OUT");
  ROS_INFO_STREAM("Replanner->"<<replanner_type<<" Duration: "<<(toc-tic).toSec()<<" success: "<<success);

  if(success)
  {
    std::vector<double> marker_color;
    marker_color = {1.0,1.0,0.0,1.0};

    std::vector<double> marker_scale(3,0.01);
    disp->changeConnectionSize(marker_scale);
    disp->displayPath(replanner->getReplannedPath(),6000,"pathplan",marker_color);
  }

  return 0;
}

