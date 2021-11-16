#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <graph_replanning/replanner.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string group_name = "cartesian_arm";
  std::string last_link = "end_effector";
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
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  // //////////////////////////////////////////UPDATING PLANNING SCENE////////////////////////////////////////////////////////
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  moveit_msgs::GetPlanningScene ps_srv;

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 0;
  }

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();
  ros::Duration(1).sleep();
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  std::vector<pathplan::PathPtr> path_vector;
  Eigen::VectorXd start_conf(3);
  start_conf << 0.0,0.0,0.0;
  Eigen::VectorXd goal_conf(3);
  goal_conf << 0.8,0.8,0.8;

  int id=100;
  int id_wp = 1000;
  for (unsigned int i =0; i<1; i++)
  {
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
    pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,1);
    path_vector.push_back(solution);

    std::vector<double> marker_color;
    if(i==0) marker_color = {0.5,0.5,0.0,1.0};
    if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};

    disp->displayPathAndWaypoints(solution,id,id_wp,"pathplan",marker_color);
    id +=1;
    id_wp +=50;

    ros::Duration(0.5).sleep();
  }

  pathplan::PathPtr current_path = path_vector.front();

  Eigen::VectorXd parent = current_path->getConnections().at(1)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(1)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration = parent + (child-parent)*0.5;

  //    /////////////////////////////////////////// VISUALIZATION OF CURRENT NODE ////////////////////////
  disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),5000,"pathplan",{1.0,0.0,1.0,1.0});

  pathplan::NodePtr node2add = current_path->addNodeAtCurrentConfig(current_configuration,current_path->getConnections().at(1),true);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path, id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  current_path->removeNodeAddedInConn(node2add);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path, id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  node2add = current_path->addNodeAtCurrentConfig(child,current_path->getConnections().at(1),true);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path, id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  current_path->removeNodeAddedInConn(node2add);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path,id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  int idx_con;
  current_configuration = parent+0.99999*(child-parent);
  pathplan::ConnectionPtr conn2add = current_path->findConnection(current_configuration,idx_con);

  ROS_INFO_STREAM("Conn: "<<idx_con);

  node2add = current_path->addNodeAtCurrentConfig(current_configuration,conn2add,true);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path, id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  current_path->removeNodeAddedInConn(node2add);

  disp->nextButton();
  disp->clearMarkers();
  ros::Duration(1).sleep();

  disp->displayPathAndWaypoints(current_path,id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});


  return 0;
}

