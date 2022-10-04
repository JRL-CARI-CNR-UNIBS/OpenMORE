#include<jsk_rviz_plugins/OverlayText.h>
#include<graph_core/solvers/birrt.h>
#include<replanners_lib/replanner_managers/replanner_manager_DRRT.h>
#include<replanners_lib/replanner_managers/replanner_manager_MARS.h>
#include<replanners_lib/replanner_managers/replanner_manager_MPRRT.h>
#include<replanners_lib/replanner_managers/replanner_manager_DRRTStar.h>
#include<replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quick_example");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ros::Duration(5).sleep();

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::Publisher text_overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz_text_overlay",1);

  //  ////////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  int n_query = 1;
  nh.getParam("n_query",n_query);

  int n_iter_per_query = 1;
  nh.getParam("n_iter_per_query",n_iter_per_query);

  std::vector<std::string> replanner_type_vector;
  nh.getParam("replanner_type_vector",replanner_type_vector);

  if(replanner_type_vector.empty())
  {
    ROS_INFO("replanner_type_vector not set");
    return 0;
  }

  std::string bench_name;
  if (!nh.getParam("bench_name",bench_name))
  {
    ROS_INFO("bench_name not set");
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

  std::vector<double> end_start_configuration;
  if (!nh.getParam("end_start_configuration",end_start_configuration))
  {
    end_start_configuration = stop_configuration;
  }

  std::vector<double> end_stop_configuration;
  if (!nh.getParam("end_stop_configuration",end_stop_configuration))
  {
    end_stop_configuration = start_configuration;
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

  double max_solver_time;
  if (!nh.getParam("max_solver_time",max_solver_time))
  {
    max_solver_time = 20;
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

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  ros::Duration(0.1).sleep();

  Eigen::VectorXd init_start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd init_goal_conf  = Eigen::Map<Eigen::VectorXd>(stop_configuration .data(), stop_configuration .size());

  Eigen::VectorXd end_start_conf = Eigen::Map<Eigen::VectorXd>(end_start_configuration.data(), end_start_configuration.size());
  Eigen::VectorXd end_goal_conf  = Eigen::Map<Eigen::VectorXd>(end_stop_configuration .data(), end_stop_configuration .size());

  Eigen::VectorXd delta_start = (end_start_conf - init_start_conf)/(std::max(n_query-1,1));
  Eigen::VectorXd delta_goal  = (end_goal_conf  - init_goal_conf )/(std::max(n_query-1,1));

  Eigen::VectorXd start_conf, goal_conf;

  std_msgs::ColorRGBA fg_color, bg_color;
  fg_color.r = 0;   bg_color.r = 0;
  fg_color.g = 0;   bg_color.g = 0;
  fg_color.b = 1;   bg_color.b = 0;
  fg_color.a = 0.8; bg_color.a = 0;

  jsk_rviz_plugins::OverlayText overlayed_text;
  overlayed_text.font = "FreeSans";
  overlayed_text.bg_color = bg_color;
  overlayed_text.fg_color = fg_color;
  overlayed_text.height = 70;
  overlayed_text.left = 10;
  overlayed_text.top = 10;
  overlayed_text.width = 1000;
  overlayed_text.line_width = 2;
  overlayed_text.text_size = 20;

  for(const std::string replanner_type:replanner_type_vector)
  {
    start_conf = init_start_conf;
    goal_conf  = init_goal_conf;

    nh.setParam("replanner_type",replanner_type);

    pathplan::MetricsPtr metrics;
    pathplan::CollisionCheckerPtr checker;
    pathplan::SamplerPtr sampler;
    pathplan::RRTPtr solver;
    pathplan::PathPtr current_path, new_path;
    std::vector<pathplan::PathPtr> other_paths;
    pathplan::ReplannerManagerBasePtr replanner_manager;
    pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(nh,planning_scene,group_name);

    int id_start,id_goal;
    disp->changeNodeSize();
    id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
    id_goal = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
    disp->defaultNodeSize();

    double distance;
    for(int i=0; i<n_query; i++)
    {
      distance = (goal_conf-start_conf).norm();

      disp->clearMarker(id_start);
      disp->clearMarker(id_goal);

      disp->changeNodeSize();
      id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
      id_goal = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
      disp->defaultNodeSize();

      for(int j=0;j<n_iter_per_query;j++)
      {
        ROS_INFO("---------------------------------------------------------------------------------------------------------");
        ROS_INFO_STREAM(replanner_type<<": query: "<<std::to_string(i)<<" Iter: "<<std::to_string(j)<<" start: "<<start_conf.transpose()<< " goal: "<<goal_conf.transpose()<< " distance: "<<distance);

        overlayed_text.text = "Replanner: "+replanner_type+"\nQuery: "+std::to_string(i)+"/"+std::to_string(n_query-1)+", iter: "+std::to_string(j)+"/"+std::to_string(n_iter_per_query-1);
        text_overlay_pub.publish(overlayed_text);

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

        metrics = std::make_shared<pathplan::Metrics>();
        checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
        sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
        solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
        solver->setMaxDistance(max_distance);

        std::srand(std::time(NULL));
        current_path = trajectory->computePath(start_conf,goal_conf,solver,true,max_solver_time);

        if(not current_path)
          continue;
        else
          ROS_INFO_STREAM("current path cost "<<current_path->cost());

        // //////////////////////////////////////////DEFINING THE REPLANNER//////////////////////////////////////////////
        replanner_manager.reset();
        if(replanner_type == "MPRRT")
        {
          replanner_manager.reset(new pathplan::ReplannerManagerMPRRT(current_path,solver,nh));
        }
        else if(replanner_type ==  "DRRTStar")
        {
          replanner_manager.reset(new pathplan::ReplannerManagerDRRTStar(current_path,solver,nh));
        }
        else if(replanner_type == "DRRT")
        {
          replanner_manager.reset(new pathplan::ReplannerManagerDRRT(current_path,solver,nh));
        }
        else if(replanner_type == "anytimeDRRT")
        {
          replanner_manager.reset(new pathplan::ReplannerManagerAnytimeDRRT(current_path,solver,nh));
        }
        else if(replanner_type == "MARS")
        {
          int n_other_paths;
          if (!nh.getParam("/MARS/n_other_paths",n_other_paths))
          {
            ROS_ERROR("n_other_paths not set, set 1");
            n_other_paths = 1;
          }

          for(unsigned int i=0;i<n_other_paths;i++)
          {
            std::srand(std::time(NULL));
            solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
            new_path = trajectory->computePath(start_conf,goal_conf,solver,true,max_solver_time);

            other_paths.clear();
            if(new_path)
            {
              other_paths.push_back(new_path);
              ROS_INFO_STREAM("other path cost "<<new_path->cost());
              assert(new_path->getTree());
            }
            else
              ROS_INFO("other path not found");
          }

          std::srand(std::time(NULL));
          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
          solver->config(nh);
          replanner_manager = std::make_shared<pathplan::ReplannerManagerMARS>(current_path,solver,nh,other_paths);
        }
        else
        {
          ROS_ERROR("Replanner manager %s does not exist",replanner_type.c_str());
          return 0;
        }

        // //////////////////////////////REPLANNING///////////////////////////////////////////////////
        replanner_manager->start();
      }

      start_conf = start_conf+delta_start;
      goal_conf  = goal_conf +delta_goal ;
    }
  }

  return 0;
}
