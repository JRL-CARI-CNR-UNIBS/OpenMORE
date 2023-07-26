#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h>
#include <replanners_lib/replanner_managers/replanner_manager_DRRTStar.h>
#include <replanners_lib/replanner_managers/replanner_manager_MPRRT.h>
#include <replanners_lib/replanner_managers/replanner_manager_MARS.h>
#include <replanners_lib/replanner_managers/replanner_manager_MARSHA.h>
#include <graph_core/solvers/birrt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crash_test_replanner_manager");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  //  ////////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  bool display;
  double max_distance;
  int n_query = 1; int n_iter_per_query = 1;
  std::string replanner_type, group_name;
  std::vector<double> start_configuration, stop_configuration, final_start_configuration, final_stop_configuration;

  nh.getParam("display",display);
  nh.getParam("n_query",n_query);
  nh.getParam("group_name",group_name);
  nh.getParam("max_distance",max_distance);
  nh.getParam("replanner_type",replanner_type);
  nh.getParam("n_iter_per_query",n_iter_per_query);
  nh.getParam("stop_configuration",stop_configuration);
  nh.getParam("start_configuration",start_configuration);
  nh.getParam("final_stop_configuration",final_stop_configuration);
  nh.getParam("final_start_configuration",final_start_configuration);

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
  pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(nh,planning_scene,group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  ros::Duration(0.1).sleep();

  Eigen::VectorXd init_start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd init_goal_conf  = Eigen::Map<Eigen::VectorXd>(stop_configuration .data(), stop_configuration .size());

  Eigen::VectorXd final_start_conf = Eigen::Map<Eigen::VectorXd>(final_start_configuration.data(), final_start_configuration.size());
  Eigen::VectorXd final_goal_conf  = Eigen::Map<Eigen::VectorXd>(final_stop_configuration .data(), final_stop_configuration .size());

  Eigen::VectorXd delta_start = (final_start_conf-init_start_conf)/(std::max(n_query-1,1));
  Eigen::VectorXd delta_goal  = (final_goal_conf -init_goal_conf )/(std::max(n_query-1,1));

  Eigen::VectorXd start_conf = init_start_conf;
  Eigen::VectorXd goal_conf  = init_goal_conf ;

  pathplan::MetricsPtr metrics;
  pathplan::CollisionCheckerPtr checker;
  pathplan::SamplerPtr sampler;
  pathplan::RRTPtr solver;
  pathplan::PathPtr current_path, new_path;
  std::vector<pathplan::PathPtr> other_paths;
  pathplan::ReplannerManagerBasePtr replanner_manager;

  int id_start,id_goal;
  double distance;
  for(int i=0; i<n_query; i++)
  {
    distance = (goal_conf-start_conf).norm();

    disp->clearMarker(id_start);
    disp->clearMarker(id_goal);

    disp->changeNodeSize();
    id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
    id_goal  = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
    disp->defaultNodeSize();
    for(int j=0;j<n_iter_per_query;j++)
    {
      ROS_INFO("---------------------------------------------------------------------------------------------------------");
      ROS_INFO_STREAM("Query: "<<std::to_string(i)<<" Iter: "<<std::to_string(j)<<" start: "<<start_conf.transpose()<< " goal: "<<goal_conf.transpose()<< " distance: "<<distance);
      std::string test_name = "./replanner_test/test_q_"+std::to_string(i)+"_i_"+std::to_string(j);

      nh.setParam("replanner/test_name",test_name); //to save test results

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
      current_path = trajectory->computePath(start_conf,goal_conf,solver,true,10);

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
      else if(replanner_type ==  "DRRT*")
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
      else if(replanner_type == "MARS" || replanner_type == "MARSHA")
      {
        int n_other_paths = 1;
        nh.getParam("/MARS/n_other_paths",n_other_paths);

        for(unsigned int i=0;i<n_other_paths;i++)
        {
          std::srand(std::time(NULL));

          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
          new_path = trajectory->computePath(start_conf,goal_conf,solver,true,10);

          other_paths.clear();
          if(new_path)
          {
            other_paths.push_back(new_path);
            ROS_INFO_STREAM("other path cost "<<new_path->cost());
            assert(new_path->getTree());
          }
        }

        if(replanner_type == "MARSHA")
        {
          int ssm_threads;
          bool ssm_parallel;
          std::string base_frame, tool_frame;
          std::vector<std::string> poi_names;
          double ssm_max_step_size, max_cart_acc, tr, min_distance, v_h;

          nh.getParam("MARSHA/base_frame",base_frame);
          nh.getParam("MARSHA/tool_frame",tool_frame);
          nh.getParam("MARSHA/ssm_max_step_size",ssm_max_step_size);
          nh.getParam("MARSHA/ssm_threads",ssm_threads);
          nh.getParam("MARSHA/max_cart_acc",max_cart_acc);
          nh.getParam("MARSHA/Tr",tr);
          nh.getParam("MARSHA/min_distance",min_distance);
          nh.getParam("MARSHA/v_h",v_h);
          nh.getParam("MARSHA/ssm_parallel",ssm_parallel);
          nh.getParam("MARSHA/poi_names",poi_names);

          Eigen::Vector3d grav; grav << 0, 0, -9.806;
          rosdyn::ChainPtr chain = rosdyn::createChain(*robot_model_loader.getURDF(),base_frame,tool_frame,grav);

          ssm15066_estimator::SSM15066EstimatorPtr ssm;
          if(ssm_parallel)
            ssm = std::make_shared<ssm15066_estimator::ParallelSSM15066Estimator2D>(chain,ssm_max_step_size,ssm_threads);
          else
            ssm = std::make_shared<ssm15066_estimator::SSM15066Estimator2D>(chain,ssm_max_step_size);

          ssm->setHumanVelocity(v_h,false);
          ssm->setMaxCartAcc(max_cart_acc,false);
          ssm->setReactionTime(tr,false);
          ssm->setMinDistance(min_distance,false);
          ssm->setPoiNames(poi_names);
          ssm->updateMembers();

          Eigen::VectorXd scale; scale.setOnes(lb.rows(),1);
          pathplan::LengthPenaltyMetricsPtr ha_metrics = std::make_shared<pathplan::LengthPenaltyMetrics>(ssm,scale);

          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
          solver->config(nh);
          replanner_manager = std::make_shared<pathplan::ReplannerManagerMARSHA>(current_path,solver,nh,ha_metrics,other_paths);
        }
        else
        {
          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
          solver->config(nh);
          replanner_manager = std::make_shared<pathplan::ReplannerManagerMARS>(current_path,solver,nh,other_paths);
        }
      }
      else
      {
        ROS_ERROR("Replanner manager %s does not exist",replanner_type.c_str());
        return 0;
      }

      // //////////////////////////////REPLANNING///////////////////////////////////////////////////
      replanner_manager->start();

      //std::system("clear"); //clear terminal
    }

    start_conf = start_conf+delta_start;
    goal_conf  = goal_conf +delta_goal ;
  }

  return 0;
}

