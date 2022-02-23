#include "replanners_lib/replanner_managers/replanner_manager_AIPRO.h"

namespace pathplan
{
ReplannerManagerAIPRO::ReplannerManagerAIPRO(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
}

ReplannerManagerAIPRO::ReplannerManagerAIPRO(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh,
                                             std::vector<PathPtr> &other_paths):ReplannerManagerAIPRO(current_path,solver,nh)
{
  setOtherPaths(other_paths);
}

void ReplannerManagerAIPRO::fromParam()
{
  ReplannerManagerBase::fromParam();

  //Additional parameters
  if(!nh_.getParam("dt_replan_replaxed",dt_replan_relaxed_))
  {
    ROS_ERROR("dt_replan_relaxed not set, set 150% of dt_replan");
    dt_replan_relaxed_ = 1.5*dt_replan_;
  }
}

void ReplannerManagerAIPRO::attributeInitialization()
{
  ReplannerManagerBase::attributeInitialization();

  first_replanning_ = true;

  for(const PathPtr& p:other_paths_)
  {
    PathPtr other_path = p->clone();
    other_paths_shared_.push_back(other_path);

    other_path->setChecker(checker_cc_);
    p->setChecker(checker_replanning_);
  }

  replan_offset_ = (dt_replan_relaxed_-dt_)*K_OFFSET;
  t_replan_ = t_+replan_offset_;

  double scaling = 1.0;
  read_safe_scaling_? (scaling = readScalingTopics()):
                      (scaling = scaling_from_param_);

  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_ ,scaling);

  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++)
    point2project(i) = pnt_replan_.positions.at(i);

  configuration_replan_ = current_path_shared_->projectOnClosestConnection(point2project);
}

bool ReplannerManagerAIPRO::replan()
{
  double cost = replanner_->getCurrentPath()->getCostFromConf(replanner_->getCurrentConf());
  cost == std::numeric_limits<double>::infinity()? replanner_->setMaxTime(0.9*dt_replan_):
                                                   replanner_->setMaxTime(0.9*dt_replan_relaxed_);

  bool success = replanner_->replan();

  if(success)
  {
    if(first_replanning_)  //add the initial path to the other paths
    {
      first_replanning_ = false;

      paths_mtx_.lock();
      //other_paths_.push_back(replanner_->getCurrentPath());

      PathPtr current_path = replanner_->getCurrentPath();
      PathPtr another_path = current_path->clone();
      another_path->setChecker(checker_cc_);
      other_paths_shared_.push_back(another_path);

      AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);
      replanner->addOtherPath(current_path);

      // ////ELIMINA//////
      int size_other_paths_rep = pnt_replan_.positions.size();
      int size_other_paths = pnt_replan_.positions.size();
      std::string message = "size rep other paths: "+std::to_string(size_other_paths_rep)+" size other paths: "+std::to_string(size_other_paths);
      assert((message, size_other_paths_rep == size_other_paths));
      // ///////////////

      paths_mtx_.unlock();
    }
  }

  return success;
}

void ReplannerManagerAIPRO::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  return;
}

bool ReplannerManagerAIPRO::haveToReplan(const bool path_obstructed)
{  
  return alwaysReplan();
}

void ReplannerManagerAIPRO::updatePathCost()
{
  ReplannerManagerBase::updatePathCost();

  for(unsigned int i=0;i<other_paths_shared_.size();i++)
  {
    std::vector<ConnectionPtr> other_path_conn        = other_paths_       .at(i)->getConnections();
    std::vector<ConnectionPtr> other_path_shared_conn = other_paths_shared_.at(i)->getConnections();

    for(unsigned int i=0;i<other_path_conn.size();i++)
      other_path_conn.at(i)->setCost(other_path_shared_conn.at(i)->getCost());

    other_paths_.at(i)->cost(); //update path cost
  }
}

void ReplannerManagerAIPRO::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::AIPRO>(configuration_replan_, current_path_replanning_, time_for_repl, solver_,other_paths_);
}

void ReplannerManagerAIPRO::collisionCheckThread()
{
  ros::Rate lp(collision_checker_thread_frequency_);

  Eigen::VectorXd current_configuration_copy;
  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);
  moveit_msgs::GetPlanningScene ps_srv;

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while ((not stop) && ros::ok())
  {
    ros::WallTime tic = ros::WallTime::now();
    double duration_copy_path, duration_update_cost_info, duration_pln_scn_srv, duration_check; //ELIMINA

    scene_mtx_.lock();
    ros::WallTime tic1 = ros::WallTime::now();
    if (!plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }
    duration_pln_scn_srv = (ros::WallTime::now()-tic1).toSec();
    checker_cc_->setPlanningSceneMsg(ps_srv.response.scene);
    scene_mtx_.unlock();

    tic1 = ros::WallTime::now();
    trj_mtx_.lock();
    current_configuration_copy = current_configuration_;
    trj_mtx_.unlock();

    paths_mtx_.lock();
    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;
      //      ROS_WARN("CC syncronized with current path "); //elimina
    }
    paths_mtx_.unlock();

    duration_copy_path = (ros::WallTime::now()-tic1).toSec();

    tic1 = ros::WallTime::now();
    current_path_copy->isValidFromConf(current_configuration_copy,checker_cc_);
    duration_check = (ros::WallTime::now()-tic1).toSec();

    tic1 = ros::WallTime::now();
    paths_mtx_.lock();
    if(not current_path_sync_needed_)  //if changed, it is useless checking current_path_copy
    {
      //      ROS_WARN("CC update cost current path "); //elimina

      std::vector<ConnectionPtr> current_path_conns      = current_path_shared_->getConnections();
      std::vector<ConnectionPtr> current_path_copy_conns = current_path_copy   ->getConnections();
      for(unsigned int z=0;z<current_path_conns.size();z++)
        current_path_conns.at(z)->setCost(current_path_copy_conns.at(z)->getCost());

      current_path_shared_->cost();
    }
    paths_mtx_.unlock();
    duration_update_cost_info = (ros::WallTime::now()-tic1).toSec();

    ros::WallTime toc=ros::WallTime::now();
    double duration = (toc-tic).toSec();

    if(duration>(1.0/collision_checker_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Collision checking thread time expired: total duration-> %f, duration_check-> %f duration_pln_scn_srv-> %f, duration_copy_path-> %f, duration_update_cost_info-> %f",duration,duration_check,duration_pln_scn_srv,duration_copy_path,duration_update_cost_info);
    }

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

}
