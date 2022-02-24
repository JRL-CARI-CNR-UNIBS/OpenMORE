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

bool ReplannerManagerAIPRO::checkPathTask(const PathPtr& path)
{
  bool valid = path->isValid();
  path->cost();

  return valid;
}

void ReplannerManagerAIPRO::checkOtherPaths()
{
  ros::Rate lp(collision_checker_thread_frequency_);

  std::vector<PathPtr> other_paths_copy;
  std::vector<CollisionCheckerPtr> checkers;

  paths_mtx_.lock();
  for(const PathPtr& p:other_paths_shared_)
  {
    PathPtr path_copy = p->clone();
    CollisionCheckerPtr checker = checker_cc_->clone();

    path_copy->setChecker(checker);
    checkers.push_back(checker);
    other_paths_copy.push_back(path_copy);
  }
  paths_mtx_.unlock();

  int other_path_size = other_paths_copy.size();

  moveit_msgs::GetPlanningScene ps_srv;

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while((not stop) && ros::ok())
  {
    scene_mtx_.lock();
    if(!plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }
    scene_mtx_.unlock();

    paths_mtx_.lock();
    if(other_path_size<other_paths_shared_.size())  //if the previous current path has been added update the vector of copied paths
    {
      assert(other_path_size == (other_paths_shared_.size()-1));

      CollisionCheckerPtr checker = checker_cc_->clone();
      PathPtr path_copy = other_paths_shared_.back()->clone();

      path_copy->setChecker(checker);
      checkers.push_back(checker);
      other_paths_copy.push_back(path_copy);

      other_path_size = other_paths_copy.size();
    }
    paths_mtx_.unlock();

    std::vector<std::shared_future<bool>> tasks;
    for(unsigned int i=0; i<other_paths_copy.size();i++)
    {
      checkers.at(i)->setPlanningSceneMsg(ps_srv.response.scene);
      tasks.push_back(std::async(std::launch::async,
                                 &ReplannerManagerAIPRO::checkPathTask,
                                 this,other_paths_copy.at(i)));
    }

    for(unsigned int i=0; i<tasks.size();i++)
      tasks.at(i).wait();  //wait the end of each task

    paths_mtx_.lock();
    for(unsigned int i=0;i<other_paths_copy.size();i++)
    {
      std::vector<ConnectionPtr> path_conns      = other_paths_shared_.at(i)->getConnections();
      std::vector<ConnectionPtr> path_copy_conns = other_paths_copy   .at(i)->getConnections();

      assert(path_conns.size() == path_copy_conns.size());

      for(unsigned int z=0;z<path_conns.size();z++)
        path_conns.at(z)->setCost(path_copy_conns.at(z)->getCost());

      other_paths_shared_.at(i)->cost();
    }
    paths_mtx_.unlock();

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

void ReplannerManagerAIPRO::collisionCheckThread()
{
  //std::thread current_path_check_thread = std::thread(&ReplannerManagerBase::collisionCheckThread,this);
  std::thread other_paths_check_thread  = std::thread(&ReplannerManagerAIPRO::checkOtherPaths,this);

//  if(current_path_check_thread.joinable())
//    current_path_check_thread.join();

  if(other_paths_check_thread.joinable())
    other_paths_check_thread.join();
}

}
