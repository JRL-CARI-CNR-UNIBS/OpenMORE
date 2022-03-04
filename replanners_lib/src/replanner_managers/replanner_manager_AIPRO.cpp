#include "replanners_lib/replanner_managers/replanner_manager_AIPRO.h"

namespace pathplan
{
ReplannerManagerAIPRO::ReplannerManagerAIPRO(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  ReplannerManagerAIPRO::additionalParam();
}

ReplannerManagerAIPRO::ReplannerManagerAIPRO(const PathPtr &current_path,
                                             const TreeSolverPtr &solver,
                                             const ros::NodeHandle &nh,
                                             std::vector<PathPtr> &other_paths):ReplannerManagerAIPRO(current_path,solver,nh)
{
  setOtherPaths(other_paths);
}

void ReplannerManagerAIPRO::additionalParam()
{
  //Additional parameters
  if(!nh_.getParam("/aipro/dt_replan_replaxed",dt_replan_relaxed_))
  {
    ROS_ERROR("/aipro/dt_replan_relaxed not set, set 150% of dt_replan");
    dt_replan_relaxed_ = 1.5*dt_replan_;
  }

  if(!nh_.getParam("/aipro/verbosity_level",verbosity_level_))
  {
    ROS_ERROR("/aipro/verbosity_level not set, set 0");
    verbosity_level_ = 0;
  }
}

void ReplannerManagerAIPRO::attributeInitialization()
{
  ReplannerManagerBase::attributeInitialization();

  AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);
  switch(verbosity_level_)
  {
  case 0:
    replanner->setVerbosity(false);
    break;
  case 1:
    replanner->setInformedOnlineReplanningVerbose(true);
    replanner->setPathSwitchVerbose(false);
    break;
  case 2:
    replanner->setInformedOnlineReplanningVerbose(true);
    replanner->setPathSwitchVerbose(true);
    break;
  }

  first_replanning_ = true;
  current_path_cost_update_ready_ = false;
  other_paths_cost_update_ready_  = false;

  old_current_node_ = nullptr;

  updating_cost_pause_ = collision_checker_thread_frequency_/100;

  other_paths_mtx_.lock();
  for(const PathPtr& p:other_paths_)
  {
    PathPtr other_path = p->clone();
    other_paths_shared_.push_back(other_path);

    other_path->setChecker(checker_cc_);
    p->setChecker(checker_replanning_);
  }
  other_paths_mtx_.unlock();

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
  bool path_changed = replanner_->replan();

  if(replanner_->getSuccess() && first_replanning_)  //add the initial path to the other paths
  {
    first_replanning_ = false;
    other_paths_mtx_.lock();

    PathPtr current_path = replanner_->getCurrentPath();
    PathPtr another_path = current_path->clone();
    another_path->setChecker(checker_cc_);
    other_paths_shared_.push_back(another_path);

    AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);
    replanner->addOtherPath(current_path, false);

    // ////ELIMINA//////
    int size_other_paths_rep = pnt_replan_.positions.size();
    int size_other_paths = pnt_replan_.positions.size();
    std::string message = "size rep other paths: "+std::to_string(size_other_paths_rep)+" size other paths: "+std::to_string(size_other_paths);
    assert((message, size_other_paths_rep == size_other_paths));
    // ///////////////

    other_paths_mtx_.unlock();
  }

  return path_changed;
}

void ReplannerManagerAIPRO::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  ROS_INFO("PRIMA DI START NEW CONF");

  AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);

  PathPtr current_path = replanner->getCurrentPath();
  PathPtr replanned_path = replanner->getReplannedPath();
  NetPtr net = replanner->getNet();

  if(old_current_node_)
    current_path->removeNode(old_current_node_,{});

  NodePtr current_node;
  ConnectionPtr conn = current_path->findConnection(configuration);

  assert(conn->isValid());
  current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true);

  std::multimap<double,std::vector<ConnectionPtr>> new_conns_map = net->getConnectionBetweenNodes(current_node,replanner->getGoal());

  replanned_path->setConnections(new_conns_map.begin()->second);

  old_current_node_ = current_node;

  ROS_INFO("DOPO DI START NEW CONF");
}

bool ReplannerManagerAIPRO::haveToReplan(const bool path_obstructed)
{  
  return alwaysReplan();
}

void ReplannerManagerAIPRO::syncPathCost()
{
  //  while(current_path_cost_update_ready_ || other_paths_cost_update_ready_) //wait until both cost of current path and costs of other paths are updated
  //    ros::Duration(updating_cost_pause_).sleep();

  ReplannerManagerBase::syncPathCost();

  other_paths_mtx_.lock();
  int other_paths_size = std::min(other_paths_.size(),other_paths_shared_.size());
  for(unsigned int i=0;i<other_paths_size;i++)
  {
    std::vector<ConnectionPtr> other_path_conn        = other_paths_       .at(i)->getConnections();
    std::vector<ConnectionPtr> other_path_shared_conn = other_paths_shared_.at(i)->getConnections();

    for(unsigned int i=0;i<other_path_conn.size();i++)
      other_path_conn.at(i)->setCost(other_path_shared_conn.at(i)->getCost());

    other_paths_.at(i)->cost(); //update path cost
  }
  other_paths_mtx_.unlock();
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

  other_paths_mtx_.lock();
  for(const PathPtr& p:other_paths_shared_)
  {
    PathPtr path_copy = p->clone();
    CollisionCheckerPtr checker = checker_cc_->clone();

    path_copy->setChecker(checker);
    checkers.push_back(checker);
    other_paths_copy.push_back(path_copy);
  }
  other_paths_mtx_.unlock();

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

      stop_mtx_.lock();
      stop_ = true;
      stop_mtx_.unlock();

      break;
    }
    scene_mtx_.unlock();

    other_paths_mtx_.lock();
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
    other_paths_mtx_.unlock();

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

    updateOtherPathsCost(other_paths_copy);

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

void ReplannerManagerAIPRO::updatePathCost(const PathPtr& current_path_updated_copy)
{
  //  current_path_cost_update_ready_ = true;
  //  while(not other_paths_cost_update_ready_)
  //    ros::Duration(updating_cost_pause_).sleep();

  paths_mtx_.lock();
  if(not current_path_sync_needed_)  //if changed, it is useless checking current_path_copy
  {
    std::vector<ConnectionPtr> current_path_conns      = current_path_shared_     ->getConnections();
    std::vector<ConnectionPtr> current_path_copy_conns = current_path_updated_copy->getConnections();
    for(unsigned int z=0;z<current_path_conns.size();z++)
      current_path_conns.at(z)->setCost(current_path_copy_conns.at(z)->getCost());

    current_path_shared_->cost();
  }
  paths_mtx_.unlock();

  current_path_cost_update_ready_ = false;
}

void ReplannerManagerAIPRO::updateOtherPathsCost(const std::vector<PathPtr>& other_paths_updated_copy)
{
  //  other_paths_cost_update_ready_ = true;
  //  while(not current_path_cost_update_ready_)
  //    ros::Duration(updating_cost_pause_).sleep();

  other_paths_mtx_.lock();
  for(unsigned int i=0;i<other_paths_updated_copy.size();i++)
  {
    std::vector<ConnectionPtr> path_conns      = other_paths_shared_     .at(i)->getConnections();
    std::vector<ConnectionPtr> path_copy_conns = other_paths_updated_copy.at(i)->getConnections();

    assert(path_conns.size() == path_copy_conns.size());

    for(unsigned int z=0;z<path_conns.size();z++)
      path_conns.at(z)->setCost(path_copy_conns.at(z)->getCost());

    other_paths_shared_.at(i)->cost();
  }
  other_paths_mtx_.unlock();

  other_paths_cost_update_ready_ = false;
}

void ReplannerManagerAIPRO::checkCurrentPath()
{
  ReplannerManagerBase::collisionCheckThread();
}

void ReplannerManagerAIPRO::collisionCheckThread()
{ 
  std::thread current_path_check_thread = std::thread(&ReplannerManagerAIPRO::checkCurrentPath,this);
  std::thread other_paths_check_thread  = std::thread(&ReplannerManagerAIPRO::checkOtherPaths ,this);

  if(current_path_check_thread.joinable())
    current_path_check_thread.join();

  if(other_paths_check_thread.joinable())
    other_paths_check_thread.join();
}

void ReplannerManagerAIPRO::displayCurrentPath()
{
  ReplannerManagerBase::displayThread();
}

void ReplannerManagerAIPRO::displayOtherPaths()
{
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);

  std::vector<PathPtr> other_paths;
  std::vector<double> marker_color = {1.0,0.5,0.3,1.0};
  std::vector<double> marker_scale = {0.01,0.01,0.01};

  disp->changeNodeSize(marker_scale);

  double display_thread_frequency = 0.75*trj_exec_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  stop_mtx_.lock();
  bool stop = stop_;
  stop_mtx_.unlock();

  while((not stop) && ros::ok())
  {
    other_paths.clear();

    other_paths_mtx_.lock();
    for(const PathPtr& p:other_paths_shared_)
      other_paths.push_back(p->clone());
    other_paths_mtx_.unlock();

    int path_id = 20000;
    int wp_id = 25000;

    //disp->changeConnectionSize(marker_scale);

    for(const PathPtr& p:other_paths)
    {
      disp->displayPathAndWaypoints(p,path_id,wp_id,"pathplan",marker_color);

      path_id +=1;
      wp_id +=1000;
    }

    //disp->defaultConnectionSize();

    stop_mtx_.lock();
    stop = stop_;
    stop_mtx_.unlock();

    lp.sleep();
  }
}

void ReplannerManagerAIPRO::displayThread()
{
  std::thread current_path_display_thread = std::thread(&ReplannerManagerAIPRO::displayCurrentPath,this);
  std::thread other_paths_display_thread  = std::thread(&ReplannerManagerAIPRO::displayOtherPaths ,this);

  if(current_path_display_thread.joinable())
    current_path_display_thread.join();

  if(other_paths_display_thread.joinable())
    other_paths_display_thread.join();
}

}
