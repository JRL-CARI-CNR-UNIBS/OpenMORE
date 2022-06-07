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

void ReplannerManagerAIPRO::setOtherPaths(std::vector<PathPtr>& other_paths)
{
  other_paths_ = other_paths;
  if(replanner_)
  {
    AIPROPtr aipro_replanner = std::static_pointer_cast<AIPRO>(replanner_);
    aipro_replanner->setOtherPaths(other_paths);
  }
}

void ReplannerManagerAIPRO::additionalParam()
{
  if(!nh_.getParam("/aipro/dt_replan_relaxed",dt_replan_relaxed_))
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

  if(replanner_verbosity_)
  {
    if(verbosity_level_>2)
      verbosity_level_ = 2;
    else if(verbosity_level_<0)
      verbosity_level_ = 0;

    switch(verbosity_level_)
    {
    case 0:
      replanner->setInformedOnlineReplanningVerbose(false);
      replanner->setPathSwitchVerbose(false);
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
  }

  first_replanning_ = true;
  old_current_node_ = nullptr;

  initial_path_ = current_path_replanning_;

  other_paths_sync_needed_.clear();
  for(const PathPtr& p:other_paths_)
  {
    PathPtr other_path = p->clone();
    other_paths_shared_.push_back(other_path);

    other_path->setChecker(checker_cc_);
    p->setChecker(checker_replanning_);

    other_paths_sync_needed_.push_back(false);
  }

  replan_offset_ = (dt_replan_relaxed_-dt_)*K_OFFSET;
  t_replan_ = t_+replan_offset_;

  double scaling = 1.0;
  read_safe_scaling_? (scaling = readScalingTopics()):
                      (scaling = scaling_from_param_);

  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_,scaling);

  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++)
    point2project(i) = pnt_replan_.positions.at(i);

  configuration_replan_ = current_path_shared_->projectOnClosestConnection(point2project);
}

bool ReplannerManagerAIPRO::replan()
{
  double cost = replanner_->getCurrentPath()->getCostFromConf(replanner_->getCurrentConf());
  (cost == std::numeric_limits<double>::infinity())? (replanner_->setMaxTime(0.9*dt_replan_)):
                                                     (replanner_->setMaxTime(0.9*dt_replan_relaxed_));

  bool path_changed = replanner_->replan();

  if(replanner_->getSuccess() && first_replanning_)  //add the initial path to the other paths
  {
    first_replanning_ = false;

    other_paths_mtx_.lock();
    assert(replanner_->getCurrentPath() == initial_path_);

    PathPtr another_path = initial_path_->clone();
    another_path->setChecker(checker_cc_);

    other_paths_shared_.push_back(another_path);
    other_paths_sync_needed_.push_back(false);

    AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);
    replanner->addOtherPath(initial_path_,false);

    assert(another_path->getConnectionsSize() == initial_path_->getConnectionsSize());

    replanner_mtx_.lock();
    other_paths_.push_back(initial_path_); //not move from here
    replanner_mtx_.unlock();

    other_paths_mtx_.unlock();
  }

  return path_changed;
}

void ReplannerManagerAIPRO::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  AIPROPtr replanner = std::static_pointer_cast<AIPRO>(replanner_);

  NetPtr  net            = replanner->getNet();
  PathPtr current_path   = replanner->getCurrentPath();
  PathPtr replanned_path = replanner->getReplannedPath();
  NodePtr node_replan    = replanned_path->getConnections().front()->getParent();

  TreePtr tree = current_path->getTree();

  if(old_current_node_ && (old_current_node_->getConfiguration() != configuration))
  {
    if((old_current_node_->parent_connections_.size()+old_current_node_->net_parent_connections_.size()) == 1)
    {
      if((old_current_node_->child_connections_.size()+old_current_node_->net_child_connections_.size()) == 1)
      {
        /* Remove the old node detaching it from the tree, restore the old connection and set it as initial
         * connection of current path to be able to insert the new current node */

        ConnectionPtr parent_conn;
        (old_current_node_->parent_connections_.size()>0)? (parent_conn = old_current_node_->parent_connections_.front()):
                                                           (parent_conn = old_current_node_->net_parent_connections_.front());

        assert(tree->isInTree(parent_conn->getParent()));

        std::vector<ConnectionPtr> new_conns = current_path->getConnections();
        new_conns.insert(new_conns.begin(),parent_conn);
        current_path->setConnections(new_conns);

        ConnectionPtr restored_conn;
        if(not current_path->removeNode(old_current_node_,{},restored_conn))
          ROS_ERROR_STREAM("OLD CURRENT NODE NOT REMOVED!"<<old_current_node_->getConfiguration().transpose());
        else
        {
          ROS_WARN_STREAM(" OLD current node removed: "<<*old_current_node_);
          for(PathPtr& p:other_paths_)
          {
            p->restoreConnection(restored_conn,old_current_node_);

            for(const NodePtr& n:p->getNodes())
              assert(n != old_current_node_);

            assert(not current_path->getTree()->isInTree(old_current_node_));
          }
        }
      }
    }
  }

  if(replanner->replanNodeIsANewNode() && (node_replan->getConfiguration() != configuration))
  {
    ConnectionPtr restored_conn;
    if(not current_path->removeNode(node_replan,{},restored_conn))
      ROS_ERROR_STREAM("REPLAN NODE NOT REMOVED !"<<node_replan->getConfiguration().transpose());
    else
    {
      ROS_WARN_STREAM("Node replan removed: "<<*node_replan);
      ROS_INFO_STREAM("RESTORED CONN: "<<*restored_conn);

      int i=-1;
      for(PathPtr& p:other_paths_)
      {
        i++; //elimina
        assert(p->getTree() != nullptr);
        if(p->restoreConnection(restored_conn,node_replan))
          ROS_WARN_STREAM("CONN RESTORED IN PATH: "<<i);

        for(const NodePtr& n:p->getNodes())
          assert(n != node_replan);

        assert(not current_path->getTree()->isInTree(node_replan));
      }
    }
  }

  int conn_idx;
  bool is_a_new_node;
  ConnectionPtr conn = current_path->findConnection(configuration,conn_idx);
  NodePtr current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true,is_a_new_node);

  ROS_INFO_STREAM("CURRENT NODE: "<<*current_node<<current_node<<"\n"<<is_a_new_node);

  if(is_a_new_node)
  {
    old_current_node_ = current_node;
    for(PathPtr &p:other_paths_)
    {
      if(p->splitConnection(current_path->getConnectionsConst().at(conn_idx),
                            current_path->getConnectionsConst().at(conn_idx+1),conn));
    }
  }
  else
    old_current_node_ = current_node;


  std::multimap<double,std::vector<ConnectionPtr>> new_conns_map = net->getConnectionBetweenNodes(current_node,replanner->getGoal());
  replanned_path->setConnections(new_conns_map.begin()->second);
}

bool ReplannerManagerAIPRO::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerAIPRO::updateSharedPath()
{
  ROS_WARN("AGGIORNO I PATH CONDIVISI");
  ReplannerManagerBase::updateSharedPath();

  other_paths_mtx_.lock();
  bool sync_needed;

  for(unsigned int i=0;i<other_paths_shared_.size();i++)
  {
    sync_needed = false;
    if(other_paths_shared_.at(i)->getConnectionsSize() == other_paths_.at(i)->getConnectionsSize())
    {
      for(unsigned int j=0;j<other_paths_shared_.at(i)->getConnectionsSize();j++)
      {
        if(other_paths_shared_.at(i)->getConnections().at(j)->getParent()->getConfiguration() != other_paths_.at(i)->getConnections().at(j)->getParent()->getConfiguration())
        {
          sync_needed = true;
          ROS_WARN("OTHER path n %i shared changed, diff %f",i, (other_paths_shared_.at(i)->getConnections().at(j)->getParent()->getConfiguration() - other_paths_.at(i)->getConnections().at(j)->getParent()->getConfiguration()).norm());

          ROS_WARN_STREAM("shared: "<<*other_paths_shared_.at(i)); //elimina
          ROS_WARN_STREAM("other: "<<*other_paths_.at(i));
          break;
        }
      }

      if(other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration() != other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration())
        sync_needed = true;
    }
    else
    {
      sync_needed = true;
      ROS_WARN("OTHER path n %i shared changed",i);
      ROS_WARN_STREAM(*other_paths_shared_.at(i));
      ROS_WARN("other path: ");
      ROS_WARN_STREAM(*other_paths_.at(i));
    }

    if(sync_needed)
    {
      other_paths_sync_needed_.at(i) = true; //NB: sync needed false set by the collision check thread

      CollisionCheckerPtr checker = other_paths_shared_.at(i)->getChecker();
      other_paths_shared_.at(i) = other_paths_.at(i)->clone();
      other_paths_shared_.at(i)->setChecker(checker);

      ROS_WARN("other path shared %i changed",i);
      ROS_WARN_STREAM(*other_paths_shared_.at(i));
    }
  }
  other_paths_mtx_.unlock();
  ROS_WARN("PATH CONDIVISI AGGIORNATI");
}

void ReplannerManagerAIPRO::syncPathCost()
{
  ROS_WARN("SINCRONIZZO IL COSTO DEI PATH");

  ReplannerManagerBase::syncPathCost();

  other_paths_mtx_.lock();
  unsigned int other_paths_size = std::min(other_paths_.size(),other_paths_shared_.size());
  for(unsigned int i=0;i<other_paths_size;i++)
  {
    std::vector<ConnectionPtr> other_path_conn        = other_paths_       .at(i)->getConnections();
    std::vector<ConnectionPtr> other_path_shared_conn = other_paths_shared_.at(i)->getConnections();

    std::vector<ConnectionPtr>::iterator it        = other_path_conn       .end();
    std::vector<ConnectionPtr>::iterator it_shared = other_path_shared_conn.end();
    while(it>other_path_conn.begin() && it_shared>other_path_shared_conn.begin())
    {
      it--;
      it_shared--;

      if((*it)->getParent()->getConfiguration() == (*it_shared)->getParent()->getConfiguration() &&
         (*it)->getChild() ->getConfiguration() == (*it_shared)->getChild() ->getConfiguration())
      {
        (*it)->setCost((*it_shared)->getCost());
      }
      else
      {
        ROS_INFO("shared path %i",i); //elimina
        ROS_INFO_STREAM(*other_paths_shared_.at(i));
        ROS_INFO("other path %i",i);
        ROS_INFO_STREAM(*other_paths_.at(i));

        break;
      }
    }

    other_paths_.at(i)->cost(); //update path cost
  }
  other_paths_mtx_.unlock();

  ROS_WARN("COSTO DEI PATH SINCRONIZZATO");
}

void ReplannerManagerAIPRO::initReplanner()
{ 
  double time_for_repl = 0.9*dt_replan_;
  replanner_ = std::make_shared<pathplan::AIPRO>(configuration_replan_,current_path_replanning_,time_for_repl,solver_,other_paths_);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_); //elimina
  replanner_->setDisp(disp);  //elimina
}

bool ReplannerManagerAIPRO::checkPathTask(const PathPtr& path)
{
  bool valid = path->isValid();
  path->cost();

  return valid;
}

void ReplannerManagerAIPRO::collisionCheckThread()
{
  moveit_msgs::GetPlanningScene ps_srv;
  Eigen::VectorXd current_configuration_copy;

  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);

  std::vector<PathPtr> other_paths_copy;
  std::vector<CollisionCheckerPtr> checkers;
  for(const PathPtr& p:other_paths_shared_)
  {
    PathPtr path_copy = p->clone();
    CollisionCheckerPtr checker = checker_cc_->clone();

    checkers.push_back(checker);
    path_copy->setChecker(checker);
    other_paths_copy.push_back(path_copy);
  }

  int other_path_size = other_paths_copy.size();

  ros::Rate lp(collision_checker_thread_frequency_);
  ros::WallTime tic;

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    /* Update planning scene */
    scene_mtx_.lock();
    if(not plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      stop_ = true;
      break;
    }

    checker_cc_->setPlanningSceneMsg(ps_srv.response.scene);
    for(const CollisionCheckerPtr& checker: checkers)
      checker->setPlanningSceneMsg(ps_srv.response.scene);

    scene_mtx_.unlock();

    /* Update paths if they have been changed */
    paths_mtx_.lock();
    ROS_WARN("AGGIORNO I PATH IN CC");

    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;

      ROS_WARN_STREAM("cc current path viene clonato"<<*current_path_copy);
    }

    other_paths_mtx_.lock();
    if(other_path_size<other_paths_shared_.size())  // if the previous current path has been added, update the vector of copied paths
    {
      assert(other_path_size == (other_paths_shared_.size()-1));

      CollisionCheckerPtr checker = checker_cc_->clone();
      PathPtr path_copy = other_paths_shared_.back()->clone();

      checkers.push_back(checker);
      path_copy->setChecker(checker);
      other_paths_copy.push_back(path_copy);

      other_path_size = other_paths_copy.size();
    }

    for(unsigned int i=0;i<other_paths_shared_.size();i++)  // sync other_paths_shared with its copy
    {
      if(other_paths_sync_needed_.at(i))
      {
        other_paths_copy.at(i) = other_paths_shared_.at(i)->clone();
        other_paths_copy.at(i)->setChecker(checkers.at(i));
        other_paths_sync_needed_.at(i) = false;
        ROS_WARN("cc path %i viene clonato",i);
        ROS_WARN_STREAM(*other_paths_copy.at(i));
      }
    }
    ROS_WARN("PATH IN CC AGGIORNATI");

    other_paths_mtx_.unlock();
    paths_mtx_.unlock();

    /* Launch collision check tasks */
    std::vector<std::shared_future<bool>> tasks;
    for(unsigned int i=0;i<other_paths_copy.size();i++)
    {
      tasks.push_back(std::async(std::launch::async,
                                 &ReplannerManagerAIPRO::checkPathTask,
                                 this,other_paths_copy.at(i)));
    }

    trj_mtx_.lock();
    current_configuration_copy = current_configuration_;
    trj_mtx_.unlock();

    if(current_configuration_copy == replanner_->getGoal()->getConfiguration())
    {
      stop_ = true;
      break;
    }

    current_path_copy->isValidFromConf(current_configuration_copy,checker_cc_);

    for(unsigned int i=0; i<tasks.size();i++)
      tasks.at(i).wait();  //wait for the end of each task

    /* Update the cost of the paths */
    scene_mtx_.lock();
    updatePathsCost(current_path_copy,other_paths_copy);
    planning_scene_msg_ = ps_srv.response.scene;
    scene_mtx_.unlock();

    ros::WallTime toc=ros::WallTime::now();
    double duration = (toc-tic).toSec();

    if(duration>(1.0/collision_checker_thread_frequency_) && display_timing_warning_)
      ROS_WARN("Collision checking thread time expired: total duration-> %f",duration);

    lp.sleep();
  }
}

void ReplannerManagerAIPRO::updatePathsCost(const PathPtr& current_path_updated_copy, const std::vector<PathPtr>& other_paths_updated_copy)
{
  ROS_WARN("CARICO IL COSTO NEI PATH CONDIVISI");

  paths_mtx_.lock();
  if(not current_path_sync_needed_)
  {
    std::vector<ConnectionPtr> current_path_conns      = current_path_shared_     ->getConnections();
    std::vector<ConnectionPtr> current_path_copy_conns = current_path_updated_copy->getConnections();

    for(unsigned int j=0;j<current_path_conns.size();j++)
    {
      assert((current_path_conns.at(j)->getParent()->getConfiguration() == current_path_copy_conns.at(j)->getParent()->getConfiguration()) &&
             (current_path_conns.at(j)->getChild() ->getConfiguration() == current_path_copy_conns.at(j)->getChild() ->getConfiguration()));

      current_path_conns.at(j)->setCost(current_path_copy_conns.at(j)->getCost());
    }

    current_path_shared_->cost();
  }

  if(current_path_shared_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
    ROS_WARN("Obstacle detected!");

  other_paths_mtx_.lock();
  for(unsigned int i=0;i<other_paths_updated_copy.size();i++)
  {
    if(not other_paths_sync_needed_.at(i))
    {
      std::vector<ConnectionPtr> path_conns      = other_paths_shared_     .at(i)->getConnections();
      std::vector<ConnectionPtr> path_copy_conns = other_paths_updated_copy.at(i)->getConnections();

      if(path_conns.size() != path_copy_conns.size()) //elimina
      {
        ROS_ERROR_STREAM("other path shared size: "<<other_paths_shared_.size());
        ROS_ERROR_STREAM("other path shared updated size: "<<other_paths_updated_copy.size());
        ROS_ERROR_STREAM("INDEX: "<<i);

        assert(i<other_paths_shared_.size());
        assert(i<other_paths_updated_copy.size());

        ROS_ERROR("shared path %i",i); //elimina
        ROS_ERROR_STREAM(*other_paths_shared_.at(i));
        ROS_ERROR("updated_shared path %i",i);
        ROS_ERROR_STREAM(*other_paths_updated_copy.at(i));
        assert(0);
      }

      assert(path_conns.size() == path_copy_conns.size());
      for(unsigned int j=0;j<path_conns.size();j++)
      {
        path_conns.at(j)->setCost(path_copy_conns.at(j)->getCost());

        assert((path_conns.at(j)->getParent()->getConfiguration() == path_copy_conns.at(j)->getParent()->getConfiguration()) &&
               (path_conns.at(j)->getChild() ->getConfiguration() == path_copy_conns.at(j)->getChild() ->getConfiguration()));
      }
      other_paths_shared_.at(i)->cost();
    }
  }
  other_paths_mtx_.unlock();
  paths_mtx_.unlock();       //here to sync the cost of all paths
  ROS_WARN("COSTO CARICATO NEI PATH CNDIVISI");

}

void ReplannerManagerAIPRO::displayCurrentPath()
{
  ReplannerManagerBase::displayThread();
}

void ReplannerManagerAIPRO::displayOtherPaths()
{
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);

  int path_id,wp_id;
  std::vector<PathPtr> other_paths;
  std::vector<double> marker_color = {1.0,0.5,0.3,1.0};
  std::vector<double> marker_scale = {0.01,0.01,0.01};

  disp->changeNodeSize(marker_scale);

  double display_thread_frequency = 0.75*trj_exec_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  while((not stop_) && ros::ok())
  {
    other_paths.clear();

    other_paths_mtx_.lock();
    for(const PathPtr& p:other_paths_shared_)
      other_paths.push_back(p->clone());
    other_paths_mtx_.unlock();

    path_id = 20000;
    wp_id = 25000;

    for(const PathPtr& p:other_paths)
    {
      disp->displayPathAndWaypoints(p,path_id,wp_id,"pathplan",marker_color);

      path_id +=1;
      wp_id +=1000;
    }

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
