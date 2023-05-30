#include "replanners_lib/replanner_managers/replanner_manager_MARS.h"

namespace pathplan
{
ReplannerManagerMARS::ReplannerManagerMARS(const PathPtr &current_path,
                                           const TreeSolverPtr &solver,
                                           const ros::NodeHandle &nh):ReplannerManagerBase(current_path,solver,nh)
{
  ReplannerManagerMARS::MARSadditionalParams();
}

ReplannerManagerMARS::ReplannerManagerMARS(const PathPtr &current_path,
                                           const TreeSolverPtr &solver,
                                           const ros::NodeHandle &nh,
                                           std::vector<PathPtr> &other_paths):ReplannerManagerMARS(current_path,solver,nh)
{
  other_paths_ = other_paths;
  if(replanner_)
  {
    MARSPtr MARS_replanner = std::static_pointer_cast<MARS>(replanner_);
    MARS_replanner->setOtherPaths(other_paths);
  }
}

void ReplannerManagerMARS::MARSadditionalParams()
{
  if(!nh_.getParam("MARS/dt_replan_relaxed",dt_replan_relaxed_))
  {
    ROS_ERROR("MARS/dt_replan_relaxed not set, set 150% of dt_replan");
    dt_replan_relaxed_ = 1.5*dt_replan_;
  }

  if(!nh_.getParam("MARS/reverse_start_nodes",reverse_start_nodes_))
  {
    ROS_ERROR("MARS/reverse_start_nodes not set, set false");
    reverse_start_nodes_ = false;
  }

  if(!nh_.getParam("MARS/full_net_search",full_net_search_))
  {
    ROS_ERROR("MARS/full_net_search_ not set, set true");
    full_net_search_ = true;
  }

  if(!nh_.getParam("MARS/verbosity_level",verbosity_level_))
  {
    ROS_ERROR("MARS/verbosity_level not set, set 0");
    verbosity_level_ = 0;
  }

  if(!nh_.getParam("MARS/display_other_paths",display_other_paths_))
  {
    ROS_ERROR("MARS/display_other_paths_ not set, set true");
    display_other_paths_ = true;
  }

}

void ReplannerManagerMARS::attributeInitialization()
{
  ReplannerManagerBase::attributeInitialization();

  MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);

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

  initial_path_ = current_path_;

  other_paths_shared_.clear();
  other_paths_sync_needed_.clear();
  for(const PathPtr& p:other_paths_)
  {
    PathPtr other_path = p->clone();
    other_paths_shared_.push_back(other_path);

    other_path->setChecker(checker_cc_);
    p->setChecker(checker_replanning_);

    other_paths_sync_needed_.push_back(false);
  }

  time_shift_ = (dt_replan_relaxed_-dt_)*K_OFFSET;
  t_replan_ = t_+time_shift_;

  double scaling = 1.0;
  read_safe_scaling_? (scaling = readScalingTopics()):
                      (scaling = scaling_from_param_);

  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_,scaling);

  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++)
    point2project(i) = pnt_replan_.positions.at(i);

  configuration_replan_ = current_path_shared_->projectOnPath(point2project);
}

bool ReplannerManagerMARS::replan()
{
  double cost = replanner_->getCurrentPath()->getCostFromConf(replanner_->getCurrentConf());
  (cost == std::numeric_limits<double>::infinity())? (replanner_->setMaxTime(0.9*dt_replan_)):
                                                     (replanner_->setMaxTime(0.9*dt_replan_relaxed_));
  bool path_changed = replanner_->replan();

  //CHANGE WITH PATH_CHANGED?
  if(replanner_->getSuccess() && first_replanning_)  //add the initial path to the other paths
  {
    first_replanning_ = false;

    other_paths_mtx_.lock();

    PathPtr another_path = initial_path_->clone();
    another_path->setChecker(checker_cc_);

    other_paths_shared_.push_back(another_path);
    other_paths_sync_needed_.push_back(false);

    MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);
    replanner->addOtherPath(initial_path_,false); //replanner->getCurrentPath() can be slightly different (some new nodes)

    assert(another_path->getConnectionsSize() == initial_path_->getConnectionsSize());

    other_paths_.push_back(initial_path_); //not move from here

    other_paths_mtx_.unlock();
  }

  return path_changed;
}

void ReplannerManagerMARS::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  MARSPtr replanner = std::static_pointer_cast<MARS>(replanner_);

  PathPtr current_path   = replanner->getCurrentPath();
  PathPtr replanned_path = replanner->getReplannedPath();
  NodePtr node_replan    = replanned_path->getStartNode();

  TreePtr tree = current_path->getTree();

  assert(current_path->findConnection(configuration) != nullptr);

  if(old_current_node_ && ((old_current_node_->getConfiguration()-configuration).norm()>TOLERANCE) && old_current_node_ != node_replan && tree->isInTree(old_current_node_))
  {
    if((old_current_node_->getParentConnectionsSize()+old_current_node_->getNetParentConnectionsSize()) == 1)
    {
      if((old_current_node_->getChildConnectionsSize()+old_current_node_->getNetChildConnectionsSize()) == 1)
      {
        /* Remove the old node detaching it from the tree, restore the old connection and set it as initial
         * connection of current path to be able to insert the new current node */

        ConnectionPtr parent_conn;
        (old_current_node_->getParentConnectionsSize()>0)? (parent_conn = old_current_node_->parentConnection(0)):
                                                           (parent_conn = old_current_node_->netParentConnection(0));
        assert([&]() ->bool{
                 if(tree->isInTree(parent_conn->getParent()))
                 return true;
                 else
                 {
                   for(const ConnectionPtr c:old_current_node_->getParentConnections())
                   ROS_INFO_STREAM("parent conn"<<*c);

                   for(const ConnectionPtr c:old_current_node_->getNetParentConnections())
                   ROS_INFO_STREAM("net parent conn"<<*c);

                   ROS_INFO_STREAM("current path "<<*current_path);
                   ROS_INFO_STREAM("old current node "<<*old_current_node_<<old_current_node_);

                   return false;
                 }

               }
               ());

        std::vector<ConnectionPtr> new_conns = current_path->getConnections();
        new_conns.insert(new_conns.begin(),parent_conn);
        current_path->setConnections(new_conns);

        ConnectionPtr restored_conn;
        if(current_path->removeNode(old_current_node_,{},restored_conn))
        {
          std::vector<PathPtr> paths = other_paths_;
          paths.push_back(replanned_path);
          for(PathPtr& p:paths)
          {
            p->restoreConnection(restored_conn,old_current_node_);

            assert([&]() ->bool{
                     for(const NodePtr& n:p->getNodes())
                     if(n == old_current_node_)
                     {
                       return false;
                     }

                     return true;
                   }());


            assert(not tree->isInTree(old_current_node_));
          }
        }
      }
    }
  }

  int conn_idx;
  bool is_a_new_node;
  PathPtr tmp_p = current_path->clone();
  ConnectionPtr conn = current_path->findConnection(configuration,conn_idx);
  NodePtr current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true,is_a_new_node);

  assert([&]() ->bool{
           if((current_node == node_replan && ((configuration-node_replan->getConfiguration()).norm()>TOLERANCE)) || (current_node != node_replan && ((configuration-node_replan->getConfiguration()).norm()<=TOLERANCE)))
           {
             ROS_INFO_STREAM("current node: "<<current_node<<" "<<*current_node);
             ROS_INFO_STREAM("is a new node: "<<is_a_new_node);
             ROS_INFO_STREAM("replan node: "<<node_replan<<" "<<*node_replan);
             ROS_INFO_STREAM("conn: "<<*conn);
             ROS_INFO_STREAM("conf: "<<configuration.transpose());
             ROS_INFO_STREAM("TOLERANCE: "<<TOLERANCE<<" norm: "<<(configuration-node_replan->getConfiguration()).norm());

             ROS_INFO_STREAM("curr p:"<<*current_path);
             ROS_INFO_STREAM("tmp p: "<<*tmp_p);

             tmp_p->findConnection(configuration,conn_idx,true);

             return false;
           }
           return true;
         }());

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
    old_current_node_ = nullptr;

  if(not replanner->getSuccess())
  {
    replanned_path->setConnections(current_path->getSubpathFromNode(current_node)->getConnections());
  }
  else
  {
    std::vector<NodePtr> nodes = current_path->getNodes();

    std::vector<NodePtr>::iterator it_current_node = std::find(nodes.begin(),nodes.end(),current_node);
    std::vector<NodePtr>::iterator it_node_replan  = std::find(nodes.begin(),nodes.end(),node_replan );

    int distance = std::distance(nodes.begin(),it_node_replan)-std::distance(nodes.begin(),it_current_node);

    if(distance==0)
    {
      assert(node_replan == current_node);
    }
    else if(distance<0)
    {
      int idx;
      ConnectionPtr current_conn = replanned_path->findConnection(configuration,idx,true);
      if(current_conn != nullptr) //current node is on replanned path
      {
        if(current_conn->getParent() == current_node || current_conn->getChild() == current_node)
        {
          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
        else
        {
          //          assert([&]() ->bool{
          //                   if(conn == current_conn)
          //                   return true;
          //                   else
          //                   {
          //                     ROS_INFO_STREAM("current conf "<<configuration);
          //                     ROS_INFO_STREAM("conn "<<*conn<<"\n"<<conn);
          //                     ROS_INFO_STREAM("current_conn "<<*current_conn<<"\n"<<current_conn);
          //                     return false;
          //                   }
          //                 }());
          if(conn != current_conn)
          {
            ROS_INFO_STREAM("conf "<<configuration.transpose());

            ROS_INFO_STREAM("conn "<<*conn<<"\n"<<conn);
            ROS_INFO_STREAM("current_conn "<<*current_conn<<"\n"<<current_conn);

            ROS_BOLDBLUE_STREAM("CURRENT PATH "<<*current_path);
            ROS_BOLDBLUE_STREAM("REPLANNED PATH "<<*replanned_path);

            stop_ = true;
            if(not display_thread_.joinable())
              display_thread_.join();


            pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);
            disp->clearMarkers();

            ros::Duration(1).sleep();

            disp->displayConnection(conn,"pathplan",{1,0,0,1});

            disp->displayConnection(current_conn,"pathplan",{0,0,1,1});

            disp->displayNode(std::make_shared<Node>(configuration),"pathplan",{1,0,0,0.5});

            ros::Duration(1).sleep();

            throw std::runtime_error("err");

          }

          if(not replanned_path->splitConnection(current_path->getConnectionsConst().at(conn_idx),
                                                 current_path->getConnectionsConst().at(conn_idx+1),current_conn))
            ROS_BOLDRED_STREAM("CONNECTION NOT SPLITTED");

          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
      }
      else
      {
        //current node should be very close to replan node, minimal difference between the connections

        std::vector<ConnectionPtr> replanned_path_conns = replanned_path->getConnections();

        ConnectionPtr first_conn = replanned_path_conns.front();
        NodePtr child = first_conn->getChild();

        ConnectionPtr new_conn = std::make_shared<Connection>(current_node,child,first_conn->isNet());
        (first_conn->getCost()<std::numeric_limits<double>::infinity())?
              new_conn->setCost(replanned_path->getMetrics()->cost(current_node->getConfiguration(),child->getConfiguration())):
              new_conn->setCost(std::numeric_limits<double>::infinity());

        first_conn->remove();
        new_conn->add();

        replanned_path_conns[0] = new_conn;
        replanned_path->setConnections(replanned_path_conns);
      }
    }
    else //distance>0
    {
      assert(current_node != node_replan);
      assert((current_node->getConfiguration()-node_replan->getConfiguration()).norm()>TOLERANCE);

      PathPtr tmp_subpath;
      tmp_subpath = current_path->getSubpathFromNode(current_node);
      try
      {
        tmp_subpath = tmp_subpath->getSubpathToNode(node_replan);
      }
      catch(...)
      {
        ROS_INFO_STREAM("current path: "<<*current_path); //elimina
        ROS_INFO_STREAM("tmp subpath: "<<*tmp_subpath); //elimina

        ROS_INFO_STREAM("current node: "<<current_node<<" "<<*current_node);
        ROS_INFO_STREAM("node_replan : "<<node_replan<<" "<<*node_replan);
        throw std::runtime_error("runtime err");
      }
      std::vector<ConnectionPtr> new_conns = tmp_subpath->getConnections();

      new_conns.insert(new_conns.end(),replanned_path->getConnectionsConst().begin(),replanned_path->getConnectionsConst().end());
      replanned_path->setConnections(new_conns);

      if(old_current_node_ != nullptr)
      {
        std::vector<NodePtr> pn = replanned_path->getNodes();
        if(std::find(pn.begin(),pn.end(),old_current_node_)>=pn.end())
        {
          ROS_INFO_STREAM("rp "<<*replanned_path);
          ROS_INFO_STREAM("old cur node "<<*old_current_node_<<old_current_node_);

          throw std::runtime_error("error");
        }
      }
    }

    if(replanner->replanNodeIsANewNode() && ((node_replan->getConfiguration()-configuration).norm()>TOLERANCE) && node_replan != old_current_node_)
    {
      ConnectionPtr restored_conn;
      if(replanned_path->removeNode(node_replan,{},restored_conn))
      {
        std::vector<PathPtr> paths = other_paths_;
        paths.push_back(current_path);

        for(PathPtr& p:paths)
        {
          assert(p->getTree() != nullptr);
          p->restoreConnection(restored_conn,node_replan);

          assert(not tree->isInTree(node_replan));
          assert([&]()->bool{
                   std::vector<NodePtr> p_nodes = p->getNodes();
                   if(std::find(p_nodes.begin(),p_nodes.end(),node_replan)<p_nodes.end())
                   {
                     ROS_INFO_STREAM("other path: "<<*p);
                     return false;
                   }
                   else
                   {
                     return true;
                   }}());

        }
      }
    }

    assert([&]() ->bool{
             for(const NodePtr& n:replanned_path->getNodes())
             {
               if(n->getParentConnectionsSize()!=1)
               {
                 for(const NodePtr& nn:replanned_path->getNodes())
                 {
                   ROS_INFO_STREAM(nn<<" "<<*nn);
                 }
                 ROS_INFO_STREAM(*replanned_path);

                 return false;
               }
             }
             return true;
           }());
  }
}

bool ReplannerManagerMARS::haveToReplan(const bool path_obstructed)
{
  return alwaysReplan();
}

void ReplannerManagerMARS::updateSharedPath()
{
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
          break;
        }
      }

      if(other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration() != other_paths_shared_.at(i)->getConnections().back()->getChild()->getConfiguration())
        sync_needed = true;
    }
    else
    {
      sync_needed = true;
    }

    if(sync_needed)
    {
      other_paths_sync_needed_.at(i) = true; //NB: sync needed false set by the collision check thread

      CollisionCheckerPtr checker = other_paths_shared_.at(i)->getChecker();
      other_paths_shared_.at(i) = other_paths_.at(i)->clone();
      other_paths_shared_.at(i)->setChecker(checker);
    }
  }
  other_paths_mtx_.unlock();
}

void ReplannerManagerMARS::downloadPathCost()
{
  ReplannerManagerBase::downloadPathCost();

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
        break;
    }

    other_paths_.at(i)->cost(); //update path cost
  }
  other_paths_mtx_.unlock();
}

void ReplannerManagerMARS::initReplanner()
{
  double time_for_repl = 0.9*dt_replan_;
  pathplan::MARSPtr replanner = std::make_shared<pathplan::MARS>(configuration_replan_,current_path_,time_for_repl,solver_,other_paths_);

  replanner->reverseStartNodes(reverse_start_nodes_);
  replanner->setFullNetSearch(full_net_search_);
  replanner_ = replanner;

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);
  replanner_->setDisp(disp);
}

bool ReplannerManagerMARS::checkPathTask(const PathPtr& path)
{
  bool valid = path->isValid();
  path->cost();

  return valid;
}

void ReplannerManagerMARS::collisionCheckThread()
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

  ros::WallRate lp(collision_checker_thread_frequency_);
  ros::WallTime tic;

  moveit_msgs::PlanningScene planning_scene_msg;

  while((not stop_) && ros::ok())
  {
    tic = ros::WallTime::now();

    /* Update planning scene */
    ps_srv.request.components.components = 20; //moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY + moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS

    if(not plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      stop_ = true;
      break;
    }

    scene_mtx_.lock();
    planning_scene_msg.world = ps_srv.response.scene.world;
    planning_scene_msg.is_diff = true;

    checker_cc_->setPlanningSceneMsg(planning_scene_msg);
    for(const CollisionCheckerPtr& checker: checkers)
      checker->setPlanningSceneMsg(planning_scene_msg);
    scene_mtx_.unlock();

    /* Update paths if they have been changed */
    trj_mtx_.lock();
    paths_mtx_.lock();

    current_configuration_copy = current_configuration_;

    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_sync_needed_ = false;
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
      }
    }

    other_paths_mtx_.unlock();
    paths_mtx_.unlock();
    trj_mtx_.unlock();

    if((current_configuration_copy-replanner_->getGoal()->getConfiguration()).norm()<goal_tol_)
    {
      stop_ = true;
      break;
    }

    /* Launch collision check tasks */

    std::vector<std::shared_future<bool>> tasks;
    for(unsigned int i=0;i<other_paths_copy.size();i++)
    {
      tasks.push_back(std::async(std::launch::async,
                                 &ReplannerManagerMARS::checkPathTask,
                                 this,other_paths_copy.at(i)));
    }

    //current_path_copy->isValidFromConf(current_configuration_copy,checker_cc_);
    int conn_idx;
    current_path_copy->findConnection(current_configuration_copy,conn_idx);
    if(conn_idx<0)
      continue;
    else
      current_path_copy->isValidFromConf(current_configuration_copy,conn_idx,checker_cc_);

    for(unsigned int i=0; i<tasks.size();i++)
      tasks.at(i).wait();  //wait for the end of each task

    /* Update the cost of the paths */
    scene_mtx_.lock();
    if(uploadPathsCost(current_path_copy,other_paths_copy))
    {
      planning_scene_msg_.world = ps_srv.response.scene.world;  //not diff,it contains all pln scn info but only world is updated
      planning_scene_diff_msg_ = planning_scene_msg;            //diff, contains only world

      download_scene_info_ = true;      //dowloadPathCost can be called because the scene and path cost are referred now to the last path found

    }
    scene_mtx_.unlock();

    double duration = (ros::WallTime::now()-tic).toSec();

    if(duration>(1.0/collision_checker_thread_frequency_) && display_timing_warning_)
      ROS_BOLDYELLOW_STREAM("Collision checking thread time expired: total duration-> "<<duration);

    lp.sleep();
  }

  ROS_BOLDCYAN_STREAM("Collision check thread is over");
}

bool ReplannerManagerMARS::uploadPathsCost(const PathPtr& current_path_updated_copy, const std::vector<PathPtr>& other_paths_updated_copy)
{
  bool updated = true;

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
  else
    updated = false;

  if(current_path_shared_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity() && (display_timing_warning_ || display_replanning_success_))
    ROS_BOLDMAGENTA_STREAM("Obstacle detected!");

  other_paths_mtx_.lock();
  for(unsigned int i=0;i<other_paths_updated_copy.size();i++)
  {
    if(not other_paths_sync_needed_.at(i))
    {
      std::vector<ConnectionPtr> path_conns      = other_paths_shared_     .at(i)->getConnections();
      std::vector<ConnectionPtr> path_copy_conns = other_paths_updated_copy.at(i)->getConnections();

      assert(path_conns.size() == path_copy_conns.size());
      for(unsigned int j=0;j<path_conns.size();j++)
      {
        path_conns.at(j)->setCost(path_copy_conns.at(j)->getCost());

        assert((path_conns.at(j)->getParent()->getConfiguration() == path_copy_conns.at(j)->getParent()->getConfiguration()) &&
               (path_conns.at(j)->getChild() ->getConfiguration() == path_copy_conns.at(j)->getChild() ->getConfiguration()));
      }
      other_paths_shared_.at(i)->cost();
    }
    else
      updated = false;
  }
  other_paths_mtx_.unlock();
  paths_mtx_.unlock();       //here to sync the cost of all paths

  return updated;
}

void ReplannerManagerMARS::displayCurrentPath()
{
  ReplannerManagerBase::displayThread();
}

void ReplannerManagerMARS::displayOtherPaths()
{ 
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);
  disp->clearMarkers();

  int path_id,wp_id;
  std::vector<PathPtr> other_paths;
  std::vector<double> marker_color = {1.0,0.5,0.3,1.0};
  std::vector<double> marker_scale = {0.01,0.01,0.01};

  disp->changeNodeSize(marker_scale);

  double display_thread_frequency = 0.75*trj_exec_thread_frequency_;
  ros::WallRate lp(display_thread_frequency);

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

  disp->clearMarkers();
  ROS_BOLDCYAN_STREAM("Display other paths thread is over");
}

bool ReplannerManagerMARS::updateTrajectory()
{
  PathPtr trj_path = replanner_->getReplannedPath()->clone();
  double max_distance = solver_->getMaxDistance();

//  trj_path->simplify(0.0005);
  trj_path->removeNodes(1e-03); //toll 1e-03
  trj_path->resample(max_distance/5.0);

  // Get robot status at t_+time starting at the beginning of trajectory update
  // to have a smoother transition from current trajectory to the new one
  trajectory_msgs::JointTrajectoryPoint pnt;
  interpolator_.interpolate(ros::Duration(t_+(ros::WallTime::now()-tic_trj_).toSec()),pnt,scaling_);

  trajectory_->setPath(trj_path);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj(pnt);
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);

  interpolator_.setTrajectory(tmp_trj_msg)   ;
  interpolator_.setSplineOrder(spline_order_);

  return true;
}

void ReplannerManagerMARS::displayThread()
{
  std::thread current_path_display_thread = std::thread(&ReplannerManagerMARS::displayCurrentPath,this);

  if(display_other_paths_)
  {
    std::thread other_paths_display_thread  = std::thread(&ReplannerManagerMARS::displayOtherPaths ,this);

    if(other_paths_display_thread.joinable())
      other_paths_display_thread.join();
  }

  if(current_path_display_thread.joinable())
    current_path_display_thread.join();
}
}
