#include "replanners_lib/replanner_managers/replanner_manager_MARSHA.h"

namespace pathplan
{

ReplannerManagerMARSHA::ReplannerManagerMARSHA(const PathPtr &current_path,
                                               const TreeSolverPtr &solver,
                                               const ros::NodeHandle &nh,
                                               const LengthPenaltyMetricsPtr& ha_metrics,
                                               std::vector<PathPtr> &other_paths):
  ReplannerManagerMARS(current_path,solver,nh,other_paths)
{
  ha_metrics_ = ha_metrics;
  setSSM();
  MARSHAadditionalParams();
}

void ReplannerManagerMARSHA::MARSHAadditionalParams()
{
  unaware_obstacles_.clear();
  if(!nh_.getParam("MARSHA/unaware_obstacles",unaware_obstacles_))
    ROS_ERROR("MARSHA/unaware_obstacles_ not set");

  poi_names_.clear();
  if(nh_.getParam("MARSHA/poi_names",poi_names_))
  {
    if(ssm_)
      ssm_->setPoiNames(poi_names_);
  }
  else
    ROS_ERROR("MARSHA/poi_names_ not set");
}

void ReplannerManagerMARSHA::attributeInitialization()
{
  ReplannerManagerMARS::attributeInitialization();

  metrics_shared_ = std::make_shared<Metrics>();
  current_path_shared_->setMetrics(metrics_shared_);

  other_metrics_shared_.clear();
  for(const PathPtr& p:other_paths_shared_)
  {
    other_metrics_shared_.push_back(std::make_shared<Metrics>());
    p->setMetrics(other_metrics_shared_.back());
  }
}

void ReplannerManagerMARSHA::setSSM()
{
  if(std::dynamic_pointer_cast<ssm15066_estimator::SSM15066Estimator>(ha_metrics_->getPenalizer()) == nullptr)
    throw std::runtime_error("Cost penalizer should be of SSM15066Estimator type");
  else
    ssm_ = std::static_pointer_cast<ssm15066_estimator::SSM15066Estimator>(ha_metrics_->getPenalizer());
}

void ReplannerManagerMARSHA::downloadPathCost()
{
  ssm_->setObstaclesPositions(obstalces_positions_);
  ReplannerManagerMARS::downloadPathCost();
}

bool ReplannerManagerMARSHA::updateTrajectory()
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

void ReplannerManagerMARSHA::startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)
{
  MARSPtr replanner = std::static_pointer_cast<MARSHA>(replanner_);

  PathPtr current_path   = replanner->getCurrentPath();
  PathPtr replanned_path = replanner->getReplannedPath();
  NodePtr node_replan    = replanned_path->getStartNode();

  TreePtr tree = current_path->getTree();

  ROS_INFO("QUA");
  if(old_current_node_ && old_current_node_ != node_replan && ((old_current_node_->getConfiguration()-configuration).norm()>TOLERANCE))
  {
    ROS_INFO("QUA1");

    if((old_current_node_->getParentConnectionsSize()+old_current_node_->getNetParentConnectionsSize()) == 1)
    {
      ROS_INFO("QUA2");

      if((old_current_node_->getChildConnectionsSize()+old_current_node_->getNetChildConnectionsSize()) == 1)
      {
        ROS_INFO("QUA3");

        if(tree->isInTree(old_current_node_))
        {
          ROS_INFO("QUA4");

          /* Remove the old node detaching it from the tree, restore the old connection and set it as initial
             * connection of current path to be able to insert the new current node */

          ConnectionPtr parent_conn;
          (old_current_node_->getParentConnectionsSize()>0)? (parent_conn = old_current_node_->parentConnection(0)):
                                                             (parent_conn = old_current_node_->netParentConnection(0));

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
            }
          }
        }
      }
    }
  }

  int conn_idx;
  bool is_a_new_node;
  ConnectionPtr conn = current_path->findConnection(configuration,conn_idx);
  NodePtr current_node = current_path->addNodeAtCurrentConfig(configuration,conn,true,is_a_new_node);

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

  if(not replanner_->getSuccess())
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
      if(node_replan != current_node)
        throw std::runtime_error("nodes are different");
    }
    else if(distance<0) //current node ahead of replan node
    {
      ROS_INFO("DISTANCE < 0");

      int idx;
      ConnectionPtr current_conn = replanned_path->findConnection(configuration,idx,true);
      if(current_conn != nullptr) //current node is still on replanned path -> simply extract subpath
      {
        ROS_INFO("ON REPLANNED PATH");

        if(current_conn->getParent() == current_node || current_conn->getChild() == current_node)
        {
          ROS_INFO("1");

          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
        else
        {
          ROS_INFO("2");
          ROS_INFO_STREAM("conn1 "<<*current_path->getConnectionsConst().at(conn_idx  ));
          ROS_INFO_STREAM("conn2 "<<*current_path->getConnectionsConst().at(conn_idx+1));
          ROS_INFO_STREAM("whole conn "<<*current_conn);

          ROS_INFO_STREAM("current path "<<*current_path);
          ROS_INFO_STREAM("replanned path "<<*replanned_path);

          if(not replanned_path->splitConnection(current_path->getConnectionsConst().at(conn_idx),
                                                 current_path->getConnectionsConst().at(conn_idx+1),current_conn))
            ROS_BOLDRED_STREAM("CONNECTION NOT SPLITTED");

          replanned_path->setConnections(replanned_path->getSubpathFromNode(current_node)->getConnections());
        }
      }
      else //current node is not on the replanned path
      {
        ROS_INFO("NOT ON THE REPLANNED PATH");

        //current node should be very close to replan node, minimal difference between the connections
        //Connect to closest node
        std::vector<ConnectionPtr> replanned_path_conns = replanned_path->getConnections();

        NodePtr child;
        ConnectionPtr conn_prj;
        replanned_path->projectOnPath(configuration,replanned_path->getStartNode()->getConfiguration(),conn_prj,false);
        if(conn_prj == nullptr)
        {
          ROS_INFO("CONN_PRJ NULLPTR");

          conn_prj = replanned_path_conns.front();
          child = conn_prj->getParent();
        }
        else
        {
          ROS_INFO("CONN_PRJ NOT NULLPTR");

          child = conn_prj->getChild();
        }

        ConnectionPtr new_conn = std::make_shared<Connection>(current_node,child,true);
        (conn_prj->getCost()<std::numeric_limits<double>::infinity())?
              new_conn->setCost(replanned_path->getMetrics()->cost(current_node->getConfiguration(),child->getConfiguration())):
              new_conn->setCost(std::numeric_limits<double>::infinity());

        //        conn_prj->remove();
        new_conn->add();

        uint idx =0;
        for(unsigned int i=0;i<replanned_path_conns.size();i++)
        {
          if(replanned_path_conns[i] == conn_prj)
          {
            idx = i;
            break;
          }
        }
        replanned_path_conns[idx] = new_conn;
        std::vector<ConnectionPtr> new_conns(replanned_path_conns.begin()+idx,replanned_path_conns.end());
        replanned_path->setConnections(new_conns);
      }
    }
    else //distance>0 (current node is before replan node)
    {
      ROS_INFO("DISTANCE > 0");

      PathPtr tmp_subpath;
      tmp_subpath = current_path->getSubpathFromNode(current_node);
      tmp_subpath = tmp_subpath->getSubpathToNode(node_replan);

      std::vector<ConnectionPtr> new_conns = tmp_subpath->getConnections();

      new_conns.insert(new_conns.end(),replanned_path->getConnectionsConst().begin(),replanned_path->getConnectionsConst().end());
      replanned_path->setConnections(new_conns);
    }

    if(replanner->replanNodeIsANewNode() && ((node_replan->getConfiguration()-configuration).norm()>TOLERANCE) && node_replan != old_current_node_)
    {
      ConnectionPtr restored_conn;
      if(replanned_path->removeNode(node_replan,{},restored_conn))
      {
        std::vector<PathPtr> paths = other_paths_;
        paths.push_back(current_path);

        for(PathPtr& p:paths)
          p->restoreConnection(restored_conn,node_replan);
      }
    }
  }
}

void ReplannerManagerMARSHA::initReplanner()
{
  if(not ha_metrics_)
    throw std::runtime_error("HA metrics not defined!");

  double time_for_repl = 0.9*dt_replan_;
  pathplan::MARSHAPtr replanner = std::make_shared<pathplan::MARSHA>(configuration_replan_,current_path_,time_for_repl,
                                                                     solver_,other_paths_,ha_metrics_);

  replanner->reverseStartNodes(reverse_start_nodes_);

  if(full_net_search_)
    ROS_WARN("full net search not available for MARSHA");

  full_net_search_ = false;
  replanner->setFullNetSearch(full_net_search_);

  replanner_ = replanner;

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_cc_,group_name_);
  replanner_->setDisp(disp);
}

Eigen::Matrix <double,3,Eigen::Dynamic> ReplannerManagerMARSHA::updateObstaclesPositions(const moveit_msgs::PlanningSceneWorld& world)
{
  double obs_x, obs_y, obs_z;
  Eigen::Vector3d obstacle_position;
  Eigen::Matrix <double,3,Eigen::Dynamic> obstacles_positions;

  for(const moveit_msgs::CollisionObject& obj:world.collision_objects)
  {
    if(std::find(unaware_obstacles_.begin(),unaware_obstacles_.end(),obj.id)>=unaware_obstacles_.end())
    {
      for(const geometry_msgs::Pose& primitive_pose: obj.primitive_poses)
      {
        obs_x = obj.pose.position.x + primitive_pose.position.x;
        obs_y = obj.pose.position.y + primitive_pose.position.y;
        obs_z = obj.pose.position.z + primitive_pose.position.z;

        obstacle_position << obs_x,obs_y,obs_z;

        obstacles_positions.conservativeResize(Eigen::NoChange, obstacles_positions.cols()+1);
        obstacles_positions.col(obstacles_positions.cols()-1) = obstacle_position;
      }
    }
  }

  return obstacles_positions;
}

void ReplannerManagerMARSHA::setSharedMetrics()
{
  //Euclidean metrics to the current path shared
  current_path_shared_->setMetrics(metrics_shared_);

  //Create new metrics if in the meanwhile a new path was added
  int new_paths = other_paths_shared_.size()-other_metrics_shared_.size();
  assert(new_paths>=0);

  for(int i=0;i<new_paths;i++)
    other_metrics_shared_.push_back(std::make_shared<Metrics>());

  //Set euclidean metrics to each shared path
  for(unsigned int i=0;i<other_paths_shared_.size();i++)
    other_paths_shared_[i]->setMetrics(other_metrics_shared_[i]);
}

void ReplannerManagerMARSHA::updateSharedPath()
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

  setSharedMetrics(); //The difference from the method of ReplanningManagerMARS
  other_paths_mtx_.unlock();
}

void ReplannerManagerMARSHA::collisionCheckThread()
{
  moveit_msgs::GetPlanningScene ps_srv;
  Eigen::VectorXd current_configuration_copy;
  moveit_msgs::PlanningScene planning_scene_msg;
  Eigen::Matrix <double,3,Eigen::Dynamic> obstacles_positions;

  ssm15066_estimator::SSM15066Estimator2DPtr current_path_ssm = std::make_shared<ssm15066_estimator::SSM15066Estimator2D>(ssm_->getChain()->clone());
  current_path_ssm->setMaxCartAcc   (ssm_->getMaxCartAcc   (),false);
  current_path_ssm->setMinDistance  (ssm_->getMinDistance  (),false);
  current_path_ssm->setReactionTime (ssm_->getReactionTime (),false);
  current_path_ssm->setHumanVelocity(ssm_->getHumanVelocity(),false);
  current_path_ssm->setPoiNames     (ssm_->getPoiNames     ());
  current_path_ssm->setMaxStepSize  (ssm_->getMaxStepSize  ());
  current_path_ssm->updateMembers();

  LengthPenaltyMetricsPtr metrics_current_path = std::make_shared<LengthPenaltyMetrics>(current_path_ssm);

  PathPtr current_path_copy = current_path_shared_->clone();
  current_path_copy->setChecker(checker_cc_);
  current_path_copy->setMetrics(metrics_current_path);

  std::vector<PathPtr> other_paths_copy;
  std::vector<CollisionCheckerPtr> other_checkers;
  std::vector<LengthPenaltyMetricsPtr> other_metrics;
  std::vector<ssm15066_estimator::SSM15066EstimatorPtr> other_ssm;

  ssm15066_estimator::SSM15066EstimatorPtr tmp_ssm;
  for(const PathPtr& p:other_paths_shared_)
  {
    PathPtr path_copy = p->clone();

    other_checkers.push_back(checker_cc_->clone());

    tmp_ssm = std::static_pointer_cast<ssm15066_estimator::SSM15066Estimator>(current_path_ssm->clone());
    other_ssm.push_back(tmp_ssm);    other_metrics.push_back(std::make_shared<LengthPenaltyMetrics>(other_ssm.back()));

    path_copy->setChecker(other_checkers.back());
    path_copy->setMetrics(other_metrics.back());

    other_paths_copy.push_back(path_copy);
  }

  int other_path_size = other_paths_copy.size();

  ros::WallRate lp(collision_checker_thread_frequency_);
  ros::WallTime tic;

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

    obstacles_positions = updateObstaclesPositions(planning_scene_msg.world);
    //    ROS_INFO_STREAM("obs rows "<<obstacles_positions.rows()<<" cols "<<obstacles_positions.cols());

    checker_cc_->setPlanningSceneMsg(planning_scene_msg);
    current_path_ssm->setObstaclesPositions(obstacles_positions);

    for(unsigned int i=0;i<other_checkers.size();i++)
    {
      other_checkers.at(i)->setPlanningSceneMsg(planning_scene_msg);
      other_ssm.at(i)->setObstaclesPositions(obstacles_positions);
    }
    scene_mtx_.unlock();

    /* Update paths if they have been changed */
    trj_mtx_.lock();
    paths_mtx_.lock();

    current_configuration_copy = current_configuration_;

    if(current_path_sync_needed_)
    {
      current_path_copy = current_path_shared_->clone();
      current_path_copy->setChecker(checker_cc_);
      current_path_copy->setMetrics(metrics_current_path);
      current_path_sync_needed_ = false;
    }

    other_paths_mtx_.lock();
    if(other_path_size<other_paths_shared_.size())  // if the previous current path has been added, update the vector of copied paths
    {
      assert(other_path_size == (other_paths_shared_.size()-1));

      PathPtr path_copy = other_paths_shared_.back()->clone();

      other_checkers.push_back(checker_cc_->clone());

      tmp_ssm = std::static_pointer_cast<ssm15066_estimator::SSM15066Estimator>(current_path_ssm->clone());
      other_ssm.push_back(tmp_ssm);
      other_metrics.push_back(std::make_shared<LengthPenaltyMetrics>(other_ssm.back()));

      path_copy->setChecker(other_checkers.back());
      path_copy->setMetrics(other_metrics.back());

      other_paths_copy.push_back(path_copy);
      other_path_size = other_paths_copy.size();
    }

    for(unsigned int i=0;i<other_paths_shared_.size();i++)  // sync other_paths_shared with its copy
    {
      if(other_paths_sync_needed_.at(i))
      {
        other_paths_copy.at(i) = other_paths_shared_.at(i)->clone();
        other_paths_copy.at(i)->setChecker(other_checkers.at(i));
        other_paths_copy.at(i)->setMetrics(other_metrics.at(i));
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
                                 &ReplannerManagerMARSHA::checkPathTask,
                                 this,other_paths_copy.at(i)));
    }

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
    if(uploadPathsCost(current_path_copy,other_paths_copy)) //if path cost can be updated, update also the planning scene used to check the path
    {
      planning_scene_msg_.world = ps_srv.response.scene.world;  //not diff,it contains all pln scn info but only world is updated
      planning_scene_diff_msg_ = planning_scene_msg;            //diff, contains only world
      obstalces_positions_ = obstacles_positions;

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

bool ReplannerManagerMARSHA::replan()
{
  assert([&]() ->bool{
           for(PathPtr& p:other_paths_)
           {
             if(p->getMetrics() != ha_metrics_)
             return false;
           }
           if(current_path_->getMetrics() != ha_metrics_)
           return false;

           return true;
         }());

  return ReplannerManagerMARS::replan();
}

}
