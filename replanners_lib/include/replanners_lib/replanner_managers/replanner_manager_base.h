#ifndef REPLANNER_MANAGER_BASE_H__
#define REPLANNER_MANAGER_BASE_H__

#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <subscription_notifier/subscription_notifier.h>
#include <thread>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Empty.h>
#include <boost/variant.hpp>
#include <replanners_lib/replanners/replanner_base.h>

namespace pathplan
{
#define K_OFFSET 1.1
class ReplannerManagerBase;
typedef std::shared_ptr<ReplannerManagerBase> ReplannerManagerBasePtr;

class ReplannerManagerBase: public std::enable_shared_from_this<ReplannerManagerBase>
{
protected:

  // To be assigned by the constructor
  double               trj_exec_thread_frequency_         ;
  double               collision_checker_thread_frequency_;
  double               dt_replan_                         ;
  PathPtr              current_path_replanning_           ;
  PathPtr              current_path_shared_               ;
  std::string          group_name_                        ;
  TreeSolverPtr        solver_                            ;
  ros::NodeHandle      nh_                                ;

  // Global variables
  bool stop_                      ;
  bool spawn_objs_                ;
  bool read_safe_scaling_         ;
  bool replanner_verbosity_       ;
  bool display_replan_config_     ;
  bool display_current_config_    ;
  bool display_timing_warning_    ;
  bool display_replan_trj_point_  ;
  bool current_path_sync_needed_  ;
  bool display_current_trj_point_ ;
  bool display_replanning_success_;

  int n_conn_;

  double real_time_                     ;
  double t_                             ;
  double dt_                            ;
  double replan_offset_                 ;
  double t_replan_                      ;
  double replanning_thread_frequency_   ;
  double scaling_from_param_            ;
  double checker_resolution_            ;
  double goal_tol_                      ;
  double scaling_                       ;
  double abscissa_current_configuration_;

  ReplannerBasePtr                          replanner_               ;
  Eigen::VectorXd                           current_configuration_   ;
  Eigen::VectorXd                           configuration_replan_    ;
  CollisionCheckerPtr                       checker_cc_              ;
  CollisionCheckerPtr                       checker_replanning_      ;
  TrajectoryPtr                             trajectory_              ;
  NodePtr                                   path_start_              ;
  planning_scene::PlanningScenePtr          planning_scn_cc_         ;
  planning_scene::PlanningScenePtr          planning_scn_replanning_ ;
  trajectory_processing::SplineInterpolator interpolator_            ;
  trajectory_msgs::JointTrajectoryPoint     pnt_                     ;
  trajectory_msgs::JointTrajectoryPoint     pnt_unscaled_            ;
  trajectory_msgs::JointTrajectoryPoint     pnt_replan_              ;
  sensor_msgs::JointState                   new_joint_state_unscaled_;
  sensor_msgs::JointState                   new_joint_state_         ;
  moveit_msgs::PlanningScene                planning_scene_msg_      ;

  std::thread display_thread_;
  std::thread spawn_obj_thread_;
  std::thread replanning_thread_;
  std::thread col_check_thread_;
  std::thread trj_exec_thread_;

  std::mutex trj_mtx_;
  std::mutex paths_mtx_;
  std::mutex scene_mtx_;
  std::mutex replanner_mtx_;
  std::mutex ovr_mtx_;

  std::vector<std::string>                                                        scaling_topics_names_ ;
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>>> scaling_topics_vector_;
  std::map<std::string,double> overrides_;
  double global_override_;

  ros::Publisher target_pub_;
  ros::Publisher unscaled_target_pub_;

  ros::ServiceClient plannning_scene_client_;
  ros::ServiceClient add_obj_;
  ros::ServiceClient remove_obj_;

  void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);
  void subscribeTopicsAndServices();
  virtual bool replan();
  virtual void fromParam();
  virtual void syncPathCost();
  virtual void updatePathCost(const PathPtr& current_path_updated_copy);
  virtual void attributeInitialization();
  virtual void replanningThread();
  virtual void collisionCheckThread();
  virtual void displayThread();
  void spawnObjects();
  void trajectoryExecutionThread();
  double readScalingTopics();

  virtual void initReplanner()=0         ;
  virtual bool haveToReplan(const bool path_obstructed)=0;

  bool alwaysReplan()
  {
    return true;
  }

  bool replanIfObstructed(const bool path_obstructed)
  {
    return path_obstructed;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerBase(const PathPtr &current_path,
                       const TreeSolverPtr &solver,
                       const ros::NodeHandle &nh);

  trajectory_msgs::JointTrajectoryPoint getJointTarget()
  {
    trj_mtx_.lock();
    trajectory_msgs::JointTrajectoryPoint pnt = pnt_;
    trj_mtx_.unlock();

    return pnt;
  }

  trajectory_msgs::JointTrajectoryPoint getUnscaledJointTarget()
  {
    trj_mtx_.lock();
    trajectory_msgs::JointTrajectoryPoint pnt_unscaled = pnt_unscaled_;
    trj_mtx_.unlock();

    return pnt_unscaled;
  }

  ReplannerBasePtr getReplanner()
  {
    return replanner_;
  }

  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)=0;

  virtual bool stop();
  virtual bool cancel();
  virtual bool run();
  virtual bool start();
  virtual bool startWithoutReplanning();
};

}

#endif // REPLANNER_MANAGER_BASE_H__
