#ifndef REPLANNER_MANAGER_BASE_H__
#define REPLANNER_MANAGER_BASE_H__

#include <mutex>
#include <thread>
#include <std_msgs/Int64.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/filesystem.hpp>
#include <replanners_lib/trajectory.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <replanners_lib/replanners/replanner_base.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <subscription_notifier/subscription_notifier.h>
#include <graph_core/parallel_moveit_collision_checker.h>

namespace pathplan
{
#define K_OFFSET 1.3
class ReplannerManagerBase;
typedef std::shared_ptr<ReplannerManagerBase> ReplannerManagerBasePtr;

class ReplannerManagerBase: public std::enable_shared_from_this<ReplannerManagerBase>
{
protected:

  /* To be assigned by the constructor */
  double               trj_exec_thread_frequency_         ;
  double               collision_checker_thread_frequency_;
  double               dt_replan_                         ;
  PathPtr              current_path_                      ;
  PathPtr              current_path_shared_               ;
  std::string          group_name_                        ;
  TreeSolverPtr        solver_                            ;
  ros::NodeHandle      nh_                                ;

  /* Global variables */
  bool stop_                      ;
  bool benchmark_                 ;
  bool goal_reached_              ;
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

  int spline_order_              ;
  int parallel_checker_n_threads_;

  double t_                          ;
  double dt_                         ;
  double real_time_                  ;
  double obj_max_size_               ;
  double time_shift_                 ;
  double t_replan_                   ;
  double t_replan_used_              ; //elimina
  double t_used_                     ; // elimina
  double replanning_time_            ;
  double replanning_thread_frequency_;
  double scaling_from_param_         ;
  double checker_resolution_         ;
  double goal_tol_                   ;
  double scaling_                    ;
  double global_override_            ;

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
  moveit_msgs::PlanningScene                planning_scene_diff_msg_ ;


  std::string obj_type_                ;
  std::vector<double> spawn_instants_  ;
  std::vector<std::string> obj_ids_    ;
  std::vector<Eigen::VectorXd> obj_pos_;

  std::thread display_thread_   ;
  std::thread trj_exec_thread_  ;
  std::thread col_check_thread_ ;
  std::thread spawn_obj_thread_ ;
  std::thread benchmark_thread_ ;
  std::thread replanning_thread_;

  std::mutex trj_mtx_      ;
  std::mutex paths_mtx_    ;
  std::mutex scene_mtx_    ;
  std::mutex replanner_mtx_;
  std::mutex ovr_mtx_      ;
  std::mutex bench_mtx_    ;

  std::vector<std::string>                                                        scaling_topics_names_ ;
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>>> scaling_topics_vector_;
  std::map<std::string,double> overrides_;

  ros::Publisher target_pub_         ;
  ros::Publisher text_overlay_pub_   ;
  ros::Publisher unscaled_target_pub_;

  std::string joint_target_topic_         ;
  std::string unscaled_joint_target_topic_;

  ros::ServiceClient add_obj_               ;
  ros::ServiceClient remove_obj_            ;
  ros::ServiceClient plannning_scene_client_;

  virtual void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);
  virtual void subscribeTopicsAndServices();
  virtual bool replan();
  virtual void fromParam();
  virtual void syncPathCost();
  virtual void updateSharedPath();
  virtual void updateTrajectory();
  virtual void updatePathCost(const PathPtr& current_path_updated_copy);
  virtual void attributeInitialization();
  virtual void replanningThread();
  virtual void collisionCheckThread();
  virtual void displayThread();
  virtual void benchmarkThread();
  virtual void spawnObjectsThread();
  virtual void trajectoryExecutionThread();
  virtual double readScalingTopics();
  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const MoveitUtils& util);
  Eigen::Vector3d forwardIk(const Eigen::VectorXd& conf, const std::string& last_link, const MoveitUtils& util, geometry_msgs::Pose &pose);

  virtual void initReplanner()=0;
  virtual bool haveToReplan(const bool path_obstructed)=0;

  bool alwaysReplan()
  {
    return true;
  }

  bool replanIfObstructed(const bool path_obstructed)
  {
    return path_obstructed;
  }

  void displayTrj(); //elimina

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerBase(const PathPtr &current_path,
                       const TreeSolverPtr &solver,
                       const ros::NodeHandle &nh);
  ~ReplannerManagerBase();

  void setGroupName(const std::string& group_name)
  {
    group_name_ = group_name;
  }

  void setGoalToll(const double& toll)
  {
    goal_tol_ = toll;
  }

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

  void setCurrentPath(const PathPtr& current_path)
  {
    current_path_ = current_path;
  }

  ReplannerBasePtr getReplanner()
  {
    return replanner_;
  }

  bool goalReached()
  {
    return goal_reached_;
  }

  virtual bool joinThreads();
  virtual bool stop();
  virtual bool run();
  virtual bool start();
  virtual bool startWithoutReplanning();

  virtual void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration)=0;
};

}

#endif // REPLANNER_MANAGER_BASE_H__
