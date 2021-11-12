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
#define K_OFFSET 1.5
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
  std::string          base_link_                         ;
  std::string          last_link_                         ;
  TreeSolverPtr        solver_                            ;
  ros::NodeHandle      nh_                                ;

  // Global variables
  bool stop_                      ;
  bool path_obstructed_           ;
  bool computing_avoiding_path_   ;
  bool spawn_objs_                ;
  bool replanning_                ;
  bool display_timing_warning_    ;
  bool display_replanning_success_;
  bool read_safe_scaling_         ;
  bool current_path_changed_      ;

  int n_conn_;

  double real_time_                  ;
  double t_                          ;
  double dt_                         ;
  double replan_offset_              ;
  double t_replan_                   ;
  double replanning_thread_frequency_;
  double scaling_from_param_         ;
  double checker_resol_              ;
  double goal_toll_                  ;
  double scaling_                    ;

  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  ReplannerBasePtr                          replanner_               ;
  Eigen::VectorXd                           current_configuration_   ;
  Eigen::VectorXd                           configuration_replan_    ;
  CollisionCheckerPtr                       checker_cc_              ;
  CollisionCheckerPtr                       checker_replanning_      ;
  TrajectoryPtr                             trajectory_              ;
  std::vector<ConnectionPtr>                added_branch_            ;
  NodePtr                                   path_start_              ;
  NodePtr                                   old_path_start_          ;
  NodePtr                                   root_for_next_detach_    ;
  NodePtr                                   root_for_detach_         ;
  planning_scene::PlanningScenePtr          planning_scn_cc_         ;
  planning_scene::PlanningScenePtr          planning_scn_replanning_ ;
  trajectory_processing::SplineInterpolator interpolator_            ;
  trajectory_msgs::JointTrajectoryPoint     pnt_                     ;
  trajectory_msgs::JointTrajectoryPoint     pnt_unscaled_            ;
  trajectory_msgs::JointTrajectoryPoint     pnt_replan_              ;
  sensor_msgs::JointState                   new_joint_state_unscaled_;
  sensor_msgs::JointState                   new_joint_state_         ;

  std::thread display_thread_   ;
  std::thread spawn_obj_thread_ ;
  std::thread replanning_thread_;
  std::thread col_check_thread_ ;
  std::thread trj_exec_thread_  ;

  std::mutex planning_mtx_ ;
  std::mutex checker_mtx_  ;
  std::mutex trj_mtx_      ;
  std::mutex paths_mtx_    ;
  std::mutex scene_mtx_    ;
  std::mutex replanner_mtx_;
  std::mutex stop_mtx_     ;
  std::mutex ovr_mtx_      ;

  std::vector<std::string>                                                        scaling_topics_names_ ;
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>>> scaling_topics_vector_;
  std::map<std::string,double> overrides_;
  double global_override_;

  ros::Publisher target_pub_         ;
  ros::Publisher unscaled_target_pub_;

  ros::ServiceClient plannning_scene_client_;
  ros::ServiceClient add_obj_               ;
  ros::ServiceClient remove_obj_            ;

  void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);
  void fromParam()                 ;
  void attributeInitialization()   ;
  void subscribeTopicsAndServices();
  void replanningThread()          ;
  void collisionCheckThread()      ;
  void displayThread()             ;
  void spawnObjects()              ;
  void trajectoryExecutionThread() ;
  double readScalingTopics()       ;
  std::vector<ConnectionPtr> connectCurrentConfToTree()  ;
  bool detachAddedBranch(std::vector<NodePtr>& nodes, std::vector<double>& costs);

  virtual bool replan()=0                ;
  virtual void initReplanner()=0         ;
  virtual void connectToReplannedPath()=0;
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

  ReplannerManagerBase(PathPtr &current_path,
                       TreeSolverPtr &solver,
                       ros::NodeHandle &nh);

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

  bool run()                                                                                      ;
  bool start()                                                                                    ;
  bool startWithoutReplanning()                                                                   ;
  bool cancel()                                                                                   ;
  bool stop()                                                                                     ;
  void setChainProperties(std::string &group_name, std::string &base_link, std::string &last_link);
};

}

#endif // REPLANNER_MANAGER_BASE_H__
