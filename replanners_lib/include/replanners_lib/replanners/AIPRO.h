#ifndef AIPRO_H__
#define AIPRO_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/graph/net_connection.h>
#include <graph_core/graph/net.h>

#define TIME_PERCENTAGE_VARIABILITY 0.7

namespace pathplan
{
struct unconnected_nodes
{
  NodePtr start_node;
  std::vector<NodePtr> goal_nodes;

  void removeGoal(const NodePtr& goal)
  {
    std::vector<NodePtr>::iterator it = std::find(goal_nodes.begin(), goal_nodes.end(), goal);

    if(it != goal_nodes.end())
      goal_nodes.erase(it);
    else
    {
      ROS_INFO("This goal is not member of the unconnected goal nodes vector");
      ROS_INFO_STREAM("Goal node: "<<*goal<<"\nStart node: "<<*start_node);
      for(const NodePtr& n:goal_nodes)
        ROS_INFO_STREAM("g: "<<n->getConfiguration().transpose());
    }
  }
};

class AIPRO;
typedef std::shared_ptr<AIPRO> AIPROPtr;

class AIPRO: public ReplannerBase
{
protected:

  TreePtr tree_;
  NetPtr net_;
  std::vector<PathPtr> replanned_paths_vector_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> admissible_other_paths_;
  std::vector<NodePtr> examined_nodes_;
  std::vector<unconnected_nodes> unconnected_nodes_;

  double time_first_sol_;
  double time_replanning_;
  double available_time_;
  double pathSwitch_max_time_;
  double pathSwitch_cycle_time_mean_;
  double time_percentage_variability_;

  int pathSwitch_path_id_;

  bool an_obstacle_;
  bool informedOnlineReplanning_disp_;
  bool pathSwitch_disp_;
  bool informedOnlineReplanning_verbose_;
  bool pathSwitch_verbose_;

  //It finds the portion of current_path_ between the obstacle and the goal and add it as first element of a vector containing the other available paths. It is used in InformedOnlineReplanning
  std::vector<PathPtr> addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path);

  // It sorts the admissible other paths depending on the distance of the closest node from node. Use in PathSwitch
  std::vector<PathPtr> sortPathsOnDistance(const NodePtr& node);

  //It finds the set of nodes of path to try to connect to starting from node. The goal is excluded. Used in PathSwitch.
  std::vector<NodePtr> nodes2connect2(const PathPtr& path, const NodePtr &this_node);

  //It fills the vector of nodes from which starting PathSwitch and set available_nodes flag. It is used in informedOnlineReplanning.
  std::vector<NodePtr> startingNodesForPathSwitch(const std::vector<ConnectionPtr>& subpath1_conn, const NodePtr& current_node, const double& current2child_conn_cost, const int& idx,  bool& available_nodes);

  //It simplifies the admissible_other_paths vector substituting the path on which the current starting node for PathSwitch resides with the subpath from that node. It is used in InformedOnlineReplanning
  void simplifyAdmissibleOtherPaths(const bool& no_available_paths, const PathPtr& confirmed_subpath_from_path2, const int& confirmed_connected2path_number, const NodePtr &starting_node_of_pathSwitch, const std::vector<PathPtr>& reset_other_paths);

  //It computes the time constraint for PathSwitch & Connect2Goal.
  double maxSolverTime(const ros::WallTime& tic, const ros::WallTime& tic_cycle);

  //  //FAI           It concatenates the connecting path with the subpath2. It is used in PathSwitch & Connect2Goal.
  PathPtr concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn, const std::vector<ConnectionPtr>& subpath2);

  //  //FAI           It compute the connecting path from path1_node to path2_node. It is used in PathSwitch and Connect2Goal.
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost, const ros::WallTime &tic, const ros::WallTime &tic_cycle, PathPtr &connecting_path, bool &direct_connection);

  //Optimize connecting path. used in PathSwitch and Connect2Goal.
  void optimizePath(PathPtr &connecting_path, const double &max_time);

  bool mergePathToTree(PathPtr& path);

  PathPtr existingSolutions(const NodePtr &start_node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AIPRO(const Eigen::VectorXd& current_configuration,
        const PathPtr& current_path,
        const double& max_time,
        const TreeSolverPtr &solver);

  AIPRO(const Eigen::VectorXd& current_configuration,
        const PathPtr& current_path,
        const double& max_time,
        const TreeSolverPtr &solver,
        std::vector<PathPtr> &other_paths);


  std::vector<PathPtr> getReplannedPathVector()
  {
    return replanned_paths_vector_;
  }

  std::vector<PathPtr> getOtherPaths()
  {
    return other_paths_;
  }

  void setInformedOnlineReplanningVerbose(const bool verbose)
  {
    informedOnlineReplanning_verbose_ = verbose;
  }

  void setPathSwitchVerbose(const bool verbose)
  {
    pathSwitch_verbose_ = verbose;
  }

  void setInformedOnlineReplanningDisp(const bool verbose)
  {
    informedOnlineReplanning_disp_ = verbose;
  }

  void setPathSwitchDisp(const bool verbose)
  {
    pathSwitch_disp_ =verbose;
  }

  void setCurrentPath(const PathPtr& path) override
  {
    current_path_ = path;
    tree_ = current_path_->getTree();
    net_->setTree(tree_);
    admissible_other_paths_ = other_paths_;
    examined_nodes_.clear();
    success_ = false;

    ROS_INFO("Current path set");
  }

  void setOtherPaths(std::vector<PathPtr> &other_paths)
  {
    other_paths_.clear();

    for(PathPtr& path:other_paths)
    {
      if(!mergePathToTree(path))
        assert(0);

      other_paths_.push_back(path);
    }

    admissible_other_paths_ = other_paths_;
    examined_nodes_.clear();
    success_ = false;
  }

  void setCurrentConf(const Eigen::VectorXd& q) override
  {
    current_configuration_ = q;
    examined_nodes_.clear();
    success_ = false;
  }

  void addOtherPath(PathPtr& path)
  {
    if(!mergePathToTree(path))
      assert(0);

    other_paths_.push_back(path);
  }

  ReplannerBasePtr pointer()
  {
    return shared_from_this();
  }

  void setAvailableTime(const double &time)
  {
    available_time_ = time;
  }

  bool simplifyReplannedPath(const double& distance);

  //  //FAI     It directly connect the node to the goal
  //  bool connect2goal(const PathPtr &current_path, const NodePtr& node, PathPtr &new_path);

  //Starting from node of current_path_ it tries to find a connection to all the available paths of admissible_other_paths_
  bool pathSwitch(const PathPtr& current_path, const NodePtr& node, PathPtr &new_path, PathPtr &subpath_from_path2, int &connected2path_number);

  //  //FAI     It menages the replanning calling more times pathSwitch from different nodes and giving the correct set of available paths
  //  bool informedOnlineReplanning(const double &max_time  = std::numeric_limits<double>::infinity());

  bool replan() override;
};
}

#endif // AIPRO_H
