#ifndef AIPRO_H__
#define AIPRO_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/graph/net.h>

#define TIME_PERCENTAGE_VARIABILITY 0.7

namespace pathplan
{
struct node_and_path
{
  NodePtr node;
  PathPtr path;
};

struct unconnected_nodes
{
  NodePtr start_node;
  std::vector<node_and_path> goals_and_paths;
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

  std::vector<PathPtr> addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path);
  std::vector<node_and_path> sortNodesOnDistance(const NodePtr& node);
  std::vector<NodePtr> startNodes(const std::vector<ConnectionPtr>& subpath1_conn);
  PathPtr getSubpath1(const ConnectionPtr& current_conn, NodePtr& current_node);
  PathPtr searchForExistingSolutions(const PathPtr& subpath1, const std::vector<PathPtr> &reset_other_paths);
  PathPtr bestExistingSolution(const PathPtr& subpath1);
  double maxSolverTime(const ros::WallTime& tic, const ros::WallTime& tic_cycle);
  void optimizePath(PathPtr &connecting_path, const double &max_time);
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost, const PathPtr &current_solution, const ros::WallTime &tic, const ros::WallTime &tic_cycle, PathPtr &connecting_path, bool &quickly_solved);
  void simplifyAdmissibleOtherPaths(const PathPtr& current_solution_path, const NodePtr &start_node, const std::vector<PathPtr>& reset_other_paths);
  bool mergePathToTree(PathPtr& path);


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

  virtual void setVerbosity(const bool& verbose)
  {
    verbose_ = verbose;
    informedOnlineReplanning_verbose_ = verbose;
    pathSwitch_verbose_ = verbose;
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
    pathSwitch_disp_ = verbose;
  }

  void setCurrentPath(const PathPtr& path) override
  {
    current_path_ = path;
    tree_ = current_path_->getTree();
    net_->setTree(tree_);
    admissible_other_paths_ = other_paths_;
    success_ = false;
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
    success_ = false;
  }

  void setCurrentConf(const Eigen::VectorXd& q) override
  {
    current_configuration_ = q;
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

  bool simplifyReplannedPath(const double& distance);
  bool pathSwitch(const PathPtr& current_path, const NodePtr& path1_node, PathPtr &new_path);
  bool informedOnlineReplanning(const double &max_time  = std::numeric_limits<double>::infinity());

  bool replan() override;
};
}

#endif // AIPRO_H
