#ifndef AIPRO_H__
#define AIPRO_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/graph/net.h>

#define TIME_PERCENTAGE_VARIABILITY 0.7

namespace pathplan
{
struct ps_goals
{
  NodePtr node;
  PathPtr path;
  double utopia;
};

struct invalid_connection
{
  ConnectionPtr connection;
  double cost;
};

class AIPRO;
typedef std::shared_ptr<AIPRO> AIPROPtr;

class AIPRO: public ReplannerBase
{
protected:

  NetPtr net_;
  TreePtr tree_;
  NodePtr paths_start_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> admissible_other_paths_;
  std::vector<ConnectionPtr> checked_connections_;
  std::vector<invalid_connection> invalid_connections_;

  double time_first_sol_;
  double time_replanning_;
  double available_time_;
  double pathSwitch_max_time_;
  double pathSwitch_cycle_time_mean_;
  double time_percentage_variability_;

  int pathSwitch_path_id_;

  bool an_obstacle_;
  bool is_a_new_node_;
  bool at_least_a_trial_;
  bool pathSwitch_disp_;
  bool pathSwitch_verbose_;
  bool informedOnlineReplanning_disp_;
  bool informedOnlineReplanning_verbose_;

  std::vector<double> ps_marker_scale_              = {0.01,0.01,0.01   };
  std::vector<double> ps_marker_color_              = {1.0,0.5,0.0,1.0  };
  std::vector<double> informed_marker_scale_        = {0.01,0.01,0.01   };
  std::vector<double> informed_marker_color_        = {1.0,1.0,0.0,1.0  };
  std::vector<double> ps_marker_color_sphere_       = {0.5,0.5,0.5,1.0  };
  std::vector<double> ps_marker_scale_sphere_       = {0.025,0.025,0.025};
  std::vector<double> informed_marker_scale_sphere_ = {0.03,0.03,0.03   };
  std::vector<double> informed_marker_color_sphere_ = {1.0,0.5,0.0,1.0  };

  std::vector<PathPtr> addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path);
  std::vector<ps_goals> sortNodesOnDistance(const NodePtr& node);
  std::vector<NodePtr> startNodes(const std::vector<ConnectionPtr>& subpath1_conn);
  PathPtr getSubpath1(NodePtr& current_node);
  PathPtr bestExistingSolution(const PathPtr& current_solution);
  PathPtr bestExistingSolution(const PathPtr& current_solution, std::multimap<double, std::vector<ConnectionPtr> > &tmp_map);
  bool findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double& cost2beat, std::vector<ConnectionPtr>& solution, double &cost, bool verbose = false);
  bool findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double& cost2beat, std::vector<ConnectionPtr>& solution, double &cost, unsigned int &number_of_candidates, bool verbose = false);
  double maxSolverTime(const ros::WallTime& tic, const ros::WallTime& tic_cycle);
  void optimizePath(PathPtr &connecting_path, const double &max_time);
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost, const PathPtr &current_solution, const ros::WallTime &tic, const ros::WallTime &tic_cycle, PathPtr &connecting_path, bool &quickly_solved);
  void simplifyAdmissibleOtherPaths(const PathPtr& current_solution_path, const NodePtr &start_node, const std::vector<PathPtr>& reset_other_paths);
  bool mergePathToTree(PathPtr& path);
  void initCheckedConnections();
  void clearInvalidConnections();
  void convertToSubtreeSolution(const PathPtr& net_solution, const std::vector<NodePtr>& black_nodes);

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

  NetPtr getNet()
  {
    return net_;
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

    bool is_a_new_tree = (tree_ != current_path_->getTree());
    tree_ = current_path_->getTree();
    if(is_a_new_tree)
      copyTreeRoot();

    net_->setTree(tree_);
    admissible_other_paths_ = other_paths_;
    goal_node_ = current_path_->getConnections().back()->getChild();
    success_ = false;
  }

  void copyTreeRoot();

  void setOtherPaths(std::vector<PathPtr> &other_paths, bool mergeTree = true)
  {
    other_paths_.clear();

    for(PathPtr& path:other_paths)
      addOtherPath(path,mergeTree);

    admissible_other_paths_ = other_paths_;
    success_ = false;
  }

  void setCurrentConf(const Eigen::VectorXd& q) override
  {
    current_configuration_ = q;
    success_ = false;
  }

  void setChecker(const CollisionCheckerPtr& checker) override
  {
    ReplannerBase::setChecker(checker);
    for(const PathPtr& p:other_paths_)
      p->setChecker(checker);
  }

  void addOtherPath(PathPtr& path, bool mergeTree = true)
  {
    if(mergeTree)
    {
      if(not mergePathToTree(path))
        assert(0);
    }

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
