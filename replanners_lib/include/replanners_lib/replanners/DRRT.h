#ifndef DRRT_H__
#define DRRT_H__
#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/solvers/rrt.h>
#include <graph_core/local_informed_sampler.h>
#include <typeinfo>

//Replanning with RRTs

namespace pathplan
{
class DynamicRRT;
typedef std::shared_ptr<DynamicRRT> DynamicRRTPtr;

class DynamicRRT: public ReplannerBase
{
protected:
  TreePtr trimmed_tree_;
  bool tree_is_trimmed_;
  InformedSamplerPtr sampler_;

  bool trimInvalidTree(NodePtr& node, std::vector<ConnectionPtr> &checked_connections);
  bool regrowRRT(NodePtr& node);
  void fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr> &old_nodes, std::vector<double> &old_connections_costs);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver);

  bool getTreeIsTrimmed()
  {
    return tree_is_trimmed_;
  }

  void setCurrentPath(const PathPtr& path) override
  {
    success_ = false;
    current_path_ = path;
    goal_node_  = current_path_->getConnections().back()->getChild();
  }

  virtual bool replan() override;
};
}

#endif // DRRT_H
