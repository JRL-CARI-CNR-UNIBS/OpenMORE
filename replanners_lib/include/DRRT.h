#ifndef DRRT_H__
#define DRRT_H__
#include <replanner_base.h>
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
  Eigen::VectorXd goal_conf_;
  TreePtr trimmed_tree_;

  bool trimInvalidTree(NodePtr& node);
  bool regrowRRT(NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver);

  bool replan() override;
};
}

#endif // DRRT_H
