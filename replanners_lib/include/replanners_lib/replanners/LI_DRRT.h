#ifndef LIDRRT_H__
#define LIDRRT_H__
#include <replanners_lib/replanners/DRRT.h>

namespace pathplan
{
class LazyInformedDRRT;
typedef std::shared_ptr<LazyInformedDRRT> LazyInformedDRRTPtr;

class LazyInformedDRRT: public DynamicRRT
{
protected:
  double cost_from_conf_;

  bool trimInvalidTree(NodePtr& node) override;
  bool regrowRRT(NodePtr& node) override;
  bool improvePath(const double& max_time);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LazyInformedDRRT(Eigen::VectorXd& current_configuration,
                   PathPtr& current_path,
                   const double& max_time,
                   const TreeSolverPtr &solver);

  bool replan() override;
};
}

#endif // LIDRRT_H
