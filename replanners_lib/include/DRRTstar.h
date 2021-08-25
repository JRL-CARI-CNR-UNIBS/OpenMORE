#ifndef DRRTSTAR_H__
#define DRRTSTAR_H__
#include <replanner_base.h>
#include <graph_core/solvers/rrt_star.h>

namespace pathplan
{
class DynamicRRTstar;
typedef std::shared_ptr<DynamicRRTstar> DynamicRRTstarPtr;

class DynamicRRTstar: public ReplannerBase
{
protected:

  bool nodeBehindObs(NodePtr& node_behind);
  bool connectBehindObs(NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicRRTstar(Eigen::VectorXd& current_configuration,
                 PathPtr& current_path,
                 const double& max_time,
                 const RRTStarPtr& solver): ReplannerBase(current_configuration,current_path,max_time,solver){};

  bool replan() override;
};
}

#endif // DRRTSTAR_H
