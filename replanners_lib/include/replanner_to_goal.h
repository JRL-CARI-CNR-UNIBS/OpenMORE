#ifndef REPLANNERTOGOAL_H__
#define REPLANNERTOGOAL_H__
#include <replanner_base.h>

namespace pathplan
{
class ReplannerToGoal;
typedef std::shared_ptr<ReplannerToGoal> ReplannerToGoalPtr;

class ReplannerToGoal: public ReplannerBase
{
protected:

  PathPtr concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn, const NodePtr& path1_node);
  bool connect2goal(const NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerToGoal(Eigen::VectorXd& current_configuration,
                  PathPtr& current_path,
                  const double& max_time,
                  const TreeSolverPtr& solver): ReplannerBase(current_configuration,current_path,max_time,solver){};

  bool replan() override;
};
}

#endif // REPLANNERTOGOAL_H
