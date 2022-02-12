#ifndef REPLANNER_MANAGER_AIPRO_H__
#define REPLANNER_MANAGER_AIPRO_H__

#include <replanners_lib/replanner_managers/replanner_manager_base.h>
#include <replanners_lib/replanners/AIPRO.h>

namespace pathplan
{
class ReplannerManagerAIPRO;
typedef std::shared_ptr<ReplannerManagerAIPRO> ReplannerManagerAIPROPtr;

class ReplannerManagerAIPRO: public ReplannerManagerBase
{
protected:

  void replanningThread() override;
  void collisionCheckThread() override;
  bool haveToReplan(const bool path_obstructed) override;
  void initReplanner() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManagerAIPRO(const PathPtr &current_path,
                        const TreeSolverPtr &solver,
                        const ros::NodeHandle &nh);

//  ReplannerManagerAIPRO(PathPtr &current_path,
//                        std::vector<PathPtr> &other_path,
//                        TreeSolverPtr &solver,
//                        ros::NodeHandle &nh);

  void startReplannedPathFromNewCurrentConf(const Eigen::VectorXd& configuration) override;
};

}

#endif // REPLANNER_MANAGER_AIPRO_H__
