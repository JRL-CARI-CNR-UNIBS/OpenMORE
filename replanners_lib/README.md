# **replanners_lib**

The package **replanners_lib** contains the implementation of a library of sample-based path replanning algorithms and a framework to manage trajectory execution with continuous path replanning.
It contains two main repositories:
1. **replanners**: contains the implementation of some sample-based replanning algorithms.
2. **replanner_managers**: contains the implementation of a framework to manage the trajectory execution with continuous replanning for each of the available replanners.

 The replanner managers take care of the robot trajectory execution with continuous replanning using one of the replanner implemented.

 This is a list of the implemented replanners:

 1. MARS
 2. [DRRT](https://ieeexplore.ieee.org/document/1641879)
 3. [Anytime DRRT](https://ieeexplore.ieee.org/document/4209270)
 4. [DRRT*](https://ieeexplore.ieee.org/document/8122814)
 5. [MPRRT](https://ieeexplore.ieee.org/document/7027233)

This library depends on [*graph_core*](https://github.com/JRL-CARI-CNR-UNIBS/cari_motion_planning/tree/cesare-devel/graph_core), which contains definition of the necessary classes of a path planning problem and the path planning solvers. Warning: in order to use **replanners_lib**, the branch of graph_core must be *cesare-devel*.

## replanners
It contains the abstract class *replanner_base.cpp* from which you need to inherit to implement your replanner.
Mainly, you have to implement the pure virtual function
```cpp
virtual bool replan() = 0;
```
which is in charge to run the replanning algorithm. Some useful and general functions are already present in replanner_base.cpp, check it out.

This is a brief explanation to create a replanner object. In this example we will use DRRT.
You need to include:
```cpp
#include <replanners_lib/DRRT.h>
#include <replanners_lib/trajectory.h>
```
Define the robot model and the planning scene:
```cpp
moveit::planning_interface::MoveGroupInterface move_group(group_name);
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

unsigned int dof = joint_names.size();
Eigen::VectorXd lb(dof);
Eigen::VectorXd ub(dof);

for (unsigned int idx = 0; idx < dof; idx++)
{
  const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
  if (bounds.position_bounded_)
  {
    lb(idx) = bounds.min_position_;
    ub(idx) = bounds.max_position_;
  }
}
```

Then, create the initial path:
```cpp
pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

Eigen::VectorXd start_conf;  //NB: INITIALIZE THIS VALUE WITH THE PATH START CONFIGURATION
Eigen::VectorXd goal_conf;   //NB: INITIALIZE THIS VALUE WITH THE PATH STOP CONFIGURATION

pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler); //NB: you can chose a different planner among those of graph_core
pathplan::PathPtr current_path = trajectory.computePath(start_conf, goal_conf,solver,optimize); //optimize = true to optimize with RRT* rewire and shortcutting
```
Define the current robot configuration, for example:
```cpp
 Eigen::VectorXd current_configuration = current_path->getConnections.at(0)->getChild()->getConfiguration();
```
Finally, create the replanner and replan:
```cpp
pathplan::DynamicRRTPtr replanner = std::make_shared<pathplan::DynamicRRT>(current_configuration,current_path,max_time,solver); //max_time is the maximum time for replanning
replanner->replan();

if(replanner->getSuccess())
  pathplan::PathPtr replanned_path = replanner->getReplannedPath();
```
You can find a complete example code [here] FAIIIIIIIIIIIIIIIIIIIIIIIIIIIIII.

## replanner_managers
The replanner manager manages the whole motion from a starting robot configuration to a goal configuration in a dynamic environment. It interpolates the robot trajectory, sends the new robot states and continuously calls the replanner to avoid obstacles or to optimize the current path. To do this, it uses three threads:
- `trajectory execution thread`: interpolates and executes the trajectory.
- `collision checking thread`: continuously updates the planning scene to check if the current path is colliding and to provide updated information about the state of the environment to the replanner.
- `replanning thread`: it continuously tries to find new paths from the robot current configuration to avoid obstacles or to optimizes the current one calling `replan` function.

The replanning framework listens to three speed overrides topics, `/speed_ovr`, `/safe_ovr_1` and `/safe_ovr_2` (but you can choose different ones), in which the overrides must be published as values between 0 (robot static) and 100 (nominal velocity).

It subscribes a service (`moveit_msgs::GetPlanningScene`) to update the information about the planning scene.
It continuously interpolates the trajectory computed starting from the replanned path and sends the new robot state, publishing a `sensor_msgs::JointState` message on topic `/joint_target`, and the unscaled state on topic `/unscaled_joint_target`.

Here you can find a complete description of parameters required.

This is a brief explanation to create a replanner manager object. In this example we will consider the manager of DRRT.
You need to include:
```cpp
#include <replanners_lib/replanner_manager_DRRT.h>
```
Define the robot model and the planning scene:
```cpp
moveit::planning_interface::MoveGroupInterface move_group(group_name);
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

unsigned int dof = joint_names.size();
Eigen::VectorXd lb(dof);
Eigen::VectorXd ub(dof);

for (unsigned int idx = 0; idx < dof; idx++)
{
  const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
  if (bounds.position_bounded_)
  {
    lb(idx) = bounds.min_position_;
    ub(idx) = bounds.max_position_;
  }
}
```

Then, create the initial path:
```cpp
pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

Eigen::VectorXd start_conf;  //NB: INITIALIZE THIS VALUE WITH THE PATH START CONFIGURATION
Eigen::VectorXd goal_conf;   //NB: INITIALIZE THIS VALUE WITH THE PATH STOP CONFIGURATION

pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics, checker, sampler);
pathplan::PathPtr current_path = trajectory.computePath(start_conf, goal_conf,solver,optimize); //optimize = true to optimize with RRT* rewire and shortcutting
```
Finally, create the replanner manager and start the execution:
```cpp
pathplan::ReplannerManagerDRRTPtr replanner_manager = std::make_shared<pathplan::ReplannerManagerDRRT>(current_path, solver, nh);
replanner_manager->start();
```
You can find a complete example code FAIIIIIIIIIIIIIIIIIIIIIIIIIIIII
