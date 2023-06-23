<div align="center">
  <h1 align="center">OpenMORE</h1>
  <h3 align="center">
    Open-source MOtion REplanning library
  </h3>
</div>

The repository contains a library of sampling-based path replanning algorithms. It develops a framework to manage robot's trajectory execution with continuous path replanning and collision checking of the current path. It is based on ROS and *MoveIt!* to get information about the environment and collision check. Check [this paper](https://ieeexplore.ieee.org/document/10013661?source=authoralert) for more information.
## Build/Installation
The software can be installed using rosinstall files.

1. Install ROS: follow the steps described in http://wiki.ros.org/ROS/Installation
2. Install Git: follow the steps described in https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
3. Install wstool: follow the steps described in http://wiki.ros.org/wstool
4. Install rosdep: follow the steps described in http://wiki.ros.org/rosdep
5. Install catkin_tools: follow the steps described in https://catkin-tools.readthedocs.io/en/latest/installing.html

Create your workspace:
```
mkdir -p ~/replanning_ws/src
cd ~/replanning_ws
catkin init
wstool init src
```

Then, download and merge the rosinstall file:
```
cd ~/replanning_ws
wget https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/replanning_strategies/master/replanning_strategies.rosinstall
wstool merge -t src ./replanning_strategies.rosinstall
```
Download and install the packages specified in the rosinstall file and the other system dipendencies:
```
cd ~/replanning_ws
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
```
Finally, compile the workspace:
```
cd ~/replanning_ws
catkin build -cs
```
And source the workspace:
```
echo "source /home/$USER/replanning_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Docker
A [docker file](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE/blob/master/dockerfile_open_more) is also available. Open a terminal, move into the folder where you have saved the docker file and run the following command:
```
sudo docker build -f dockerfile_open_more -t open_more .
```
Once completed, run the container:
```
sudo docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    open_more
```
## Quick examples
If you want to take a look at how the library works, you can run these two quick examples in which the robot follows a trajectory and replans it when a random object obstructs its path. The simulation is repeated with different replanning algorithms, with different performance in the two following scenarios.

To use a Cartesian point robot, launch:
```
roslaunch replanners_lib quick_example_3d.launch
```
To use a 6 dof anthropomorphic robot, launch:
```
roslaunch replanners_lib quick_example_6d.launch
```
Note: when launching the examples you may get errors due to the lack of some libraries (e.g. trac-ik). They aren't needed for these examples, but you can install them with the following commands
```
sudo apt install ros-$ROS_DISTRO-trac-ik-kinematics-plugin ros-$ROS_DISTRO-chomp-motion-planner ros-$ROS_DISTRO-moveit-planners-chomp \ros-$ROS_DISTRO-pilz-industrial-motion-planner
```

## Packages
### **replanners_lib [see README](https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/blob/master/replanners_lib)**
It contains two main repository:
 1. **replanners**: contains the implementation of some sample-based replanning algorithms.
 2. **replanner_managers**: contains the implementation of a framework to manage the trajectory execution with continuous replanning for each of the available replanners.

 You can implement your replanning algorithm and integrate it into the framework.

### **replanners_benchmark [see README](https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/blob/master/replanners_benchmark)**
It contains a node to benchmark the available replanners and useful *launch* files. You can configure your benchmark or add new tests.

### **replanners_cells [see README](https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/blob/master/replanners_cells)**
It contains the urdf and *moveit_config* packages of the environments used for benchmarking. You can add your scenario.

## Work in progress
This repository is continuously evolving. If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/issues).

## How to cite
BibTex:
```
@ARTICLE{Tonola2023,
  author={Tonola, Cesare and Faroni, Marco and Beschi, Manuel and Pedrocchi, Nicola},
  journal={IEEE Access}, 
  title={Anytime Informed Multi-Path Replanning Strategy for Complex Environments}, 
  year={2023},
  volume={11},
  number={},
  pages={4105-4116},
  doi={10.1109/ACCESS.2023.3235652}}
  
  
@misc{open_more,
  author = {Tonola, Cesare and Beschi, Manuel},
  title = {{\textit{open_more}: an open-source library for robot path replanning}},
  url = {https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE}}
```

## Developer Contact
### **Authors**
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)

## Acknowledgements
**replanning_strategies** is developed by [CNR-STIIMA](http://www.stiima.cnr.it/) and [University of Brescia](https://www.unibs.it/en).

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
