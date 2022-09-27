# **replanning_strategies**

The repository contains the implementation of a library of sample-based path replanning algorithms and a framework to manage trajectory execution with continuous path replanning. It is based on ROS and *MoveIt!* to get information about the environment.
<!-- You can read technical specifications about this replanner [here](https://arxiv.org/abs/2103.13245). -->
## Build/Installation
The software can be installed using rosinstall files.

1. Install ROS: follow the steps described in http://wiki.ros.org/ROS/Installation.
2. Install wstool and initialize the workspace: follow the steps described in http://wiki.ros.org/wstool.
3. Install and configure rosdep: follow the steps described in http://wiki.ros.org/rosdep.

Then, download and merge the rosinstall file:
```
cd ~/catkin_ws
wget https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/blob/master/replanning_strategies.rosinstall
wstool merge -t src ./replanning_strategies.rosinstall
```
Download and install the packages specified in the rosinstall file and the other system dipendencies:
```
cd ~/catkin_ws
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
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

## Developer Contact
### **Authors**
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)

## Acknowledgements
**replanning_strategies** is developed by CNR-STIIMA (http://www.stiima.cnr.it/)

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
