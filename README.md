![](Documentation/logo_blue.png?raw=true)

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
xhost + 

sudo docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    open_more
```
Then, inside the container you can try the library (see Quick examples below).

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
This repository is a work in progress and is continuously evolving. As such, it is not free of bugs. Please be careful if you use it on real hardware and take all necessary precautions.
If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/replanning_strategies/issues).

We are actively seeking support for further development. If you're interested, please reach out via email at <mailto::c.tonola001@unibs.it>.

Future works:
1. ROS-free version and integration to ROS2
2. Generalization of the scene-monitoring software (currently, Moveit)
3. More documentation and tutorials
   
## How to cite
BibTex:
```
@INPROCEEDINGS{openmore,
  author={Tonola, Cesare and Beschi, Manuel and Faroni, Marco and Pedrocchi, Nicola},
  booktitle={2023 IEEE 28th International Conference on Emerging Technologies and Factory Automation (ETFA)}, 
  title={OpenMORE: an open-source tool for sampling-based path replanning in ROS}, 
  year={2023},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/ETFA54631.2023.10275365}} 
```

## Disclaimer
Copyright (c) 2023, Cesare Tonola CNR-STIIMA cesare.tonola@stiima.cnr.it.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the <organization> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Developer Contact
### **Authors**
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)

## Acknowledgements
**OpenMORE** is developed by [CNR-STIIMA](http://www.stiima.cnr.it/) and [University of Brescia](https://www.unibs.it/en).

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
