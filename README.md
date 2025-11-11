![](documentation/logo_blue.png?raw=true)

## Introduction
**OpenMORE** is a library that provides a framework for managing robot's trajectory execution with online path replanning. It also includes several state-of-the-art sampling-based path replanning algorithms.
The goal of `OpenMORE` is to provide an efficient and flexible tool that simplifies the use of existing path replanning algorithms while also enabling the development and testing of new ones.

Check [this paper](https://ieeexplore.ieee.org/abstract/document/10275365) for more information.

`OpenMORE` is based on ROS and [MoveIt](https://moveit.github.io/moveit_tutorials/) to get information about the environment and collision checking. Currently, it supports Ubuntu 20.04 with ROS Noetic. We are actively working on extending compatibility to ROS 2 Humble.

## Concepts

At its core, `OpenMORE` is organized into three key packages:

- [`replanners_lib`](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib): A library implementing state-of-the-art sampling-based path replanning algorithms.
- [`replanners_managers_lib`](https://github.com/JRL-CARI-CNR-UNIBS/replanners_managers_lib): Provides the framework for executing robot trajectories with real-time path replanning.
- [`trajectories_processors_lib`](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib): Provides and interface to convert paths into trajectories and for trajectory interpolation.

For detailed information on each package, refer to the official documentation.

## Build & Install
While some `OpenMORE`'s packages are ROS-independent, others require compilation within a ROS workspace (e.g.,[`replanners_managers_lib`](https://github.com/JRL-CARI-CNR-UNIBS/replanners_managers_lib)). This tutorial assumes that `OpenMORE` and all its dependencies are installed within the same workspace.

Before proceeding, ensure you have the necessary dependencies installed. You can use the provided [deps.repos](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE/blob/devel/deps.repos) file and [vcstool](https://github.com/dirk-thomas/vcstool). Follow these instructions:

1. Install vcstool:
```bash
sudo apt install python3-vcstool
```

2. Set up a catkin workspace:
```bash
mkdir -p ~/openmore_ws/src
cd ~/openmore_ws
catkin init 
catkin config --extend /opt/ros/$ROS_DISTRO
```

3. Clone `OpenMORE` and its dependencies:
```bash
cd ~/openmore_ws/src
git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git
vcs import < OpenMORE/deps.repos
rosdep install --from-paths . --ignore-src -r -y
```

4. Finally, build the workspace:
```bash
cd ~/openmore_ws
catkin build -cs
source devel/setup.bash
```

Note that dependency `cnr_param` requires the environment variable `CNR_PARAM_ROOT_DIRECTORY` to be defined. For example, you can define it in the `~/.bashrc` file as follows:

```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"
```

This is the folder used by `cnr_param` to save parameters. See the dedicated [GitHub page](https://github.com/CNR-STIIMA-IRAS/cnr_param) for more information.

### Docker
A [docker file](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE/blob/master/dockerfile_OpenMORE) is also available. Open a terminal, move into the folder where you have saved the docker file and run the following command:
```
sudo docker build -f dockerfile_OpenMORE -t openmore .
```
Once completed, run the container:
```
xhost + 

sudo docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    openmore
```
Then, inside the container you can try the library.

## Tutorials
To get started with OpenMORE, refer to the following resources:

- **[Replanners Tutorials](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/blob/master/README.md)**: Learn how to use and implement new path replanners.
- **[Replanner Managers Tutorials](https://github.com/JRL-CARI-CNR-UNIBS/replanners_managers_lib/blob/master/README.md)**: Explore how to create and manage replanner architectures.

Additionally, the **[`openmore_ros_examples`](https://github.com/JRL-CARI-CNR-UNIBS/openmore_ros_examples)** repository provides practical examples of using `OpenMORE` exploiting MoveIt for managing the planning scene.

## Work in progress
This repository is a work in progress and is continuously evolving. As such, it is not free of bugs.
 **Please be careful if you use it on real hardware and ensure all necessary safety measures are in place**.

If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE/issues).

We are actively seeking support for further development. If you're interested, please reach out via email at <mailto::c.tonola001@unibs.it>.

Future works:
1. Compatibility with ROS2

## How to cite
Plain text:
```
C. Tonola, M. Beschi, M. Faroni and N. Pedrocchi, "OpenMORE: an open-source tool for sampling-based path replanning in ROS," 2023 IEEE 28th International Conference on Emerging Technologies and Factory Automation (ETFA), Sinaia, Romania, 2023, pp. 1-4, doi: 10.1109/ETFA54631.2023.10275365.
```

BibTex:
```
@INPROCEEDINGS{openmore,
  author={Tonola, Cesare and Beschi, Manuel and Faroni, Marco and Pedrocchi, Nicola},
  booktitle={2023 IEEE 28th International Conference on Emerging Technologies and Factory Automation (ETFA)}, 
  title={{OpenMORE: an open-source tool for sampling-based path replanning in ROS}}, 
  year={2023},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/ETFA54631.2023.10275365}} 
```

## Developer Contact
### **Authors**
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)

## Acknowledgements
**OpenMORE** is developed with [CNR-STIIMA](http://www.stiima.cnr.it/) and [University of Brescia](https://www.unibs.it/en).

***

![EC-H2020](documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
