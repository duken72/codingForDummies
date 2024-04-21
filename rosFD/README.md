# ROS

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [CLI commands](#cli-commands)
* [ROS Package](#ros-package)
* [Building ROS Package](#building-ros-package)
* [Overview of ROS Graph Concepts](#overview-of-ros-graph-concepts)
* [Listening to ROS node in external Docker container](#listening-to-ros-node-in-external-docker-container)

<!-- vim-markdown-toc -->

## CLI commands

```bash
alias  rosg='less $(fd rosFD.bash ~)'
alias rosgg='less $(fd rosFD.bash ~) | grep'
```

## ROS Package

To create a new package:

1. Start with create a workspace, a.k.a. a worksplace_folder
2. Run this code to create a new package, define some dependencies

   ```bash
   cd ../src
   catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
   ```

3. Build a catkin workspace and sourcing the setup file

   ```bash
   cd ~/catkin_ws
   catkin_make
   . ~/catkin_ws/devel/setup.bash # add workspace to ROS environment, by sourcing the setup file
   ```

4. Customize: edit stuffs in package.xml and CMakeLists.txt, mostly related to dependencies
5. Expected results

   ```txt
   workspace_folder/         -- WORKSPACE
   ├── src/                  -- SOURCE SPACE
   │  ├── CMakeLists.txt     -- 'Toplevel' CMake file, provided by catkin
   │  ├── ..
   │  ├── package_n/
   │  │  ├── CMakeLists.txt  -- CMakeLists.txt file for package_n
   │  │  ├── package.xml     -- Package manifest for package_n
       ..
   ```

---

## Building ROS Package

```bash
source /opt/ros/%ROS_DISTRO%/setup.bash
cd ~/catkin_ws
catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

Expected results:

```txt
catkin_ws       -- WORKSPACE
├── build       -- 'cmake' and 'make' are called to configure and build packages
├── devel       -- the executables and libraries go here, before the packages are installed
└── src         -- SOURCE SPACE
```

---

## Overview of ROS Graph Concepts

- Node is an executable that uses `ROS` to communicate with other nodes.
- Messages: `ROS` data type used when subscribing or publishing to a topic.
- Topics: Nodes can publish messages to a topic, or subscribe to a topic to receive messages.
- Master: Name service for `ROS` (i.e. helps nodes find each other)
- `rosout`: `ROS` equivalent of `stdout/stderr`
- `roscore`: Master + `rosout` + parameter server (parameter server will be introduced later)

---

## Listening to ROS node in external Docker container

References:

- [finnrietz.dev](https://www.finnrietz.dev/linux/ros-docker/)
- [roboticseabass.com](https://roboticseabass.com/2021/04/21/docker-and-ros/)
- [tuw-cpsg.github](https://tuw-cpsg.github.io/tutorials/docker-ros/)

- This is just for ROS1, not for ROS2
- Has to specify the network connection when starting the Docker container

```bash
docker run -it --network=host ros:melodic
```
