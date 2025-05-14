# ROS2

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [Installation](#installation)
* [CLI commands](#cli-commands)
* [Code Style](#code-style)
* [ROS2 Workspace](#ros2-workspace)
* [Launch file](#launch-file)
* [ROS2 Package](#ros2-package)
* [Running from Docker](#running-from-docker)
* [ROS1 vs ROS2](#ros1-vs-ros2)

<!-- vim-markdown-toc -->

---

## Installation

- If you are using supported OSes (Ubuntu, Windows, Debian, Fedora), just follow the instructions
- Installing ROS2 for Arch Linux is pain in the ass. Read some references materials:
  - Use [Docker](https://www.docker.com/) (probably the easiest)
  - [ROS2 Arch Wiki](https://wiki.archlinux.org/title/ROS#ROS_2)
  - [ROS2 Eloquent on Arch](https://discourse.ros.org/t/get-ros2-working-on-archlinux/12827)
  - [ROS2 Foxy on Arch](https://discourse.ros.org/t/some-tips-for-ros2-on-archlinux/17768)
  - [Reddit discussion](https://www.reddit.com/r/archlinux/comments/m4j7eu/has_anyone_managed_to_properly_build_ros2_foxy_on/)

---

## CLI commands

```bash
alias  rosg2='less $(fd ros2FD.bash ~)'
alias rosgg2='less $(fd ros2FD.bash ~) | grep'
```

Sourcing:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=<DOMAIN_ID>" >> ~/.bashrc
echo "export ROS_PARTICIPANT_ID=<PARTICIPANT_ID>" >> ~/.bashrc
```

Check with:

```bash
printenv | grep -i ROS
```

---

## Code Style

Use linter of ROS2:

```bash
ament_copyright
ament_cppcheck
ament_cpplint
ament_flake8
ament_link_cmake
ament_mypy
ament_pep257
ament_xmllint
ament_uncrustify
```

---

## ROS2 Workspace

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/ws_folder/src
cd ~/ws_folder/src
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

The structure should be: `build  install  log  src` (as the result from `ls ws_folder`)

- [Overlay and underlay](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)

---

## Launch file

1. Create directory and file in src/

   ```bash
   mkdir launch
   touch launch/launch_file.py
   ```

2. Check example
3. Add `exec_depend` dependency inn `package.xml`:
   `<exec_depend>ros2launch</exec_depend>`

---

## ROS2 Package

---

## Running from Docker

References: [docs.ros.org](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)

```bash
docker pull osrf/ros:humble-desktop

docker run -it --rm osrf/ros:humble-desktop     # start the container
ros2 pkg list | grep demo                       # inside the container
ros2 pkg executables demo_nodes_cpp
ros2 run demo_nodes_cpp talker
```

Start another container for the `listener`:

```bash
docker run -it --rm osrf/ros:humble-desktop     # start the container
ros2 run demo_nodes_cpp listener
```

- The two nodes can also be run in one single container

  ```bash
  ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker
  ```

- Alternatively, you can create a `docker-compose.yml` file
- To run with GUI:

```bash
xhost +     # run once each time you log into your machine
docker run -it --rm --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  osrf/ros:humble-desktop
ros2 run turtlesim turtlesim_node
```

## ROS1 vs ROS2

Source: [Robotics Back-End | YouTube](https://youtu.be/yn638LmVwlw?si=bCrm4KOoeyAXNcnY)

- Main reasons were that ROS1 misses many important features and not suitable for industrial applications (real time, safety, certification, security)
- API:
  - ROS1 has fewer layers: coding libraries `rospy` and `roscpp`, communication layers - TCPROS/UDPROS, Linux system
  - ROS2 has more layers: `rclcpp` and `rclpy` on top of shared `rcl` lib, DDS, Linux/Windows/MacOS
- Compatible with newer versions of Python and C++
- ROS2 has components as core functionality, instead of ROS1 Nodelets. We can have many nodes/components in an executable
- ROS2 has lifecycled nodes
- ROS2 has launch files in Python, which is more modular
- No more ROS master in ROS2, instead, we have decentralized communication
- ROS1 has centralized parameter server. In ROS2, each node has its own parameter server
- Services in ROS1 are synchronous, but can be asynchronous in ROS2
- ROS action is a core functionality in ROS2 now
- ROS2 has more customizable QoS - Quality of Service
- ROS2 introduces workspace overlays
- ROS2 supports more OSs: Linux, Windows, MacOS
