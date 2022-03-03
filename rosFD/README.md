<!-- omit in toc -->
# ROS

<!-- omit in toc -->
## Table of Contents

- [CLI commands](#cli-commands)
- [ROS Package](#ros-package)
- [Building ROS Package](#building-ros-package)

## CLI commands

```bash
alias  rosg='less $(fd rosFD.bash ~)'
alias rosgg='less $(fd rosFD.bash ~) | grep'
```

## ROS Package

To create a new package:

1. Start with create a workspace, a.k.a. a worksplace_folder
2. Run this code to create package, define some stuffs (dependencies)

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

    ```bash
    workspace_folder/         -- WORKSPACE
    ├── src/                  -- SOURCE SPACE
    │  ├── CMakeLists.txt     -- 'Toplevel' CMake file, provided by catkin
    │  ├── ..
    │  ├── package_n/
    │  │  ├── CMakeLists.txt  -- CMakeLists.txt file for package_n
    │  │  ├── package.xml     -- Package manifest for package_n
        ..
    ```

-------

## Building ROS Package

```bash
source /opt/ros/%ROS_DISTRO%/setup.bash
cd ~/catkin_ws
catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

Expected results:

```bash
workspace_folder  -- WORKSPACE
├── build         -- 'cmake' and 'make' are called to configure and build packages
├── devel         -- the executables and libraries go here, before the packages are installed
└── src           -- SOURCE SPACE
```
