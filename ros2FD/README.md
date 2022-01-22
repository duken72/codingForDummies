# ROS2

## Table of Contents

- [CLI commands](#cli-commands)
- [ROS2 Workspace](#ros2-workspace)
- [Launch file](#launch-file)
- [ROS2 Package](#ros2-package)

-------

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

-------

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

-------

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

-------

## Launch file

1. Create directory and file in src/

    ```bash
    mkdir launch
    touch launch/launch_file.py
    ```

2. Check example
3. Add `exec_depend` dependency inn `package.xml`:
`<exec_depend>ros2launch</exec_depend>`

-------

## ROS2 Package
