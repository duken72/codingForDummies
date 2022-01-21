# CMake for ROS1

[WIKI ROS](http://wiki.ros.org/catkin/CMakeLists.txt)

> Your CMakeLists.txt file **MUST** follow this format otherwise your packages will not build correctly. The order in the configuration **DOES** count.

1. Required CMake Version (cmake_minimum_required)
2. Package Name (project())
3. Find other CMake/Catkin packages needed for build (find_package())
4. Enable Python module support (catkin_python_setup())
5. Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
6. Invoke message/service/action generation (generate_messages())
7. Specify package build info export (catkin_package())
8. Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
9. Tests to build (catkin_add_gtest())
10. Install rules (install())