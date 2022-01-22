# CMake for ROS2

`ament_cmake` is the build system for `CMake` based packages in ROS2.

When an external package contains some warnings which make your build fail, you have to specify it as a SYSTEM dependency.
For ament dependencies, you can specify this like so:

```cmake
ament_target_dependencies(${library_name}
  SYSTEM nav2
)
```
