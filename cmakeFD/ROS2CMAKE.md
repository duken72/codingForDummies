# CMake for ROS2

`ament_cmake` is the build system for `CMake` based packages in ROS2. It is already pretty well documented [here](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html). I enjoy reading that user documentation, you should too :)). Here, only small parts will be briefly summarized, which are surely not enough for a newbie to understand.

## Table of Contents

- [Basics](#basics)
  - [Project outline](#project-outline)
  - [Linking](#linking)
  - [Build a Library](#build-a-library)
- [Testing and Linting](#testing-and-linting)
  - [Testing](#testing)
  - [Linting](#linting)
- [Extra notes](#extra-notes)

--------

## Basics

Common ament macros:

- `ament_package`
- `ament_target_dependencies`
- `ament_export_targets`
- `ament_export_dependencies`
- `ament_add_gtest`
- `ament_lint_auto_find_test_dependencies`
- `ament_add_gmock`

### Project outline

- CMake outline is produced from the moment you create the ROS2 package from the command line:

  ```bash
  ros2 pkg create <package_name>
  ```

- `ament_package()`, which should be somewhere in the bottom part of `CMakeLists.txt` (as explained in the user doc), install the `package.xml`.

### Linking

For linking, there are two ways. The first and recommended way is to use ament macro `ament_target_dependencies`. The second, which is still sometimes necessary to call, is to use `target_link_libaries`.

```cmake
find_package(Eigen3 REQUIRED)
ament_target_dependencies(my_target Eigen3)
target_link_libraries(my_target Eigen3::Eigen)
```

>It should never be necessary to find_package a library that is not explicitly needed, but is a dependency of another dependency that is explicitly needed. If that is the case, file a bug against the corresponding package.

### Build a Library

Use `ament_export_targets` and `ament_export_dependencies` as mentioned [here](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library).

--------

## Testing and Linting

### Testing

<!-- TODO(Duc): Add example -->
Sample part:

Dir structure:

### Linting

```bash
ament_lint_cmake
```

--------

## Extra notes

When an external package contains some warnings which make your build fail, you have to specify it as a SYSTEM dependency.
For ament dependencies, you can specify this like so:

```cmake
ament_target_dependencies(${library_name}
  SYSTEM nav2
)
```
