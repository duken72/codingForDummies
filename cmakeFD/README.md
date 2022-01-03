# CMAKE For Dummies

CMake is a tool for build system. If you don't know or forget what it means to compile and build code, check this video: ["How do computers read code?"](https://youtu.be/QXjU9qTsYCc).

The project needs to be compiled, built with makefiles. There is a need to maintain and update this files. It would be a tedious task as the project scale grows, unless there is a tool to automate the process of generating these makefiles. This is where build systems come into play: GNU Make, Autotools, SCons, Premake, Ninja, Meson, FASTBuild, Sharpmake, Maven, Ant, Gradle. ([src](https://julienjorge.medium.com/an-overview-of-build-systems-mostly-for-c-projects-ac9931494444))

Hopefully, this guide will give you a brief overview of what is needed.

<!-- TODO - add table of content -->

-------

## Makefile

> GNU Make is a tool which controls the generation of executables and other non-source files of a program from the program’s source files.

In the terminal, at the directory level where there is a `Makefile`, running `make` will simply execute commands in the file.

### Example 1

In [MakefileTest](MakefileTest), the content of [Makefile](MakefileTest/Makefile):

```make
default:
  g++ duck.cpp -o duck
```

Run `make` to generate the executable, then run it:

```bash
cd MakefileTest
make # simply equivalent to: "g++ duck.cpp -o duck"
./duck # run the executable
```

**Note**:

- `Makefile` accepts `tab`, not `space`.
- This is a Linux thing, not Windows thing.

Writing Makefiles quickly gets painful when the project scale grows. There are problems with dependecies, linker arguments, etc.

-------

## CMake

### CMake Overview

So `cmake` makes `Makefile`. Instead of manually write and manage `Makefile`(s), you write and manage `CMakeLists.txt`.

- Write a top level file `CMakeLists.txt` (the name has to be exactly like that).
- Run `cmake`, specifying the path to sources and path to build.

  ```bash
  cmake [options] <path-to-source>
  cmake [options] <path-to-existing-build>
  cmake [options] -S <path-to-source> -B <path-to-build>
  ```

- Run `make` to generate executables (at dir level with `Makefile`).

CMake has its own language. If you're going to work with ROS, working with CMake would be, like Thanos once said, "**I am inevitable**".

#### Example 2

Go to [CMakeEmptyTest](CMakeEmptyTest) and try `cmake` with an empty `CMakeLists.txt`.

```bash
cd CMakeEmptyTest
cmake -S . -B build
cd build && ls
make
```

Above command specifies the source is at current folder `CMakeEmptyTest`, the build folder is `build`. Go to folder `build`, there will be:

```bash
build
├── cmake_install.cmake
├── CMakeCache.txt
├── CMakeFiles
└── Makefile
```

At the dir level with `Makefile`, running `make` will generates the executable file. In this case, however, without specifying executable in `CMakeLists.txt`, no executables will be generated.

-------

### Basic Definition

"**Target**" are executable files, binary files. As the name itself suggests, they are the ultimate targets, the goal of CMake, to compile and build source code to these executables.

These targets have properties, which are to be managed in order for the source code to be compiled and built properly. Some common properties are: `LINK_LIBRARIES`, `INCLUDE_DIRECTORIES`, `COMPILE_DEFINITIONS`, `COMPILE_OPTIONS`.

-------

### CMake Syntax Overview

Again, **CMake has its own language.** It has variables, statements, flow control commands, etc. Check [CMakeLists.txt](CMakeLists.txt).

- In CMake, every variables is a string. When a variable is undefined, it defaults to an empty string. ([src](https://preshing.com/20170522/learn-cmakes-scripting-language-in-15-minutes/))

Some Statement Commands:

- message: `message("Hello world!")`
- set: `set(THING funk)`
- math: `math(EXPR VAR "1+1")`
- list: `list(REMOVE_ITEM MY_LIST "separate")`
- Quotes around arguments are **optional**, as long as there are no **spaces** or **variable references** in the argument. Example, the following two lines are equivalent: `set(THING funk)` and `set(THING "funk")`.
- Lists are Just Semicolon-Delimited Strings

  ```cmake
  set(ARGS "EXPR;T;1 + 1")
  math(${ARGS})   # Equivalent to math(EXPR T "1 + 1")
  ```

- Functions Run In Their Own Scope; Macros Don’t

Flow Control Commands:

- if()-else()-endif()
- while()-endwhile()
- foreach()-endforeach()

#### Example 3

Try the script in [CMakeScriptTest](CMakeScriptTest). Play with the content of [test.txt](CMakeScriptTest/test.txt) and see how the output changes.

```bash
cd CMakeScriptTest
cmake -P test.txt #without specifing, NAME would be empty string
cmake -DNAME=DUCK -P test.txt #Set variable NAME = DUCK
```

-------

## CMAKE with C++ and ROS

Good tutorial videos: [CMake Tutorial](https://www.youtube.com/watch?v=nlKcXPUJGwA&list=PLalVdRk2RC6o5GHu618ARWh0VO0bFlif4&t=0s).

### Important commands

1. `add_executable`: [CMake Doc](https://cmake.org/cmake/help/latest/command/add_executable.html)

    ```cmake
    add_executable(<name> [WIN32] [MACOSX_BUNDLE]
                  [EXCLUDE_FROM_ALL]
                  [source1] [source2 ...])
    add_executable(duck duck.cpp)
    ```

2. Include vs link libraries: [stackoverflow discussion](https://stackoverflow.com/questions/56565665/difference-between-target-link-libraries-and-target-include-directories)

    - `*include_directories` is used to supply a list of include directories to the compiler. When a file is included using the pre-processor, these directories will be searched for the file.

    <!-- TODO - I don't really understand this shit -->
    - `*link_libraries` is used to supply a list of libraries (object archives) to the linker. If the linked item is a cmake target, with specified include directories, they don't need to be specified separately with *include_directories.

    - The `target_*` versions apply only to the target that is given as an operand. The non-target versions apply to all targets in the directory. The `target_*` versions should be used whenever possible (i.e. pretty much always). [(stackoverflow)](https://stackoverflow.com/questions/31969547/what-is-the-difference-between-include-directories-and-target-include-directorie#:~:text=include_directories(x%2Fy)%20affects,include%20path%20for%20target%20t%20.)  
    -> `include_directories` vs `target_include_directories`  
    -> `link_libraries` vs `target_link_libraries`


3. `find_package`, `include` and `add_subdirectory`

   - `include` executes another CMake script in the same scope as the calling script.
   - `find_package` looks for scripts in one of these forms: `Find<PackageName>.cmake`, `<PackageName>Config.cmake`, etc (read [CMake docs](https://cmake.org/cmake/help/latest/index.html)). It also runs them in the same scope.
   - `find_package(SDL2)` is equivalent to `include(FindSDL2.cmake)`  
   Let just stick with `find_package`.
   - `add_subdirectory`, on the other hand, creates a new scope, then executes the script named `CMakeLists.txt` from the specified directory in that new scope.

4. `include_directories` and `target_include_directories`:

    Assume having `z.hpp` in x/y:
    - `include_directories(x/y)` affects directory scope. All targets in this `CMakeLists.txt`, as well as those in all subdirectories added after the point of its call, will have the path x/y added to their include path. Thus you can `#include <z.hpp>` instead of `#include <x/y/z.hpp>`.
    - `target_include_directories(t x/y)` has target scope. It adds x/y to the include path for **only** target t. Eg., for only `t.cpp`, you can write `#include <z.hpp>`.
    - Syntax:

    ```cmake
    include_directories([AFTER|BEFORE] [SYSTEM] dir1 [dir2 ...])
    target_include_directories(<target> [SYSTEM] [BEFORE]
      <INTERFACE|PUBLIC|PRIVATE> [items1...]
      [<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
    ```

    - The meanings of scopes: [`PUBLIC`, `PRIVATE`, and `INTERFACE`](https://stackoverflow.com/questions/26243169/cmake-target-include-directories-meaning-of-scope_)
    - Example:

    ```cmake
    find_package(Eigen3 REQUIRED)
    find_package(grid_map_core REQUIRED)

    include_directories(
      include
      ${EIGEN3_INCLUDE_DIR}
    )

    target_include_directories(${PROJECT_NAME} PUBLIC grid_map_core)

    ```

5. as

### Including other scripts

- `add_library`
- `add_custom_target`
- `set_property`
- `get_property`

```cmake
# Get the target's SOURCES property and assign it to MYAPP_SOURCES
get_property(MYAPP_SOURCES TARGET MyApp PROPERTY SOURCES)
```

### Target properties

Common target properties and the commands to change them:

- `target_link_libraries` -
- `target_include_directories` -

- `target_compile_definitions` -
- `target_compile_options`

### CMake Structure for ROS1

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

### CMake for ROS2

`ament_cmake` is the build system for `CMake` based packages in ROS2.
