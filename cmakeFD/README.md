# CMAKE For Dummies

CMake is a tool for build system. If you don't know or forget what it means to compile and build code, check this video: ["How do computers read code?"](https://youtu.be/QXjU9qTsYCc).

The project needs to be compiled, built with makefiles. There is a need to maintain and update this files. It would be a tedious task as the project scale grows (.eg, using ROS), unless there is a tool to automate the process of generating these makefiles. This is where build systems come into play: GNU Make, Autotools, SCons, Premake, Ninja, Meson, FASTBuild, Sharpmake, Maven, Ant, Gradle. ([src](https://julienjorge.medium.com/an-overview-of-build-systems-mostly-for-c-projects-ac9931494444))

Hopefully, these notes will give you a brief overview of what is needed.

-------

## Table of Contents

- [Makefile](#makefile)
- [CMake](#cmake)
  - [CMake Overview](#cmake-overview)
  - [Basic Definition](#basic-definition)
  - [CMake Syntax Overview](#cmake-syntax-overview)
- [Important commands](#important-commands)
  - [Add target](#add-target)
  - [Find / Include Package](#find--include-package)
  - [Add Subdirectory](#add-subdirectory)
  - [Include Directories](#include-directories)
  - [Link Libraries](#link-libraries)
  - [Including other scripts](#including-other-scripts)
  - [Installing the Software](#installing-the-software)
  - [Build Types](#build-types)
- [Code Style](#code-style)
- [ROS](#ros)

-------

## Makefile

> GNU Make is a tool which controls the generation of executables and other non-source files of a program from the program’s source files.

In the terminal, at the directory level where there is a `Makefile`, running `make` will simply execute commands in the file.

**Example 1**: In [MakefileTest](MakefileTest), the content of [Makefile](MakefileTest/Makefile):

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
- This is a Linux's thing, not a Windows thing.

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

**Example 2**: Go to [CMakeEmptyTest](CMakeEmptyTest) and try `cmake` with an empty `CMakeLists.txt`.

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

At the dir level with `Makefile`, running `make` will generates the executable file. In this example, however, without specifying executable in `CMakeLists.txt`, no executables will be generated.

-------

### Basic Definition

"**Target**" are executable files, binary files. As the name itself suggests, they are the ultimate targets, the goal of CMake, to compile and build source code (in C++) to these executables.

These targets have properties, which are to be managed in order for the source code to be compiled and built properly. Some common properties are: `LINK_LIBRARIES`, `INCLUDE_DIRECTORIES`, `COMPILE_DEFINITIONS`, `COMPILE_OPTIONS`.

Include directories are where you need to find the C++ header files. With correct ones, you can add header files on top of your `.cpp` files: `#include <something.h>`. This is where the compiler will find function declarations, type definitions, classes, structures, etc. While compiler concerns with the declarations, linker concerns with functions, methods definitions. These are handled by link libraries. In the build process, a target has to be compiled and linked in order to become a executable. Check [C++ for Dummies](../cppFD/README.md) for more in depth understand.

The commands to change them:

- `target_include_directories`
- `target_link_libraries`
- `target_compile_definitions`
- `target_compile_options`
- `set_property`
- `get_property`

```cmake
# Get the target's SOURCES property and assign it to MYAPP_SOURCES
get_property(MYAPP_SOURCES TARGET MyApp PROPERTY SOURCES)
```

In other cases, instead of executable(s), the targets to build could be "**libraries**" - group of functionalities that could be used somewhere else.

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
- Lists are just semicolon-delimited strings

  ```cmake
  set(ARGS "EXPR;T;1 + 1")
  math(${ARGS})   # Equivalent to math(EXPR T "1 + 1")
  ```

- Functions Run In Their Own Scope; Macros Don’t

Flow Control Commands:

- if()-else()-endif()
- while()-endwhile()
- foreach()-endforeach()

**Example 3**: Try the script in [CMakeScriptTest](CMakeScriptTest). Play with the content of [test.txt](CMakeScriptTest/test.txt) and see how the output changes.

```bash
cd CMakeScriptTest
cmake -P test.txt #without specifing, NAME would be empty string
cmake -DNAME=DUCK -P test.txt #Set variable NAME = DUCK
```

-------

## Important commands

CMake Tutorials: [Youtube](https://www.youtube.com/watch?v=nlKcXPUJGwA&list=PLalVdRk2RC6o5GHu618ARWh0VO0bFlif4&t=0s), [cmake.org](https://cmake.org/cmake/help/latest/guide/tutorial/index.html).

### Add target

CMake command: `add_executable` [(CMake Doc)](https://cmake.org/cmake/help/latest/command/add_executable.html)

```cmake
add_executable(<name> [WIN32] [MACOSX_BUNDLE]
              [EXCLUDE_FROM_ALL]
              [source1] [source2 ...])
add_executable(duck duck.cpp)
```

-------

### Find / Include Package

- `find_package` looks for scripts in one of these forms: `Find<PackageName>.cmake`, `<PackageName>Config.cmake`, etc (read [CMake docs](https://cmake.org/cmake/help/latest/index.html)). It also runs them in the same scope.

  ```cmake
  # Module mode finds Find<PackageName>.cmake
  find_package(OpenCV MODULE REQUIRED) #find FindOpenCV.cmake
  # Config mode finds <PackageName>Config.cmake or <PackageName>-config.cmake
  find_package(OpenCV MODULE REQUIRED) #find OpenCVConfig.cmake
  ```

- `include` executes another CMake script in the same scope as the calling script. Eg., `find_package(SDL2)` is equivalent to `include(FindSDL2.cmake)`. Let just stick with `find_package`.
- `find_package` = `find_library` + `find_path` ??

```cmake
find_library(NameYouMadeUp ActualName NAMES LiBnAmE lIbNaMe)
```

`find_library` assumes `ActualName` to be `libActualName.o`. If found, `NameYouMadeUp` becomes the path to the library, if not, then it's set as `NameYouMadeUp-NOTFOUND`

```cmake
message("suffixes: ${CMAKE_FIND_LIBRARY_SUFFIXES}")
message("prefixes: ${CMAKE_FIND_LIBRARY_PREFIXES}")
#RESULTS:
#suffixes: .so;.a
#prefixes: lib
```

TODO - WHAT ARE??

- [PACKAGE_ROOT_PATH]
- [CMAKE_PATH]
- [CMAKE_ENVIRONMENT_PATH]
- [SYSTEM_ENVIRONMENT_PATH]
- [CMAKE_SYSTEM_PATH]

-------

### Add Subdirectory

`add_subdirectory` creates a new scope, then executes the `CMakeLists.txt` from the specified directory in that new scope. Use this if, inside your package dir, you group files into folders.

Example:

```bash
pkg
├── CMakeLists.txt
└── Test
    └── CMakeLists.txt
```

The content inside the high-level `CMakeLists.txt` in the `pkg` folder:

```cmake
message("Before add_subdirectory")
add_subdirectory(Test)
message("After add_subdirectory")
```

The content inside the low-level `CMakeLists.txt` in the `Test` folder:

```cmake
message("Inside add_subdirectory")
```

Running `cmake` in `pkg` directory level will result in:

```bash
$ cmake -S . -B build
Before add_subdirectory
Inside add_subdirectory
After add_subdirectory
```

That's a worthless piece of code :)). But I hope you get the idea. Again, it simply runs the `CMakeLists.txt` script in the added subdirectory. Real usage would be define variables, library, include path, etc., such that the higher-level `CMakeLists.txt` can call.

-------

### Include Directories

The include paths are the lines in the begining of your CPP files `#include <z.hpp>`

There are two `CMake` commands: `include_directories` and `target_include_directories`

- `include_directories` affects directory scope. All targets in this `CMakeLists.txt`, as well as those in all subdirectories added after the point of its call, will have the path added to their include path.
- `target_include_directories` has target scope. It adds to the include path for **only** specified target.
- The `target_*` versions should be used whenever possible (i.e. pretty much always).
- Syntax:

  ```cmake
  include_directories([AFTER|BEFORE] [SYSTEM] dir1 [dir2 ...])
  target_include_directories(<target> [SYSTEM] [BEFORE]
    <INTERFACE|PUBLIC|PRIVATE> [items1...]
    [<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
  ```

- The meanings of different scopes: [`PUBLIC`, `PRIVATE`, and `INTERFACE`](https://stackoverflow.com/questions/26243169/cmake-target-include-directories-meaning-of-scope_)

Pseudo example: assume having `z.hpp` in x/y:

- `include_directories(x/y)`: All targets will have the path x/y added. Thus `#include <z.hpp>` can be called, instead of `#include <x/y/z.hpp>`.
- `target_include_directories(t x/y)`: It adds x/y to the include path for **only** target t. For only `t.cpp`, `#include <z.hpp>` can be called.

Real example:

```cmake
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
```

After using `find_package` to find a package, it almost always sets the variables, which specify package include directory and link library. In above example, `EIGEN3_INCLUDE_DIR` is set in `FindEigen3.cmake`, which is called by `find_package(Eigen3 REQUIRED)`. Thus, the include directory of Eigen could be add with either `include_directories` or `target_include_directories`.

-------

### Link Libraries

I understand library as a group of object files / targets, not necessary executables. Define the library for internal / external usages (to your package/workspace) with:

```cmake
add_library(LibName file1.cpp, file2.cpp)
```

For a newly defined target to be able to use the group for object files, the target needs to be linked with that group, aka., the library.

1. Get the library definition. If the library is defined somewhere else, not in the same `CMakeLists.txt`, then it can be included with `add_subdirectory`, `find_library`, `find_package`, `link_directories`.

2. Link to target(s) with `(target_)link_libraries`

**Example 4:**

```cmake
add_subdirectory(LibName)
add_executable(TargetName file.cpp)
target_link_libraries(TargetName PUBLIC LibName)
target_include_directories(TargetName PUBLIC
  "${PROJECT_BINARY_DIR}"
  "${PROJECT_SOURCE_DIR}/LibName"
)
```

Making library optional:

```cmake
option(USE_LIBTEST "Use test lib" ON)
configure_file(LibName.h.in LibName.h)
if (USE_LIBTEST)
  add_subdirectory(LibName)
  list(APPEND EXTRA_LIBS LibName)
  list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/LibName")
endif()
```

-------

### Including other scripts

- `add_library`
- `add_custom_target`

-------

### Installing the Software

> If you are only ever using your software from a source build, you can probably ignore it. If you ever want to deploy your software, however, I would strongly encourage having an install process.  
Installing makes a software package generally available to users of the system, by installing its components into a well-known prefix (e.g. /usr, /usr/local, /opt/MySoft). It is often much more convenient to use an installed software package rather than stuff in a build directory, as installed binaries tend to be in e.g. PATH, whereas build directories may not be readable by all users [[cmake.org](https://cmake.org/pipermail/cmake/2013-April/054247.html)].

Syntax:

```cmake
install(TARGETS ${target_name} DESTINATION bin)
install(FILES "${PROJECT_BINARY_DIR}/TargetConfig.h" DESTINATION include)
```

Command:

```bash
cmake --install
# for cmake before 3.15
# make install
```

-------

### Build Types

Source: [stackoverflow](https://stackoverflow.com/questions/48754619/what-are-cmake-build-t.ype-debug-release-relwithdebinfo-and-minsizerel)

- Only makes sense for single-target generators, like `Makefiles`.
- `Release`: high optimization level, no debug info, code or asserts.
- `Debug`: No optimization, asserts enabled, [custom debug (output) code enabled], debug info included in executable (so you can step through the code with a debugger and have address to source-file:line-number translation).
- `RelWithDebInfo`: optimized, *with* debug info, but no debug (output) code or asserts.
- `MinSizeRel`: same as Release but optimizing for size rather than speed.

Commands:

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

<!-- TODO - Explain debug info -->
<!-- TODO - Don't really understand yet -->

<!-- TODO
Ask Marco about:
- Build types: example of debug info, code enabled?
- Linking
- find_library v/s find_package
- find_package = find_library + find_path (auto the necessary)
- link_directory -->

-------

## Code Style

For ROS2 package, use:

```bash
ament_lint_cmake
```

-------

## ROS

Check CMake for [ROS1]((ROS1CMAKE.md)), [ROS2](ROS2CMAKE.md).

-------

## Tips

In case `FindPackage.cmake` is not found, search for customized script online and put it in your package directory as follow:

```bash
pkg
├── CMakeLists.txt
└── cmake_modules
    └── FindPackage.cmake
```

In the top-level `CMakeLists.txt`, add this line, before `find_package`:

```cmake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)
```
