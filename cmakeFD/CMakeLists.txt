cmake_minimum_required(VERSION 3.22.1)

# set the project name and version
project(cmakeForDummies VERSION 1.0)

configure_file(${PROJECT_NAME}Config.h.in ${PROJECT_NAME}Config.h)

# Set the Standard
# C Standard: Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# C++ Standard: Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()
# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find_package(<PackageName> [version] [EXACT] [QUIET] [MODULE]
#              [REQUIRED] [[COMPONENTS] [components...]]
#              [OPTIONAL_COMPONENTS components...]
#              [NO_POLICY_SCOPE])

add_executable(${PROJECT_NAME} duck.cpp)
