cmake_minimum_required(VERSION 3.22.1)

project(cmake_test)

add_executable(${PROJECT_NAME} duck.cpp)

add_subdirectory(Adder)

target_include_directories(${PROJECT_NAME} PUBLIC Adder)
# With target_include_directories .. Adder, you can use
# #include "adder.hpp" instead of
# #include "Adder/adder.hpp"

# target_link_directories(${PROJECT_NAME} PRIVATE Adder/)
# ??

target_link_libraries(${PROJECT_NAME} adder)
