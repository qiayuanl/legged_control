cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        controller_interface
        ocs2_legged_robot_ros
        angles
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        controller_interface
        ocs2_legged_robot_ros
        angles
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/legged_controller.cpp
        src/state_estimate.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
