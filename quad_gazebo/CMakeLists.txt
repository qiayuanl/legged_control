cmake_minimum_required(VERSION 3.10)
project(quad_gazebo)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        quad_common
        gazebo_ros_control
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        quad_hw_sim
        CATKIN_DEPENDS
        roscpp
        quad_common
        gazebo_ros_control
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(quad_hw_sim
        src/quad_hw_sim.cpp
        )

add_dependencies(quad_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(quad_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

#############
## Install ##
#############

install(TARGETS quad_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES quad_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
