cmake_minimum_required(VERSION 3.10)
project(isaac_legged_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  legged_common
  legged_hw
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    hardware_interface
    legged_common
    legged_hw
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    urdf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(isaac_hw_node
  src/isaac_hw_node.cpp
  src/IsaacHW.cpp
)
add_dependencies(isaac_hw_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(isaac_hw_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/isaac_gazebo_like_cmd_filter.py
  scripts/mode_to_contacts.py
  scripts/ros1_socket_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS isaac_hw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
