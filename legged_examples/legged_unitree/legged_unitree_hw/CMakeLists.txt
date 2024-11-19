cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)

#####################################################################   debug
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
#####################################################################

add_executable(${PROJECT_NAME}_2
        ${SOURCE}
        )

###   target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)

target_link_libraries(${PROJECT_NAME}_2
        ${catkin_LIBRARIES}
        unitree_sdk2
        )

#####################################################################   debug
# add_executable(low_level low_level.cpp)
# target_link_libraries(low_level unitree_sdk2)

# add_executable(stand_example_go2 stand_example_go2.cpp)
# target_link_libraries(stand_example_go2 unitree_sdk2)

# add_executable(stand_example_b2  stand_example_b2.cpp)
# target_link_libraries(stand_example_b2 unitree_sdk2)
#####################################################################


#########################
###   CLANG TOOLING   ###
#########################

#####################################################################   debug
find_package(unitree_sdk2 REQUIRED)
#####################################################################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_2 
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_2 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}_2/ 
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
