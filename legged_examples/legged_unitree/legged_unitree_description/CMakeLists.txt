cmake_minimum_required(VERSION 3.10)
project(legged_unitree_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)

#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
