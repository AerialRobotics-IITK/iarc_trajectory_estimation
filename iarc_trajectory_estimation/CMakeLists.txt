cmake_minimum_required(VERSION 2.8.3)
project(iarc_trajectory_estimation)

add_definitions(-std=c++17)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

## cs_add_library()

###############
# EXECUTABLES #
###############

## cs_add_executable()
## target_link_libraries()

##########
# EXPORT #
##########

cs_install()
cs_export()