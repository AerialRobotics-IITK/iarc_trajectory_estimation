cmake_minimum_required(VERSION 2.8.3)
project(iarc_trajectory_estimation)

add_definitions(-std=c++17)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}_lib src/trajectory_generation.cpp)

###############
# EXECUTABLES #
###############

cs_add_executable(${PROJECT_NAME}_node src/trajectory_generation_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

##########
# EXPORT #
##########

cs_install()
cs_export()