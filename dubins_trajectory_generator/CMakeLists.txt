cmake_minimum_required(VERSION 2.8.3)
project(dubins_trajectory_generator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
    src/dubins_trajectory.cpp
)

############
# BINARIES #
############

cs_add_executable(dubins_trajectory_node
    src/dubins_trajectory_node.cpp
)
target_link_libraries(dubins_trajectory_node ${PROJECT_NAME})

##########
# EXPORT #
##########

cs_install()
cs_export()
