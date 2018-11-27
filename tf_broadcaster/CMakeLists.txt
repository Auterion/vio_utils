cmake_minimum_required(VERSION 2.8.3)
project(tf_broadcaster)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############

cs_add_executable(tf_broadcaster
  src/tf_broadcaster.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
