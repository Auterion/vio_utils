cmake_minimum_required(VERSION 2.8.3)
project(butterworth)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############

cs_add_executable(butterworth_node
  src/butterworth.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
