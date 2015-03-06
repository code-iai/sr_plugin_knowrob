cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++0x")

rosbuild_add_library(sr_plugin_knowrob
  src/PluginKnowRob.cpp)

target_link_libraries(sr_plugin_knowrob
  sr_plugin_owlexporter
  config++)
