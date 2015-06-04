cmake_minimum_required(VERSION 2.8.3)
project(sr_plugin_knowrob)

find_package(catkin REQUIRED COMPONENTS roscpp json_prolog semrec)

set(CMAKE_CXX_FLAGS "-std=c++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sr_plugin_knowrob
  CATKIN_DEPENDS json_prolog semrec
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(sr_plugin_knowrob
  src/PluginKnowRob.cpp)

target_link_libraries(sr_plugin_knowrob
  sr_plugin_owlexporter
  config++)
