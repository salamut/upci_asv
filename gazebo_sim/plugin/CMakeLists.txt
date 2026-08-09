cmake_minimum_required(VERSION 3.10)
project(asv_json_bridge)

find_package(ignition-gazebo6 REQUIRED)

add_library(AsvJsonBridge SHARED AsvJsonBridge.cc)
set_property(TARGET AsvJsonBridge PROPERTY CXX_STANDARD 17)
target_link_libraries(AsvJsonBridge PRIVATE ignition-gazebo6::ignition-gazebo6)
