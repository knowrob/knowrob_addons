cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(mod_vis)

add_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)

add_classpath(${mod_vis_PACKAGE_PATH}/lib/snakeyaml-1.9.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/processing/core.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(flowchart_vis org.knowrob.vis.flowchart.FlowChartVis)
