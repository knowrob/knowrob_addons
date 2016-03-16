cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(mod_vis)
rosbuild_find_ros_package(knowrob_common)

add_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/processing/core.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/processing/core.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(flowchart_vis org.knowrob.vis.flowchart.FlowChartVis)


# replace path to local OWL files
rosbuild_find_ros_package(flowchart_vis)
set( LOCAL_PACKAGE_PATH ${flowchart_vis_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

rosbuild_find_ros_package(comp_ehow)
set( COMP_EHOW_PACKAGE_PATH ${comp_ehow_PACKAGE_PATH} )
fix_ros_destination_path(COMP_EHOW_PACKAGE_PATH)

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

rosbuild_find_ros_package(knowrob_motion_constraints)
set( KNOWROB_MOTION_CONSTRAINTS_PACKAGE_PATH ${knowrob_motion_constraints_PACKAGE_PATH} )
fix_ros_destination_path(KNOWROB_MOTION_CONSTRAINTS_PACKAGE_PATH)


file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )