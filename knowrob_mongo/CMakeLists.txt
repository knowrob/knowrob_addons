cmake_minimum_required(VERSION 2.8.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

add_classpath(${PROJECT_SOURCE_DIR}/lib/mongo-java-driver-2.11.2.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/mongo-java-driver-2.11.2.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)


# replace path to local OWL files
rosbuild_find_ros_package(knowrob_mongo)
set( LOCAL_PACKAGE_PATH ${knowrob_mongo_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

rosbuild_find_ros_package(mod_srdl)
set( SRDL_PATH_PREFIX ${mod_srdl_PACKAGE_PATH} )
fix_ros_destination_path(SRDL_PATH_PREFIX)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
