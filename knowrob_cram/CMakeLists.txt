cmake_minimum_required(VERSION 2.8.5)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)


# replace path to local OWL files

rosbuild_find_ros_package(ias_knowledge_base)
set( IAS_KB_PACKAGE_PATH ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(IAS_KB_PACKAGE_PATH)

rosbuild_find_ros_package(knowrob_mongo)
set( LOCAL_PACKAGE_PATH ${knowrob_mongo_PACKAGE_PATH} )
fix_ros_destination_path(MONGO_PACKAGE_PATH)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
