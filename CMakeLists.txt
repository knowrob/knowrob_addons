cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


# replace paths for includes in owl files
rosbuild_find_ros_package(inf9_knowledge_base)
set( OWL_PATH_PREFIX ${inf9_knowledge_base_PACKAGE_PATH} )

file(GLOB inputs owl/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


# replace paths for includes in prolog files
rosbuild_find_ros_package(prolog_perception)
set( PL_PATH_PREFIX "${prolog_perception_PACKAGE_PATH}/" )

file(GLOB inputs prolog/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

