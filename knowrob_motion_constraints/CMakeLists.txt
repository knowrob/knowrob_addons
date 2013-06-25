cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)
rosbuild_genmsg()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# replace path to local OWL files
rosbuild_find_ros_package(knowrob_motion_constraints)
set( LOCAL_PACKAGE_PATH ${knowrob_motion_constraints_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)


rosbuild_find_ros_package(mod_srdl)
set( SRDL_PATH_PREFIX ${mod_srdl_PACKAGE_PATH} )
fix_ros_destination_path(SRDL_PATH_PREFIX)

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

add_java_source_dir( ${PROJECT_SOURCE_DIR}/src )
rospack_add_java_executable( constr_to_owl org.knowrob.constr.ConstraintsToOWLService )
rospack_add_java_executable( constr_to_owl_test_client org.knowrob.constr.ConstraintsToOWLTestClient )
