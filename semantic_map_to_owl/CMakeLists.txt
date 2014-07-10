
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

#######################################
# Classpath

rosbuild_find_ros_package(mod_vis)
rosbuild_find_ros_package(knowrob_common)
rosbuild_find_ros_package(knowrob_cad_parser)

add_classpath(${mod_vis_PACKAGE_PATH}/bin)
add_classpath(${knowrob_common_PACKAGE_PATH}/bin)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/owlapi-bin.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/owlapi-bin.jar)

add_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/controlP5.jar)
add_runtime_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/controlP5.jar)
add_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/processing/core.jar)
add_runtime_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/processing/core.jar)


# external dependencies in other packages
rosbuild_find_ros_package(rosjava)
add_classpath(${rosjava_PACKAGE_PATH}/bin)


# generate launch script
rospack_add_java_executable(SemanticMapToOWL edu.tum.cs.ias.knowrob.map.SemanticMapToOWL)
rospack_add_java_executable(SemanticMapToOWLTestClient edu.tum.cs.ias.knowrob.map.SemanticMapToOWLTestClient)
rospack_add_java_executable(SemanticMapEditor edu.tum.cs.ias.knowrob.map.SemanticMapEditor)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
