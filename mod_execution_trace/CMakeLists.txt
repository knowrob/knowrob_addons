cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#######################################
# reset local paths in Prolog files

rosbuild_find_ros_package(mod_execution_trace)
set( LOCAL_PACKAGE_PATH ${mod_execution_trace_PACKAGE_PATH} )

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )

rosbuild_find_ros_package(ias_semantic_map)
set( IAS_MAP_PATH_PREFIX ${ias_semantic_map_PACKAGE_PATH} )

rosbuild_find_ros_package(knowrob_cram)
set( KNOWROB_CRAM_PATH_PREFIX ${knowrob_cram_PACKAGE_PATH} )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(jpl)

add_jni_path(${jpl_PACKAGE_PATH}/prolog)
add_ld_preload(/usr/lib/libswipl.so)
add_ld_preload(/usr/lib/libpl.so)
  
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_classpath(/usr/share/java/json-lib.jar)
add_classpath(/usr/share/java/commons-collections3.jar)
add_classpath(/usr/share/java/ezmorph.jar)
add_classpath(/usr/share/java/commons-beanutils.jar)
add_classpath(/usr/share/java/commons-lang.jar)
add_classpath(/usr/share/java/commons-logging.jar)

add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_runtime_classpath(/usr/share/java/json-lib.jar)
add_runtime_classpath(/usr/share/java/commons-collections3.jar)
add_runtime_classpath(/usr/share/java/ezmorph.jar)
add_runtime_classpath(/usr/share/java/commons-beanutils.jar)
add_runtime_classpath(/usr/share/java/commons-lang.jar)
add_runtime_classpath(/usr/share/java/commons-logging.jar)


# external dependencies in other packages
rosbuild_find_ros_package(rosjava_jni)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)
#add_jar_dir(${PROJECT_SOURCE_DIR})
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(ROSClient edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient)
rospack_add_java_executable(ROSClient2 edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level)
