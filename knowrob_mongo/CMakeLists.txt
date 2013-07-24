cmake_minimum_required(VERSION 2.8.7)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

add_classpath(${PROJECT_SOURCE_DIR}/lib/mongo-java-driver-2.11.2.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/mongo-java-driver-2.11.2.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

include(UseJava)

find_jar(JSON_LIB_JAR NAMES json-lib)
add_classpath(${JSON_LIB_JAR})
add_runtime_classpath(${JSON_LIB_JAR})

find_jar(COMMONS_BEANUTILS_JAR NAMES commons-beanutils)
add_classpath(${COMMONS_BEANUTILS_JAR})
add_runtime_classpath(${COMMONS_BEANUTILS_JAR})

find_jar(COMMONS_COLLECTIONS3_JAR commons-collections3)
add_classpath(${COMMONS_COLLECTIONS3_JAR})
add_runtime_classpath(${COMMONS_COLLECTIONS3_JAR})

find_jar(COMMONS_LOGGING_JAR commons-logging)
add_classpath(${COMMONS_LOGGING_JAR})
add_runtime_classpath(${COMMONS_LOGGING_JAR})

find_jar(EZMORPH_JAR ezmorph)
add_classpath(${EZMORPH_JAR})
add_runtime_classpath(${EZMORPH_JAR})

find_jar(COMMONS_LANG_JAR commons-lang)
add_classpath(${COMMONS_LANG_JAR})
add_runtime_classpath(${COMMONS_LANG_JAR})


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
