##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_beliefstate)

##############################################################################
# Catkin
##############################################################################

#find_package(catkin REQUIRED rosjava_build_tools knowrob_common)
#find_package(catkin REQUIRED knowrob_common knowrob_paramserver COMPONENTS message_generation)
find_package(catkin REQUIRED COMPONENTS rosprolog knowrob_common knowrob_paramserver
             message_generation
             )

catkin_python_setup()

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})
message(STATUS "SWIPL found.")

find_package(RapidJSON QUIET)
if(${RapidJSON_FOUND})
    add_definitions(-DWITH_JSON -DRAPIDJSON_HAS_STDSTRING)
endif()

add_service_files(
   FILES
   DirtyObject.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
)

#catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

#include_directories(
#        include
#        ${SWIPL_INCLUDE_DIRS}
#        ${catkin_INCLUDE_DIRS}
#        )

catkin_package(
	DEPENDS knowrob_common knowrob_paramserver knowrob_assembly #roscpp roslib rospack
)

#add_library(kr_beliefstate SHARED 
#	src/kr_beliefstate.cpp)
#add_dependencies(kr_beliefstate knowrob_beliefstate_generate_messages_cpp)
#target_link_libraries(kr_beliefstate
#	${SWIPL_LIBRARIES}
#	${catkin_LIBRARIES})

#add_executable(krb_tf_listener
#               src/tf_listener.cpp)
#add_dependencies(krb_tf_listener knowrob_beliefstate_generate_messages_cpp)
#target_link_libraries(krb_tf_listener
#	${catkin_LIBRARIES})

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
#install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
#        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
#
#install(DIRECTORY latex/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/latex)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
