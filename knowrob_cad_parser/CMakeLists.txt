##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_cad_parser)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools knowrob_common)

catkin_rosjava_setup(installDist publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
  DEPENDS knowrob_common
)

##############################################################################
# Installation
##############################################################################

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
