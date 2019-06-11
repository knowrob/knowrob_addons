##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_sim)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED knowrob_mongo)

catkin_package(
  DEPENDS knowrob_mongo
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
