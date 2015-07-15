
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_rs_components)

find_package(catkin REQUIRED rosprolog knowrob_common knowrob_actions knowrob_srdl)


catkin_package(
    DEPENDS rosprolog knowrob_common knowrob_actions knowrob_srdl
)

install(DIRECTORY example/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/example)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
