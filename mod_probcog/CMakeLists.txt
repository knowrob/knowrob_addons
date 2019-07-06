cmake_minimum_required(VERSION 2.8.3)
project(mod_probcog)

find_package(catkin REQUIRED COMPONENTS comp_spatial)

catkin_package(
    DEPENDS comp_spatial
)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
