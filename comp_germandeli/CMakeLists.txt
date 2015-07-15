cmake_minimum_required(VERSION 2.8.3)
project(comp_germandeli)

find_package(catkin REQUIRED COMPONENTS knowrob_common)

execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${PROJECT_SOURCE_DIR}/owl/germandeli.tar.gz WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/owl/)
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${PROJECT_SOURCE_DIR}/owl/germandeli-small.tar.gz WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/owl/)

catkin_package(
    DEPENDS knowrob_common
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)