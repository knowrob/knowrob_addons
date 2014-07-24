cmake_minimum_required(VERSION 2.8.3)
project(comp_germandeli)

find_package(catkin REQUIRED COMPONENTS knowrob_common)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${PROJECT_SOURCE_DIR}/owl/germandeli.tar.gz WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/owl/)
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${PROJECT_SOURCE_DIR}/owl/germandeli-small.tar.gz WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/owl/)

catkin_package(
    DEPENDS knowrob_common
)