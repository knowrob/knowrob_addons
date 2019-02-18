##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_kautham)

##############################################################################
# Catkin
##############################################################################

#find_package(catkin REQUIRED rosjava_build_tools knowrob_common knowrob_paramserver)
find_package(catkin REQUIRED COMPONENTS
  tf
  knowrob_common
  knowrob_paramserver
  knowrob_assembly
  roscpp
  roslib
  std_msgs
  genmsg
  geometry_msgs
  message_generation
  kautham
  yumik)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

include_directories(${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

generate_messages(DEPENDENCIES roscpp std_msgs geometry_msgs kautham)

catkin_package(
        INCLUDE_DIRS  include
	DEPENDS knowrob_common knowrob_paramserver knowrob_assembly roscpp roslib geometry_msgs std_msgs kautham tf
)

add_definitions( "--std=gnu++11" )
add_library(kauthamwrapper SHARED src/kauthamwrapper.cpp)
target_link_libraries(kauthamwrapper ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(kauthamwrapper ${catkin_EXPORTED_TARGETS} )

## C++ examples
add_executable(world2map src/world2map.cpp)
target_link_libraries(world2map ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(world2map ${catkin_EXPORTED_TARGETS})
endif()

## Install C++ Examples
install(TARGETS world2map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##############################################################################
# Installation
##############################################################################

install(TARGETS kauthamwrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

