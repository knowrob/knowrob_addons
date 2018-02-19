##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(knowrob_kautham)

##############################################################################
# Catkin
##############################################################################

#find_package(catkin REQUIRED rosjava_build_tools knowrob_common knowrob_paramserver)
find_package(catkin REQUIRED COMPONENTS knowrob_common knowrob_paramserver knowrob_planning roscpp roslib rospack kautham)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.

catkin_package(
	DEPENDS knowrob_common knowrob_paramserver knowrob_planning roscpp roslib rospack kautham
)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

include_directories(include ${SWIPL_INCLUDE_DIRS} ${CATKIN_INCLUDE_DIRS})
add_library(kauthamwrapper SHARED src/kauthamwrapper.cpp)
target_link_libraries(kauthamwrapper ${SWIPL_LIBRARIES} ${CATKIN_LIBRARIES})

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

