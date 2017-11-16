cmake_minimum_required(VERSION 2.8.3)
project(knowrob_robosherlock)


find_package(catkin REQUIRED rosprolog
             knowrob_common knowrob_actions knowrob_srdl)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        )
