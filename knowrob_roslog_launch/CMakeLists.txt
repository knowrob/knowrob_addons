cmake_minimum_required(VERSION 2.8.3)
project(knowrob_roslog_launch)

find_package(catkin REQUIRED COMPONENTS mod_execution_trace iai_maps json_prolog knowrob_vis mjpeg_server rosbridge_server beliefstate mongodb_log knowrob_motion_constraints)



catkin_package(
    DEPENDS mod_execution_trace iai_maps json_prolog knowrob_vis mjpeg_server rosbridge_server beliefstate mongodb_log knowrob_motion_constraints
)