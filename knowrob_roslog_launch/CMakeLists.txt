cmake_minimum_required(VERSION 2.8.3)
project(knowrob_roslog_launch)

find_package(catkin REQUIRED COMPONENTS knowrob_cram json_prolog knowrob_vis mjpeg_server rosbridge_server knowrob_motion_constraints)



catkin_package(
    DEPENDS knowrob_cram json_prolog knowrob_vis mjpeg_server rosbridge_server knowrob_motion_constraints
)