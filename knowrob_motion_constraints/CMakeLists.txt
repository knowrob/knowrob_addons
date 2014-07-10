
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_motion_constraints)

find_package(catkin REQUIRED COMPONENTS rosjava_jni rospy knowrob_cad_models tf json_prolog mod_srdl)


add_message_files(
  FILES
  # TODO: List your msg files here
)
add_service_files(
  FILES
  # TODO: List your msg files here
)


generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)


catkin_package(
    DEPENDS rosjava_jni rospy knowrob_cad_models tf json_prolog mod_srdl
)