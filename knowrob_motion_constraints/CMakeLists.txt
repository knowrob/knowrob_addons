
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_motion_constraints)

find_package(catkin REQUIRED COMPONENTS rospy tf knowrob_srdl)


add_message_files(
  FILES MotionConstraint.msg  MotionConstraintTemplate.msg  MotionPhase.msg  MotionTask.msg
  # TODO: List your msg files here
)
add_service_files(
  FILES ConstraintsToOWL.srv
)


generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)


catkin_package(
    DEPENDS rosjava_jni rospy tf knowrob_srdl
)