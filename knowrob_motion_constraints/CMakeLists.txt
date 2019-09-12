
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_motion_constraints)

find_package(catkin REQUIRED COMPONENTS rospy tf srdl)


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
    DEPENDS rospy tf srdl
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)