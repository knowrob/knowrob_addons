cmake_minimum_required(VERSION 2.8.3)
project(knowrob_roslog_launch)

find_package(catkin REQUIRED COMPONENTS knowrob_sim knowrob_cram json_prolog knowrob_vis knowrob_meshes rosbridge_server knowrob_plan_summary)



catkin_package(
    DEPENDS knowrob_sim knowrob_cram json_prolog knowrob_vis knowrob_meshes  rosbridge_server knowrob_plan_summary
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
