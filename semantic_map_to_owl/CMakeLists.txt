cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_to_owl)

find_package(catkin REQUIRED rosjava_build_tools geometry_msgs knowrob_common message_generation)


catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

add_message_files(
  FILES SemMap.msg SemMapObject.msg
)
add_service_files(
  FILES GenerateSemanticMapOWL.srv
)


generate_messages(
   DEPENDENCIES geometry_msgs
)

catkin_package(
    DEPENDS geometry_msgs knowrob_common
)


##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
