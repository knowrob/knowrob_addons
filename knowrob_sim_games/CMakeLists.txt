
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_sim_games)

find_package(catkin REQUIRED rosjava_build_tools rosprolog knowrob_vis knowrob_mongo)

catkin_rosjava_setup(installDist publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
  DEPENDS knowrob_vis knowrob_mongo
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
