
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_rs_components)

find_package(catkin REQUIRED rosprolog knowrob_common knowrob_actions knowrob_srdl)


catkin_package(
    DEPENDS rosprolog knowrob_common knowrob_actions knowrob_srdl
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
