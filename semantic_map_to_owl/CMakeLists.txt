cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_to_owl)

find_package(catkin REQUIRED COMPONENTS rosjava_jni geometry_msgs ias_knowledge_base knowrob_common)


add_message_files(
  FILES
  # TODO: List your msg files here
)
add_service_files(
  FILES
  # TODO: List your msg files here
)


# generate launch script
rospack_add_java_executable(SemanticMapToOWL edu.tum.cs.ias.knowrob.map.SemanticMapToOWL)
rospack_add_java_executable(SemanticMapToOWLTestClient edu.tum.cs.ias.knowrob.map.SemanticMapToOWLTestClient)
rospack_add_java_executable(SemanticMapEditor edu.tum.cs.ias.knowrob.map.SemanticMapEditor)


catkin_package(
    DEPENDS rosjava_jni geometry_msgs ias_knowledge_base knowrob_common
)