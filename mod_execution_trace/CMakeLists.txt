cmake_minimum_required(VERSION 2.8.3)
project(mod_execution_trace)

find_package(catkin REQUIRED COMPONENTS knowrob_cram knowrob_mongo std_msgs sensor_msgs designator_integration_msgs)

catkin_package(
    DEPENDS knowrob_cram knowrob_mongo std_msgs sensor_msgs designator_integration_msgs
)