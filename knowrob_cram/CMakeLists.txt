cmake_minimum_required(VERSION 2.8.3)
project(knowrob_cram)

find_package(catkin REQUIRED COMPONENTS knowrob_mongo)

catkin_package(
    DEPENDS knowrob_mongo
)