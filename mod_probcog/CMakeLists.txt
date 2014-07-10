cmake_minimum_required(VERSION 2.8.3)
project(mod_probcog)

find_package(catkin REQUIRED COMPONENTS comp_temporal comp_spatial)

catkin_package(
    DEPENDS comp_temporal comp_spatial
)