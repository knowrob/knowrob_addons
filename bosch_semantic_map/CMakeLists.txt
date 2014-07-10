cmake_minimum_required(VERSION 2.8.3)
project(bosch_semantic_map)

find_package(catkin REQUIRED COMPONENTS comp_spatial json_prolog)

catkin_package(
    DEPENDS comp_spatial json_prolog
)