# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(comp_missingobj)

find_package(catkin REQUIRED COMPONENTS mod_probcog)

catkin_package(
    DEPENDS mod_probcog
)