cmake_minimum_required(VERSION 2.8.3)
project(knowrob_cad_models)

find_package(catkin REQUIRED COMPONENTS knowrob_common iai_cad_downloader)
message(STATUS ${iai_cad_downloader_INCLUDE_DIRS})


# check out the CAD models from the IAI repository
execute_process(COMMAND rosrun iai_cad_downloader download-cad-models.py -d models industry kitchen WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})


catkin_package(
    DEPENDS knowrob_common
)