%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies


:- register_ros_package(knowrob_roslog_launch).
:- register_ros_package(mod_execution_trace).
:- register_ros_package(knowrob_vis).
:- register_ros_package(iai_maps).
:- register_ros_package(mod_srdl).
:- owl_parse('/home/demo/work/ros/groovy/rosbuild_ws/knowrob/ias_semantic_map/owl/ias_semantic_map.owl', false, false, true).
:- owl_parse('/var/roslog/exp-2014-03-24_13-18-34/cram_log.owl', false, false, true).
:- visualisation_canvas.
:- diagram_canvas.

