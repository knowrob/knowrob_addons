%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(mod_vis).
:- register_ros_package(ias_semantic_map).
:- register_ros_package(mod_action_visualization).

:- use_module(library('comp_action_visualization')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('/home/asil/asil-ros-pkg/knowrob/mod_action_visualization/owl/action_visualization.owl', false, false, true).
:- rdf_db:rdf_register_ns(self_info, 'http://ias.cs.tum.edu/kb/action_visualization.owl#', [keep(true)]).

