%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(mod_vis).
:- register_ros_package(ias_semantic_map).
:- register_ros_package(mod_action_visualization).

:- use_module(library('comp_action_visualization')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('package://mod_action_visualization/owl/action_visualization.owl').
:- rdf_db:rdf_register_ns(self_info, 'http://knowrob.org/kb/action_visualization.owl#', [keep(true)]).

