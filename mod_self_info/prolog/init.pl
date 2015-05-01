%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(mod_vis).
:- register_ros_package(ias_semantic_map).
:- register_ros_package(mod_self_info).

:- use_module(library('comp_self_info')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('package://mod_self_info/owl/self_info.owl', false, false, true).
:- rdf_db:rdf_register_ns(self_info, 'http://knowrob.org/kb/self_info.owl#', [keep(true)]).

