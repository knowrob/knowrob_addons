%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_robosherlock).

:- use_module(library(rs_plan_pipeline)).

:- owl_parser:owl_parse('package://knowrob_robosherlock/owl/rs_components.owl').
:- rdf_db:rdf_register_prefix(rs_components, 'http://knowrob.org/kb/rs_components.owl#', [keep(true)]).


