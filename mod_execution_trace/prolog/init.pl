%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(comp_temporal).
:- register_ros_package(knowrob_cram).
:- register_ros_package(mod_execution_trace).

:- use_module(library('comp_execution_trace')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://mod_execution_trace/owl/knowrob_cram.owl').
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

