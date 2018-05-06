:- begin_tests(cram_assembly).

:- register_ros_package(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/swrl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).
:- use_module(library('cram_assembly')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_strategy_test.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_scene1.owl').

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_strategy, 'http://knowrob.org/kb/battat_strategy.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_simulation.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(params, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).


test(assembly_BattatPlaneBodyWithoutWindow) :-
  owl_instance_from_class(battat_toys:'BattatPlaneBodyWithoutWindow', Assemblage),
  agenda_create(Assemblage, battat_strategy:'AgendaStrategy_1', Agenda),
  agenda_write(Agenda),
  test_perform_agenda_cram(Agenda).

test_perform_agenda_cram(Agenda) :-
  %agenda_write(Agenda),
  (agenda_perform_next(Agenda) -> test_perform_agenda_cram(Agenda) ; true).

:- end_tests(cram_assembly).
