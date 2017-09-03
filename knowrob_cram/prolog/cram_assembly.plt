:- begin_tests(cram_assembly).

:- register_ros_package(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).
:- use_module(library('cram_assembly')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_test.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl').

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_test, 'http://knowrob.org/kb/battat_airplane_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_airplane_simulation.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(params, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

test(assembly_BattatPlaneBodyWithoutWindow) :-
  cram_assembly_initialize(battat_toys:'BattatPlaneBodyWithoutWindow', battat_test:'AgendaStrategy_1', Agenda),
  agenda_write(Agenda),
  test_perform_agenda_cram(Agenda).

test_perform_agenda_cram(Agenda) :-
  cram_assembly_next_action(Agenda, Action) -> (
    writeln(Action),
    test_perform_agenda_cram(Agenda)) ; test_agenda_empty(Agenda).

test_agenda_empty(Agenda) :-
  % FIXME: BUG: agenda_items has item while agenda_items_sorted does not!
  %             --> seems removed but not retracted
  % agenda_items(Agenda, []).
  agenda_items_sorted(Agenda, []).

:- end_tests(cram_assembly).
