:- begin_tests(knowrob_battat).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_test.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl').

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_test, 'http://knowrob.org/kb/battat_airplane_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_airplane_simulation.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(params, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

test(assembly_BattatPlaneBodyWithoutWindow) :-
  rdf_instance_from_class(battat_toys:'BattatPlaneBodyWithoutWindow', BattatPlaneBody),
  rdf_assert(BattatPlaneBody, rdf:type, owl:'NamedIndividual'),
  agenda_create(BattatPlaneBody, battat_test:'AgendaStrategy_1', Agenda),
  test_perform_agenda(Agenda).

test_perform_agenda(Agenda) :-
  agenda_perform_next(Agenda) -> test_perform_agenda(Agenda) ; true.

:- end_tests(knowrob_battat).
