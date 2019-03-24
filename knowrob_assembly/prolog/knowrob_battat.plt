:- begin_tests(knowrob_battat).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_strategy.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl', belief_state).

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_strategy, 'http://knowrob.org/kb/battat_strategy.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_airplane_simulation.owl#', [keep(true)]).

test(assembly_battat_beliefstate_filled) :-
  belief_existing_objects(KnownObjects),
  KnownObjects \= [].
  
test('ogp_execute_assembly(BattatPlaneBodyWithoutWindow)') :-
  ogp_execute_assembly(assembly_test:'OGP1',
    battat_toys:'BattatPlaneBodyWithoutWindow', BodyOnChassis),
  % test whether BodyOnChassis is a full assemblage
  ogp_agenda_create(assembly_test:'OGP1',BodyOnChassis,A0),
  ogp_agenda_isEmpty(A0).

%test(assembly_BattatPlaneBodyWithoutWindow) :-
  %rdf_instance_from_class(battat_toys:'BattatPlaneBodyWithoutWindow', BattatPlaneBody),
  %rdf_assert(BattatPlaneBody, rdf:type, owl:'NamedIndividual'),
  %agenda_create(BattatPlaneBody, battat_strategy:'AgendaStrategy_1', Agenda),
  %test_perform_agenda(Agenda).

%test_perform_agenda(Agenda) :-
  %%agenda_write(Agenda),
  %(agenda_perform_next(Agenda) -> test_perform_agenda(Agenda) ; true).

:- end_tests(knowrob_battat).
