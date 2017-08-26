
:- module(cram_assembly,
    [
      cram_initialize_assembly/2,
      cram_next_assembly_action/2
    ]).

:- register_ros_package(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/knowrob_assembly.owl').

:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).

:- dynamic cram_agenda_actions/2.
:- rdf_meta cram_initialize_assembly(r,r),
            cram_next_assembly_action(r,t).

cram_initialize_assembly(AssemblageType, Agenda) :-
  rdf_instance_from_class(AssemblageType, Assemblage),
  rdf_assert(Assemblage, rdf:type, owl:'NamedIndividual'),
  agenda_create(Assemblage, 'http://knowrob.org/kb/battat_airplane_test.owl#AgendaStrategy_1', Agenda).

cram_next_assembly_action(Agenda, ActionDesignator) :-
  cram_assembly_pop_action(Agenda, ActionDesignator), !.

cram_next_assembly_action(Agenda, ActionDesignator) :-
  agenda_pop(Agenda, Item, Descr),
  agenda_item_subject(Item, S),
  agenda_perform(Agenda, Item, Descr),
  ( cram_assembly_action_possible(S, Assemblage) ->
  ( cram_assembly_possible_actions(Assemblage, ActionSequence),
    cram_assembly_update_actions(Agenda, ActionSequence),
    cram_assembly_pop_action(Agenda, ActionDesignator) ) ;
  ( cram_next_assembly_action(Agenda, ActionDesignator) )).

cram_assembly_update_actions(Agenda, ActionSequence) :-
  retractall(cram_agenda_actions(Agenda, _)),
  assertz(cram_agenda_actions(Agenda, ActionSequence)).

cram_assembly_pop_action(Agenda, ActionDesignator) :-
  cram_agenda_actions(Agenda, [(ActionAssemblage,ActionDesignator)|Rest]),
  cram_assembly_update_actions(Agenda, Rest),
  rdf_assert(ActionAssemblage, '__PERFORMED', 'true'). % FIXME: proper predicate

cram_assembly_action_possible(Subject, Assemblage) :-
  rdfs_individual_of(Subject, knowrob_assembly:'AssemblyConnection'),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Subject),
  assemblage_specified(Assemblage).

cram_assembly_possible_actions(Assemblage, ActionSequence) :-
  findall(X, cram_assembly_finalized_parent(Assemblage, X), Finalized),
  list_to_set(Finalized,Finalized_set),
  cram_assembly_format_designators(Finalized_set, ActionSequence).
cram_assembly_finalized_parent(Assemblage, Finalized) :-
  Finalized=Assemblage ; (
  assemblage_parent(Assemblage, Parent),
  assemblage_specified(Parent),
  cram_assembly_finalized_parent(Parent, Finalized)).

cram_assembly_format_designators([], []).
cram_assembly_format_designators([Assemblage|X], [(Assemblage,Designator)|Y]) :-
  phrase(assemblage_designator(Assemblage), Tokens),
  atomic_list_concat(Tokens, '', Designator),
  cram_assembly_format_designators(X,Y).
  
cram_assemblage_parts(Assemblage, PrimaryPart, SecondaryParts) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  % find part with minimum number of blocked affordances
  findall(NumBlocked-Part, (
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance),
    rdf_has(Part, knowrob_assembly:'hasAffordance', Affordance),
    part_blocked_affordances(Part, Blocked),
    length(Blocked, NumBlocked)
  ), BlockedParts),
  sort(BlockedParts, [_-PrimaryPart|_]),
  % find secondary parts
  findall(Secondary, (
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance),
    rdf_has(Secondary, knowrob_assembly:'hasAffordance', Affordance),
    Secondary \= PrimaryPart
  ), SecondaryParts_list),
  list_to_set(SecondaryParts_list, SecondaryParts).

part_blocked_affordances(Part, Blocked) :-
  findall(Aff, (
    rdf_has(Part, knowrob_assembly:'hasAffordance', Aff),
    % TODO: handle non connections blocking affordances
    rdf_has(Connection, knowrob_assembly:'blocksAffordance', Aff),
    rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
    rdf_has(Assemblage, '__PERFORMED', 'true') % FIXME: proper predicate
  ), Blocked).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Designator formatting

assemblage_designator([])         --> [].
assemblage_designator(Assemblage) -->
  { atom(Assemblage),
    cram_assemblage_parts(Assemblage, PrimaryPart, SecondaryParts),
    rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn) },
  ['[an, action,'], newline,
  ['  [type: \'connecting\'],'], newline,
  ['  [connection: '], connection_designator(Conn),    ['],'], newline,
  ['  [object: '],     object_designator(PrimaryPart), ['],'], newline,
  with_objects_designator(SecondaryParts),
  [']'].

with_objects_designator([]) --> [].
with_objects_designator([Obj|Next]) -->
  ['  [with-object: '], object_designator(Obj), [']'], newline,
  with_objects_designator(Next).

connection_designator(Obj) -->
  ['[a, thing,'], newline,
    ['    '], object_type(Obj), [','], newline,
    ['    '], object_name(Obj), newline,
  ['  '], [']'].
object_designator(Obj) -->
  ['[an, object,'], newline,
    ['    '], object_type(Obj), [','], newline,
    ['    '], object_name(Obj), newline,
  ['  ]'].
object_type(Obj) --> { atom(Obj), rdfs_type_of(Obj,Type) },
  ['[type: '], ['\''], [Type], ['\''], [']'].
object_name(Obj) -->
  ['[name: '], ['\''], [Obj], ['\''], [']'].

newline --> ['\n'].