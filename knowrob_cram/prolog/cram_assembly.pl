
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

% TODO(DB): There might be some issues about asserting assemblages before CRAM actually performed the action.
%           SOLUTION: assert into separate planning KB and project back when action was performed.

cram_initialize_assembly(AssemblageType, Agenda) :-
  rdf_instance_from_class(AssemblageType, Assemblage),
  rdf_assert(Assemblage, rdf:type, owl:'NamedIndividual'),
  % TODO: declar strategy in knowrob_cram
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
  rdf_assert(ActionAssemblage, knowrob_assembly:'assemblageEstablished', literal(type(xsd:boolean,'true'))).

cram_assembly_action_possible(Subject, Assemblage) :-
  rdfs_individual_of(Subject, knowrob_assembly:'AssemblyConnection'),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Subject),
  assemblage_specified(Assemblage).

cram_assembly_possible_actions(Assemblage, ActionSequence) :-
  findall(X, cram_assembly_specified_parent(Assemblage, X), Finalized),
  list_to_set(Finalized,Finalized_set),
  cram_assembly_format_designators(Finalized_set, ActionSequence).
cram_assembly_specified_parent(Assemblage, Finalized) :-
  Finalized=Assemblage ; (
  assemblage_parent(Assemblage, Parent),
  assemblage_specified(Parent),
  cram_assembly_specified_parent(Parent, Finalized)).

cram_assembly_format_designators([], []).
cram_assembly_format_designators([Assemblage|X], [(Assemblage,Designator)|Y]) :-
  phrase(assemblage_designator(Assemblage), Tokens),
  atomic_list_concat(Tokens, '', Designator),
  cram_assembly_format_designators(X,Y).
  
cram_assemblage_parts(Assemblage, PrimaryPart, SecondaryParts) :-
  % find part with minimum number of blocked affordances,
  % prefer parts not attached to a fixture and
  % require parts with non blocked grasping affordances
  % TODO: what if none of the parts are graspable?
  % FIXME: BUG: planner will fail when holder blocks required affordance.
  %             --> need to retract connections blocking required affordance if possible! 
  %             --> PROBLEM: need to retract for planning before the object was actually detached from holder
  findall(Fixed-NumBlocked-Part, (
    assemblage_atomic_part(Assemblage, Part),
    \+ assemblage_fixture(Part),
    assemblage_graspable(Part),
    part_attached_to_fixture(Part, Fixed),  % primary sort key
    part_blocked_affordances(Part, Blocked),
    length(Blocked, NumBlocked)             % secondary sort key
  ), BlockedParts),
  sort(BlockedParts, [_-_-PrimaryPart|_]),
  % find secondary parts
  findall(Secondary, (
    assemblage_atomic_part(Assemblage, Secondary),
    Secondary \= PrimaryPart
  ), SecondaryParts_list),
  list_to_set(SecondaryParts_list, SecondaryParts).

part_attached_to_fixture(Part, 1) :-
  part_established_assemblages(Part, Assemblages),
  member(Assemblage, Assemblages),
  assemblage_atomic_part(Assemblage, Fixture),
  assemblage_fixture(Fixture), !.
part_attached_to_fixture(_Part, 0).

part_established_assemblages(Part, Assemblages) :-
  part_established_assemblages(Part, [], Assemblages_list),
  list_to_set(Assemblages_list, Assemblages).
part_established_assemblages(Part, Blacklist, LinkedAssemblages) :-
  % find all assemblages with connections consuming an affordance of the part
  findall(Direct_assemblage, (
    assemblage_atomic_part(Direct_assemblage, Part),
    assemblage_established(Direct_assemblage),
    \+ member(Direct_assemblage,Blacklist)
  ), LinkedAssemblages_direct),
  append(Blacklist, LinkedAssemblages_direct, NewBlacklist),
  % find all other parts used in linked assemablages
  findall(Direct_part, (
    member(Direct_assemblage,LinkedAssemblages_direct),
    assemblage_atomic_part(Direct_assemblage, Direct_part),
    Direct_part \= Part
  ), LinkedParts_list),
  list_to_set(LinkedParts_list, LinkedParts),
  % recursively find linked assemblages for linked parts
  findall(LinkedAssemblages_sibling, (
    member(Direct_part,LinkedParts),
    part_established_assemblages(Direct_part,NewBlacklist,LinkedAssemblages_sibling)
  ), LinkedAssemblages_indirect),
  flatten([LinkedAssemblages_direct|LinkedAssemblages_indirect], LinkedAssemblages).

part_blocked_affordances(Part, Blocked) :-
  findall(Aff, assemblage_blocked_affordance(Part,Aff), Blocked).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Designator formatting

assemblage_designator([])         --> [].
assemblage_designator(Assemblage) -->
  { atom(Assemblage),
    cram_assemblage_parts(Assemblage, PrimaryPart, SecondaryParts),
    rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn) },
  ['[an, action,'], newline,
  ['  [type: '],       enquote(connecting),            ['],'], newline,
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
  ['[type: '], enquote(Type), [']'].
object_name(Obj) -->
  ['[name: '], enquote(Obj), [']'].

enquote(X) --> ['\''], [X], ['\''].
newline    --> ['\n'].
