
:- module(cram_assembly,
    [
      cram_assembly_initialize/2,
      cram_assembly_next_action/2,
      cram_assembly_apply_connection/2,
      cram_assembly_apply_grasp/3,
      cram_assembly_designator/2
    ]).

:- register_ros_package(knowrob_assembly).
:- register_ros_package(knowrob_beliefstate).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).
:- use_module(library('knowrob_beliefstate')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/knowrob_assembly.owl').

:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).

:- dynamic cram_agenda_actions/2.
:- rdf_meta cram_initialize_assembly(r,r),
            cram_assembly_next_action(r,t),
            cram_assembly_apply_connection(r,r),
            cram_assembly_apply_grasp(r,r,r),
            cram_assembly_designator(r,-).

% TODO(DB): it is in general problematic that the planner has to select atomic parts along
%           the model during top-down planning (i.e., linksAssemblage inserts this)
%           before the actual action was performed.

%% cram_assembly_initialize(+AssemblageType, -Agenda) is det.
%
cram_assembly_initialize(AssemblageType, Agenda) :-
  rdf_instance_from_class(AssemblageType, Assemblage),
  rdf_assert(Assemblage, rdf:type, owl:'NamedIndividual'),
  % TODO: declar strategy in knowrob_cram
  agenda_create(Assemblage, 'http://knowrob.org/kb/battat_airplane_test.owl#AgendaStrategy_1', Agenda).

%% cram_assembly_next_action(+Agenda, -ActionDesignator) is det.
%
cram_assembly_next_action(Agenda, ActionDesignator) :-
  cram_assembly_pop_action(Agenda, ActionDesignator), !.

cram_assembly_next_action(Agenda, ActionDesignator) :-
  agenda_pop(Agenda, Item, Descr),
  agenda_item_subject(Item, S),
  agenda_perform(Agenda, Item, Descr),
  ( cram_assembly_action_possible(S, Assemblage) ->
  ( cram_assembly_possible_actions(Assemblage, ActionSequence),
    cram_assembly_update_actions(Agenda, ActionSequence),
    cram_assembly_pop_action(Agenda, ActionDesignator) ) ;
  ( cram_assembly_next_action(Agenda, ActionDesignator) )).

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

%% cram_assembly_designator(+Assemblage, -Designator) is det.
%
cram_assembly_designator(Assemblage, Designator) :-
  phrase(assemblage_designator(Assemblage), Tokens),
  atomic_list_concat(Tokens, '', Designator).

cram_assembly_format_designators([], []).
cram_assembly_format_designators([Assemblage|X], [(Assemblage,Designator)|Y]) :-
  cram_assembly_designator(Assemblage,Designator),
  cram_assembly_format_designators(X,Y).
  
%% cram_assembly_primary_part(+Assemblage, -PrimaryPart, -PrimaryPart) is det.
%
% Find the primary part of an assembly action, i.e., the part that needs to be moved into some other part
%
cram_assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts) :-
  % find part with minimum number of blocked affordances,
  % prefer parts not attached to a fixture and
  % require parts with non blocked grasping affordances
  % TODO: what if none of the parts are graspable?
  % FIXME: BUG: planner will fail when holder blocks required affordance.
  %             --> need to generate retract items in planner!!!
  findall(Fixed-NumBlocked-Part, (
    assemblage_part(Assemblage, Part),
    \+ assemblage_fixed_part(Part),
    assemblage_graspable_part(Assemblage,Part),
    part_attached_to_fixture(Part, Fixed),  % primary sort key
    part_blocked_affordances(Part, Blocked),
    length(Blocked, NumBlocked)             % secondary sort key
  ), BlockedParts),
  sort(BlockedParts, [_-_-PrimaryPart|_]),
  % find secondary parts
  findall(Secondary, (
    assemblage_part(Assemblage, Secondary),
    Secondary \= PrimaryPart
  ), SecondaryParts_list),
  list_to_set(SecondaryParts_list, SecondaryParts).

part_attached_to_fixture(Part, 1) :-
  assemblage_part_links_fixtures(Part, Fixtures), Fixtures \= [], !.
part_attached_to_fixture(_Part, 0).

part_blocked_affordances(Part, Blocked) :-
  findall(Aff, assemblage_part_blocked_affordance(Part,Aff), Blocked).

%% cram_assembly_apply_connection(+PrimaryObject, +Connection) is det.
%
cram_assembly_apply_connection(PrimaryObject, Connection) :-
% TODO: ungrasp object:
%   - check if current transform is relative to a connected object
%   - update all transforms relativeTo PrimaryObject
%   - unblock affordances

  once(owl_has(Connection, knowrob_assembly:'usesTransform', TransformId)),
  transform_data(TransformId, TransformData),
  connection_reference_object(Connection, TransformId, ReferenceObject),
  belief_at(PrimaryObject, TransformData, ReferenceObject).

transform_data(TransformId, (Translation, Rotation)) :-
  % TODO: should be part of knowrob_common
  rdf_has(TransformId, knowrob:'translation', literal(type(_,Translation_atom))),
  rdf_has(TransformId, knowrob:'quaternion', literal(type(_,Rotation_atom))),
  knowrob_math:parse_vector(Translation_atom, Translation),
  knowrob_math:parse_vector(Rotation_atom, Rotation).

connection_reference_object(_Connection, TransformId, ReferenceObj) :-
  rdf_has(TransformId, knowrob:'relativeTo', ReferenceObj), !.
connection_reference_object(Connection, TransformId, ReferenceObj) :-
  % FIXME: won't work when multiple instances of the reference object class are linked in the connection
  rdfs_individual_of(TransformId, Restr),
  rdfs_individual_of(Restr, owl:'Restriction'),
  rdf_has(Restr, owl:'onProperty', knowrob:'relativeTo'),
  rdf_has(Restr, owl:'onClass', ReferenceCls),
  rdf_has(Connection, knowrob_assembly:'consumesAffordance', Aff),
  rdf_has(ReferenceObj, knowrob_assembly:'hasAffordance', Aff),
  owl_individual_of(ReferenceObj,ReferenceCls), !.
  
%% cram_assembly_apply_grasp(+GraspedObject, +Gripper, +GraspSpecification) is det.
%
cram_assembly_apply_grasp(GraspedObject, Gripper, GraspSpecification) :-
  % retract connections to fixed objects
  assemblage_part_links_fixtures(GraspedObject, FixedParts),
  forall((
    member(Fixture,FixedParts),
    rdf_has(Fixture, knowrob_assembly:'hasAffordance', Aff1),
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Aff1),
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Aff2),
    % FIXME: not grasped object but some connected to grasped object
    rdf_has(GraspedObject, knowrob_assembly:'hasAffordance', Aff2)
  ), assemblage_destroy(Connection)), % FIXME: not safe to do outside planner
  % apply grasp transform
  belief_at_gripper(GraspedObject, Gripper, GraspSpecification),
  % TODO  grasp blocks affordances, grasp specification names them
  true.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Designator formatting

assemblage_designator([])         --> [].
assemblage_designator(Assemblage) -->
  { atom(Assemblage),
    cram_assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts),
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
