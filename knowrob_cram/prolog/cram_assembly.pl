
:- module(cram_assembly,
    [
      cram_assembly_initialize/3,
      cram_assembly_next_action/2,
      cram_assembly_apply_connection/2,
      cram_assembly_apply_grasp/3,
      cram_assembly_apply_ungrasp/3,
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
:- use_module(library('knowrob_math')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/knowrob_assembly.owl').

:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).

:- dynamic cram_agenda_actions/2.
:- rdf_meta cram_assembly_initialize(r,r,r),
            cram_assembly_next_action(r,t),
            cram_assembly_apply_connection(r,r),
            cram_assembly_apply_grasp(r,r,r),
            cram_assembly_apply_ungrasp(r,r,r),
            cram_assembly_designator(r,-).

% TODO(DB): grasping only breaks connections to fixtures.
%           can we express somehow when this happens for non permanent connections?

%% cram_assembly_initialize(+AssemblageType, +Strategy, -Agenda) is det.
%
cram_assembly_initialize(AssemblageType, Strategy, Agenda) :-
  rdf_instance_from_class(AssemblageType, Assemblage),
  rdf_assert(Assemblage, rdf:type, owl:'NamedIndividual'),
  agenda_create(Assemblage, Strategy, Agenda).

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
  subassemblage(Parent, Assemblage),
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
  
%% cram_assembly_primary_part(+Assemblage, -PrimaryPart, -SecondaryParts) is det.
%
% Find the primary part of an assembly action, i.e., the part that needs to be moved into some other parts.
% Prefer part with minimum number of blocked affordances, and parts not attached to a fixture.
% Furthermore require parts with non blocked grasping affordances.
% Will fail if none of the parts is graspable or connected to a graspable object.
%
cram_assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts) :-
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
  once(assemblage_part_links_fixture(Part, _)), !.
part_attached_to_fixture(_Part, 0).

part_blocked_affordances(Part, Blocked) :-
  findall(Aff, assemblage_part_blocked_affordance(Part,Aff), Blocked).

%% cram_assembly_apply_connection(+PrimaryObject, +Connection) is det.
%
% Make PrimaryObject reference object of its TF parent frames and
% make TF frame of PrimaryObject relative to reference object of the
% connection.
%
cram_assembly_apply_connection(PrimaryObject, Connection) :-
  %%%% input checking
  ground(PrimaryObject), ground(Connection),
  assemblage_mechanical_part(PrimaryObject),
  assemblage_connection(Connection),
  %%%%
  assemblage_remove_fixtures(PrimaryObject),
  once(owl_has(Connection, knowrob_assembly:'usesTransform', TransformId)),
  transform_data(TransformId, TransformData),
  assemblage_part_make_reference(PrimaryObject, Parents),
  assemblage_connection_reference(Connection, TransformId, ReferenceObject),
  belief_at_internal(PrimaryObject, TransformData, ReferenceObject),
  belief_marker_update([PrimaryObject|Parents]).

%% cram_assembly_apply_grasp(+GraspedObject, +Gripper, +GraspSpec) is det.
%
% Make PrimaryObject reference object of its TF parent frames and
% ensure that all objects physically connected to GraspedObject are
% also connected via TF.
% Finall apply the transform from GraspSpec on GraspedObject relative to Gripper.
%
cram_assembly_apply_grasp(GraspedObject, Gripper, GraspSpec) :-
  %%%% input checking
  ground(GraspedObject), ground(Gripper), 
  once((
    rdf_has(GraspedObject, knowrob_assembly:'hasAffordance', GraspedAffordance),
    owl_has(GraspedAffordance, knowrob_assembly:'graspAt', GraspSpec)
  )),
  assemblage_mechanical_part(GraspedObject),
  cram_assembly_gripper(Gripper),
  %%%%
  rdf_has(GraspSpec, paramserver:'hasGraspTransform', TransformId),
  transform_data(TransformId, TransformData),
  % retract connections to fixed objects
  assemblage_remove_fixtures(GraspedObject),
  % there could be objects with transforms not connected to GraspedObject.
  % find the physical bridges to tf unconnected objects and make
  % tf unconnected objects transforms relative to the bridge.
  findall(X, assemblage_part_connect_transforms(GraspedObject, X), DirtyUnconnected),
  % invert the transform topology along the parent frame relation.
  % GraspedObject is then reference of all phsically connected parts
  assemblage_part_make_reference(GraspedObject, Parents),
  % apply grasp transform on grasped object
  belief_at_internal(GraspedObject, TransformData, Gripper),
  % accumulate list of dirty objects and cause beliefstate to republich TF frames
  findall(X, ( member(X, [GraspedObject|Parents]) ;
    ( member(List, DirtyUnconnected), member(X,List) )), Dirty),
  belief_marker_update(Dirty),
  % assert temporary connections that consume affordances blocked by the grasp
  cram_assembly_block_grasp_affordances(GraspedObject, GraspSpec).

cram_assembly_gripper(Gripper) :-
  % just require that the object has a TF name for now
  rdf_has(Gripper, srdl2comp:'urdfName', literal(_)).
  
%% cram_assembly_apply_ungrasp(+GraspedObject, Gripper, GraspSpec) is det.
%
cram_assembly_apply_ungrasp(GraspedObject, Gripper, GraspSpec) :-
  %%%% input checking
  ground(GraspedObject), ground(Gripper), ground(GraspSpec),
  assemblage_mechanical_part(GraspedObject),
  cram_assembly_gripper(Gripper),
  %%%%
  % make GraspedObject absolute if still relative to gripper
  rdf_has(GraspedObject, paramserver:'hasTransform', TransformId),
  ( rdf_has(TransformId, knowrob:'relativeTo', Gripper) -> (
    rdf_has(GraspedObject, srdl2comp:'urdfName', literal(ObjFrame)),
    % FIXME: hardcoded map frame name
    get_current_tf('map', ObjFrame, Tx,Ty,Tz, Rx,Ry,Rz,Rw),
    belief_at_update(GraspedObject, ([Tx,Ty,Tz], [Rx,Ry,Rz,Rw]))
  ) ; true ),
  % retract temporary connections that consume affordances blocked by the grasp
  cram_assembly_unblock_grasp_affordances(GraspedObject, GraspSpec).
  
% TODO: handle additional affordances blocked during the grasp
% NOTE: planner should not be called during grasp!
cram_assembly_block_grasp_affordances(GraspedObject, GraspSpec) :-
  % block the affordance that is grasped
  once((
    rdf_has(GraspedObject, knowrob_assembly:'hasAffordance', GraspedAffordance),
    owl_has(GraspedAffordance, knowrob_assembly:'graspAt', GraspSpec)
  )),
  rdf_instance_from_class(knowrob_assembly:'GraspingConnection', GraspConnection),
  rdf_assert(GraspConnection, knowrob_assembly:'consumesAffordance', GraspedAffordance).
cram_assembly_unblock_grasp_affordances(GraspedObject, GraspSpec) :-
  once((
    rdf_has(GraspedObject, knowrob_assembly:'hasAffordance', GraspedAffordance),
    owl_has(GraspedAffordance, knowrob_assembly:'graspAt', GraspSpec)
  )),
  rdf_has(GraspConnection, knowrob_assembly:'consumesAffordance', GraspedAffordance),
  rdfs_individual_of(GraspConnection, knowrob_assembly:'GraspingConnection'),
  rdf_retractall(GraspConnection, _, _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Designator formatting

assemblage_designator([])         --> [].
assemblage_designator(Assemblage) -->
  { atom(Assemblage),
    cram_assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts),
    rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn) },
  ['[an, action, ['], newline,
  ['  [type: '],       enquote(connecting),            ['],'], newline,
  ['  [connection: '], connection_designator(Conn),    ['],'], newline,
  ['  [object: '],     object_designator(PrimaryPart), ['],'], newline,
  with_objects_designator(SecondaryParts),
  [']]'].

with_objects_designator([]) --> [].
with_objects_designator([Obj|Next]) -->
  ['  [with-object: '], object_designator(Obj), [']'], newline,
  with_objects_designator(Next).

connection_designator(Obj) -->
  ['[a, thing, ['], newline,
    ['    '], object_type(Obj), [','], newline,
    ['    '], object_name(Obj), newline,
  ['  '], [']]'].
object_designator(Obj) -->
  ['[an, object, ['], newline,
    ['    '], object_type(Obj), [','], newline,
    ['    '], object_name(Obj), newline,
  ['  ]]'].
object_type(Obj) --> { atom(Obj), rdfs_type_of(Obj,Type) },
  ['[type: '], enquote(Type), [']'].
object_name(Obj) -->
  ['[name: '], enquote(Obj), [']'].

enquote(X) --> ['\''], [X], ['\''].
newline    --> ['\n'].
