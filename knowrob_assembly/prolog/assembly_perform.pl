
% This is a collection of some assembly agenda processing methods.

:- module(assembly_perform,
    [
      assembly_action_put_away/4,
      assembly_action_connecting_parts/4
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module('knowrob_assembly').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

assembly_action_connecting_parts(Connection, _Descr, _Strategy, ActionEntity) :-
  rdfs_individual_of(Connection,knowrob_assembly:'AssemblyConnection'), !,
  rdf_has(Assemblage,knowrob_assembly:usesConnection,Connection),
  assembly_action_designator_(Assemblage, ActionEntity).

assembly_action_connecting_parts(Assemblage, _, _, ActionEntity) :-
  rdfs_individual_of(Assemblage,knowrob_assembly:'Assemblage'), !,
  assembly_action_designator_(Assemblage, ActionEntity).

assembly_action_designator_(Assemblage, ActionEntity) :-
  assemblage_specified(Assemblage),
  assembly_possible_action(Assemblage, ActionAssemblage, ActionEntity),
  % HACK planner is faster then real world
  once((
    rdf_has(ActionAssemblage, knowrob_assembly:'assemblageEstablished', _);
    rdf_assert(ActionAssemblage, knowrob_assembly:'assemblageEstablished', literal(type(xsd:boolean,'true')))
  )).

assembly_possible_action(Assemblage, ActionAssemblage, ActionEntity) :-
  findall(X, assembly_specified_parent(Assemblage, X), Finalized),
  list_to_set(Finalized,Finalized_set),
  member(ActionAssemblage,Finalized_set),
  assembly_action_entity(ActionAssemblage, ActionEntity).

assembly_specified_parent(Assemblage, Finalized) :-
  Finalized=Assemblage ; (
  subassemblage(Parent, Assemblage),
  assemblage_specified(Parent),
  assembly_specified_parent(Parent, Finalized)).
  
assembly_action_entity(Assemblage, ActionEntity) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  assembly_primary_part(Assemblage, MobilePart, SecondaryParts),
  owl_instance_from_class(knowrob_assembly:'ConnectingParts',ActionEntity),
  rdf_assert(ActionEntity, knowrob_assembly:assembledConnection, Connection),
  rdf_assert(ActionEntity, knowrob_assembly:mobilePart, MobilePart),
  rdf_assert(ActionEntity, knowrob:objectOfStateChange, Assemblage),
  forall(member(Fixed,SecondaryParts),
         rdf_assert(ActionEntity, knowrob_assembly:fixedPart, Fixed)).
  
%% assembly_primary_part(+Assemblage, -PrimaryPart, -SecondaryParts) is det.
%
% Find the primary part of an assembly action, i.e., the part that needs to be moved into some other parts.
% Prefer part with minimum number of blocked affordances, and parts not attached to a fixture.
% Furthermore require parts with non blocked grasping affordances.
% Will fail if none of the parts is graspable or connected to a graspable object.
%
assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts) :-
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
  once(assemblage_part_links_fixture(Part, _, _)), !.
part_attached_to_fixture(_Part, 0).

part_blocked_affordances(Part, Blocked) :-
  findall(Aff, assemblage_part_blocked_affordance(Part,Aff), Blocked).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

assembly_action_put_away(
    PlanningEntity,
    item(detach,_Occluded,'http://knowrob.org/kb/knowrob_assembly.owl#partOccludedBy',Occluder,_Cause),
    _Strategy,
    ActionEntity) :-
  owl_instance_from_class(knowrob_assembly:'PutAwayPart',ActionEntity),
  rdf_assert(ActionEntity, knowrob:movedObject, Occluder),
  forall((
    rdf_has(PlanningEntity,knowrob:objectActedOn,Obj),
    rdfs_individual_of(Obj,knowrob:'HumanScaleObject')),(
    rdf_assert(ActionEntity, knowrob:avoidedObject, Obj)
  )).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
