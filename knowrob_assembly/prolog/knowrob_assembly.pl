/** <module> Predicates for managing assemblies

  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Daniel Beßler
@license BSD
*/
:- module(knowrob_assembly,
    [
        assemblage_specified/1,
        assemblage_fixed_part/1,
        assemblage_mechanical_part/1,
        assemblage_underspecified/1,
        assemblage_established/1,
        assemblage_connection/1,
        assemblage_connection_create/3,
        assemblage_connection_established/1,
        assemblage_connection_part/2,
        assemblage_connection_reference/3,
        assemblage_connection_transform/3,
        assemblage_destroy/1,
        assemblage_graspable_part/2,
        assemblage_possible_grasp/3,
        assemblage_possible_grasp/2,
        assemblage_part/2,
        assemblage_part_blocked_affordance/2,
        assemblage_part_links_fixture/3,
        assemblage_part_links_part/2,
        assemblage_part_connect_transforms/2,
        assemblage_part_make_reference/2,
        assemblage_remove_fixtures/1,
        subassemblage/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_planning')).
:- use_module(library('knowrob_beliefstate')).
:- use_module(library('knowrob_math')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

:-  rdf_meta
      assemblage_specified(r),
      assemblage_underspecified(r),
      assemblage_established(r),
      assemblage_destroy(r),
      assemblage_mechanical_part(r),
      assemblage_fixed_part(r),
      assemblage_graspable_part(r,r),
      assemblage_possible_grasp(r,r,t),
      assemblage_possible_grasp(r,t),
      assemblage_connection_create(r,t,-),
      assemblage_connection_established(r),
      assemblage_connection_transform(r,r,t),
      assemblage_connection_reference(r,r,r),
      assemblage_connection_part(r,r),
      assemblage_part(r,r),
      assemblage_part_blocked_affordance(r,r),
      assemblage_part_links_fixture(r,r,r),
      assemblage_part_links_part(r,t),
      assemblage_part_connect_transforms(r,-),
      assemblage_part_make_reference(r,t),
      assemblage_remove_fixtures(r),
      subassemblage(r,r).


%% assemblage_fixed_part(+FixedPart) is det.
%
assemblage_fixed_part(FixedPart) :-
  rdfs_individual_of(FixedPart, knowrob_assembly:'FixedPart').

%% assemblage_mechanical_part(+BasicMechanicalPart) is det.
%
assemblage_mechanical_part(BasicMechanicalPart) :-
  rdfs_individual_of(BasicMechanicalPart, knowrob_assembly:'BasicMechanicalPart').

%% assemblage_connection(+Connection) is det.
%
assemblage_connection(Connection) :-
  rdfs_individual_of(Connection, knowrob_assembly:'AssemblyConnection').

%% assemblage_specified(+Assemblage) is det.
%
assemblage_specified(Assemblage) :-
  \+ assemblage_underspecified(Assemblage),
  forall( subassemblage(Assemblage, SubAssemblage),
          assemblage_specified(SubAssemblage) ).

%% assemblage_underspecified(+Assemblage) is det.
%
assemblage_underspecified(Assemblage) :-
  owl_unsatisfied_restriction(Assemblage, Restr),
  assemblage_relevant_restriction(Restr), !.
assemblage_underspecified(Assemblage) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  owl_unsatisfied_restriction(Connection, Restr),
  assemblage_relevant_restriction(Restr), !.

assemblage_relevant_restriction(Restr) :-
  rdf_has(Restr, owl:'onProperty', P),
  assemblage_relevant_restriction_(P).
assemblage_relevant_restriction(Restr) :-
  rdf_has(Restr, owl:'onProperty', P),
  rdf_has(P, owl:'inverseOf', P_inv),
  assemblage_relevant_restriction_(P_inv).
assemblage_relevant_restriction_(P) :-
  rdf_equal(P, knowrob_assembly:'hasPart') ;
  rdf_equal(P, knowrob_assembly:'usesConnection') ;
  rdf_equal(P, knowrob_assembly:'consumesAffordance') ;
  rdf_equal(P, knowrob_assembly:'linksAssemblage').

%% assemblage_established(?Assemblage) is det.
%
assemblage_established(Assemblage) :-
  rdf_has(Assemblage, knowrob_assembly:'assemblageEstablished', literal(type(_,'true'))).

%% assemblage_connection_established(?Connection) is det.
%
assemblage_connection_established(Connection) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection), 
  rdfs_individual_of(Assemblage, knowrob_assembly:'Assemblage'), !,
  rdf_has(Assemblage, knowrob_assembly:'assemblageEstablished', literal(type(_,'true'))), !.
assemblage_connection_established(_Connection).

%% assemblage_destroy(+Connection) is det.
%% assemblage_destroy(+Assemblage) is det.
%
assemblage_destroy(Connection) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection), !,
  assemblage_destroy(Assemblage).
assemblage_destroy(Assemblage) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection), !,
  rdf_retractall(Assemblage,_,_),
  assemblage_destroy_connection(Connection).
assemblage_destroy(Connection) :-
  assemblage_destroy_connection(Connection).

assemblage_destroy_connection(Connection) :-
  write('[assembly] detach: '), rdf_write_readable(Connection), nl,
  rdf_retractall(_,_,Connection),
  rdf_retractall(Connection,_,_).

%% assemblage_connection_create(+ConnType,+Objects,-ConnId) is det.
%
assemblage_connection_create(ConnType, Objects, ConnId) :-
  rdf_instance_from_class(ConnType, ConnId),
  forall((
    rdfs_subclass_of(ConnType,Restr),
    rdfs_individual_of(Restr,owl:'Restriction'),
    rdf_has(Restr, owl:onProperty, knowrob_assembly:'consumesAffordance'),
    rdf_has(Restr, owl:'onClass', AffType)),once(((
      member(Obj,Objects),
      rdf_has(Obj, knowrob_assembly:'hasAffordance', Affordance),
      rdfs_individual_of(Affordance, AffType),
      rdf_assert(ConnId, knowrob_assembly:'consumesAffordance', Affordance)
    ) ; (
      write('[ERR] Create '), rdf_write_readable(ConnType),
      write(': affordance '), rdf_write_readable(AffType), write(' missing.'), nl,
      rdf_retractall(ConnId,_,_),
      fail
    )))).

%% assemblage_remove_fixtures(+Part) is det.
%
assemblage_remove_fixtures(Part) :-
  forall( assemblage_part_links_fixture(Part,_,Connection),
          assemblage_destroy(Connection) ).

%% subassemblage(+Assemblage, ?ChildAssemblage) is det.
%
% TODO: this shows a weakness of the model, the hierarchy can be interpreted in two directions:
%             A linksAssemblage B always implies B linksAssemblage A.
%        thus I need to check on restrictions here instead of using just linksAssemblage property.
%
subassemblage(Assemblage, ChildAssemblage) :-
  ground(Assemblage), !,
  assemblage_linksAssemblage(Assemblage, ChildAssemblage),
  once((
    assemblage_linksAssemblage_restriction(Assemblage, ChildAssemblageType),
    owl_individual_of(ChildAssemblage, ChildAssemblageType))).
subassemblage(Assemblage, ChildAssemblage) :-
  ground(ChildAssemblage), !,
  assemblage_linksAssemblage(ChildAssemblage, Assemblage),
  once((
    assemblage_linksAssemblage_restriction(Assemblage, ChildAssemblageType),
    owl_individual_of(ChildAssemblage, ChildAssemblageType))).

assemblage_linksAssemblage(Assemblage, Linked) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn1),
  findall(X, (
    rdf_has(Conn1, knowrob_assembly:'consumesAffordance', Aff1),
    rdf_has(Part1, knowrob_assembly:'hasAffordance', Aff1),
    rdf_has(Part1, knowrob_assembly:'hasAffordance', Aff2),
    rdf_has(Conn2, knowrob_assembly:'consumesAffordance', Aff2),
    Conn1 \= Conn2,
    rdf_has(X, knowrob_assembly:'usesConnection', Conn2)
  ), Links),
  list_to_set(Links, Links_set),
  member(Linked, Links_set).

assemblage_linksAssemblage_restriction(Assemblage, ChildAssemblageType) :-
  rdfs_individual_of(Assemblage, Restr),
  rdfs_individual_of(Restr, owl:'Restriction'),
  rdf_has(Restr, owl:'onProperty', knowrob_assembly:'usesConnection'),
  owl_restriction(Restr, restriction(_, cardinality(_,_,Descr))),
  owl_description(Descr, Descr_x),
  ( Descr_x = restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(ChildAssemblageType)) ; (
    Descr_x = intersection_of(List),
    member(restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(ChildAssemblageType)), List)
  )).

%% assemblage_graspable_part(+Assemblage,+Part) is det.
%
assemblage_graspable_part(Assemblage, Part) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn),
  assemblage_part(Assemblage, Part),
  assemblage_possible_grasp(Part, Conn, _).

%% assemblage_possible_grasp(+Assemblage, -(GraspObj,GraspAff,GraspSpec)) is det.
%% assemblage_possible_grasp(+Object, +TargetConnection, -(GraspObj,GraspAff,GraspSpec)) is det.
%
assemblage_possible_grasp(Assemblage, PossibleGrasp) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', TargetConnection),
  assemblage_part(Assemblage, Part),
  assemblage_possible_grasp(Part, TargetConnection, PossibleGrasp).
assemblage_possible_grasp(Object, TargetConnection, (GraspObj,GraspAff,GraspSpec)) :-
  (GraspObj=Object ; assemblage_part_links_part(Object, GraspObj)),
  rdf_has(GraspObj, knowrob_assembly:'hasAffordance', GraspAff),
  rdfs_individual_of(GraspAff, knowrob_assembly:'GraspingAffordance'),
  % ensure grasped affordance not used in (not yet established) target assemblage
  \+ rdf_has(TargetConnection, knowrob_assembly:'consumesAffordance', GraspAff),
  % ensure the affordance is not yet blocked
  \+ assemblage_part_blocked_affordance(GraspObj, GraspAff),
  % find the grasp specification
  owl_has(GraspAff, knowrob_assembly:'graspAt', GraspSpec),
  % TODO: also ensure that grasp does not indirectly block affordances required by TargetConnection
  true.

%% assemblage_part(?Assemblage,?AtomicPart) is det.
%
assemblage_part(Assemblage, AtomicPart) :-
  ground(AtomicPart), !,
  assemblage_connection_part(Conn,AtomicPart),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn).
assemblage_part(Assemblage, AtomicPart) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn),
  assemblage_connection_part(Conn, AtomicPart).

%% assemblage_connection_part(?Connection,?AtomicPart) is det.
%
assemblage_connection_part(Connection, AtomicPart) :-
  ground(AtomicPart), !,
  rdf_has(AtomicPart, knowrob_assembly:'hasAffordance', Affordance),
  rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance).
assemblage_connection_part(Connection, AtomicPart) :-
  rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance),
  rdf_has(AtomicPart, knowrob_assembly:'hasAffordance', Affordance).

%% assemblage_part_blocked_affordance(?Part,?Affordance) is det.
%
assemblage_part_blocked_affordance(Part, Affordance) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', Affordance),
  once((
    rdf_has(Connection, knowrob_assembly:'blocksAffordance', Affordance),
    assemblage_connection_established(Connection)
  )).

%% assemblage_part_links_fixture(+Part, ?Fixture, ?Connection) is det.
%
assemblage_part_links_fixture(Part, Fixture, Connection) :-
  assemblage_mechanical_part(Part),
  assemblage_part_links_connections(Part, Connections),
  findall([X,C], (
    member(C, Connections),
    assemblage_connection_part(C,X),
    assemblage_fixed_part(X)
  ), Fixtures),
  list_to_set(Fixtures, Fixture_set),
  member([Fixture,Connection], Fixture_set).

%% assemblage_part_links_part(+Part, ?LinkedPart) is det.
%
assemblage_part_links_part(Part, LinkedPart) :-
  assemblage_mechanical_part(Part),
  assemblage_part_links_connections(Part, Connections),
  findall([X,C], (
    member(C, Connections),
    assemblage_connection_part(C,X),
    \+ assemblage_fixed_part(X)
  ), Parts_list),
  list_to_set(Parts_list, Parts),
  member([LinkedPart,_], Parts),
  LinkedPart \= Part.

assemblage_part_links_connections(Part, Connections) :-
  assemblage_part_links_connections(Part, [], Connections_list),
  list_to_set(Connections_list, Connections).
assemblage_part_links_connections(Part, Blacklist, LinkedConnections) :-
  % find all connections consuming an affordance of the part
  findall(Direct_connection, (
    assemblage_connection_part(Direct_connection, Part),
    assemblage_connection_established(Direct_connection),
    \+ member(Direct_connection,Blacklist)
  ), LinkedConnections_direct),
  append(Blacklist, LinkedConnections_direct, NewBlacklist),
  % find all other parts used in directly linked connections
  findall(Direct_part, (
    member(Direct_connection,LinkedConnections_direct),
    assemblage_connection_part(Direct_connection, Direct_part),
    Direct_part \= Part
  ), LinkedParts_list),
  list_to_set(LinkedParts_list, LinkedParts),
  % recursively find linked assemblages for linked parts
  findall(LinkedConnections_sibling, (
    member(Direct_part,LinkedParts),
    \+ assemblage_fixed_part(Direct_part), % don't follow fixtures
    assemblage_part_links_connections(Direct_part,NewBlacklist,LinkedConnections_sibling)
  ), LinkedConnections_indirect),
  flatten([LinkedConnections_direct|LinkedConnections_indirect], LinkedConnections).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Part/Connection transforms 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% assemblage_connection_transform(+Connection,+PrimaryObject,-Transform) is det.
%
assemblage_connection_transform(Connection, PrimaryObject, [TargetFrame,RefFrame,Translation,Rotation]) :-
  assemblage_connection_reference(Connection, TransformId, ReferenceObj),
  rdf_has(PrimaryObject, srdl2comp:'urdfName', literal(TargetFrame)),
  rdf_has(ReferenceObj , srdl2comp:'urdfName', literal(RefFrame)),
  once(owl_has(Connection, knowrob_assembly:'usesTransform', TransformId)),
  transform_data(TransformId, (Translation, Rotation)).

%% assemblage_connection_reference_object(+Connection,+TransformId,-ReferenceObj) is det.
%
assemblage_connection_reference(_Connection, TransformId, ReferenceObj) :-
  rdf_has(TransformId, knowrob:'relativeTo', ReferenceObj), !.
assemblage_connection_reference(Connection, TransformId, ReferenceObj) :-
  % FIXME: won't work when multiple instances of the reference object class are linked in the connection
  rdfs_individual_of(TransformId, Restr),
  rdfs_individual_of(Restr, owl:'Restriction'),
  rdf_has(Restr, owl:'onProperty', knowrob:'relativeTo'),
  rdf_has(Restr, owl:'onClass', ReferenceCls),
  rdf_has(Connection, knowrob_assembly:'consumesAffordance', Aff),
  rdf_has(ReferenceObj, knowrob_assembly:'hasAffordance', Aff),
  owl_individual_of(ReferenceObj,ReferenceCls), !.

%% assemblage_part_make_reference(+RefObj,-OldParents) is det.
%
assemblage_part_make_reference(RefObj, OldParents) :-
  assemblage_transform_parents(RefObj, ChildParentTuples, []),
  findall( Parent, (
           member((Child,Parent), ChildParentTuples),
           belief_at_invert_topology(Child,Parent) ), OldParents).

assemblage_transform_parents(Child, [(Child,Parent)|Rest], Blacklist) :-
  \+ member(Child, Blacklist),
  rdf_has(Child, paramserver:'hasTransform', TransformId),
  rdf_has(TransformId, knowrob:'relativeTo', Parent),
  assemblage_mechanical_part(Parent), !,
  assemblage_transform_parents(Parent, Rest, [Child|Blacklist]).
assemblage_transform_parents(_Child, [], _Blacklist).

%% assemblage_part_connect_transforms(+RefObj,-DirtyObjects) is det.
%
assemblage_part_connect_transforms(RefObj, [Bridge|BridgeParents]) :-
  assemblage_global_reference_object(RefObj, RefObj_global),
  assemblage_part_links_part(RefObj, Bridge),
  \+ assemblage_has_parent_frame(Bridge, RefObj_global),
  once((
    assemblage_connection_part(Conn,Bridge),
    assemblage_connection_part(Conn,Other), Bridge \= Other,
    assemblage_has_parent_frame(Other, RefObj)
  )),
  belief_at_global(Bridge, BridgePoseMap),
  belief_at_global(Other, OtherPoseMap),
  transform_compute_relative(BridgePoseMap, OtherPoseMap, [_,_,BridgeT,BridgeR]),
  assemblage_part_make_reference(Bridge, BridgeParents),
  belief_at_internal(Bridge, (BridgeT,BridgeR), Other).

assemblage_global_reference_object(Child, Parent) :-
  rdf_has(Child, paramserver:'hasTransform', TransformId),
  (( rdf_has(TransformId, knowrob:'relativeTo', X),
     \+ assemblage_fixed_part(X) )
  -> assemblage_global_reference_object(X,Parent)
  ;  Parent = Child ).

assemblage_has_parent_frame(Parent, Parent) :- !.
assemblage_has_parent_frame(Child, Parent) :-
  rdf_has(Child, paramserver:'hasTransform', TransformId),
  rdf_has(TransformId, knowrob:'relativeTo', X),
  assemblage_has_parent_frame(X, Parent).
