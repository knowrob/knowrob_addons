
:- module(cram_assembly,
    [
      cram_assembly_apply_connection/2,
      cram_assembly_apply_grasp/3,
      cram_assembly_apply_ungrasp/3
    ]).

:- register_ros_package(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/transforms')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/knowrob_assembly.owl').

:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

:- rdf_meta cram_assembly_apply_connection(r,r),
            cram_assembly_apply_grasp(r,r,r),
            cram_assembly_apply_ungrasp(r,r,r).

% TODO(DB): grasping only breaks connections to fixtures.
%           can we express somehow when this happens for non permanent connections?

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
  belief_republish_objects([PrimaryObject|Parents]).

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
  belief_republish_objects(Dirty),
  % assert temporary connections that consume affordances blocked by the grasp
  cram_assembly_block_grasp_affordances(GraspedObject, GraspSpec).

cram_assembly_gripper(Gripper) :-
  % just require that the object has a TF name for now
  rdf_has(Gripper, knowrob:'frameName', literal(_)).
  
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
    rdf_has(GraspedObject, knowrob:'frameName', literal(ObjFrame)),
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
