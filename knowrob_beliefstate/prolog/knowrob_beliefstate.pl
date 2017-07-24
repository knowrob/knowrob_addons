/** <module> knowrob_beliefstate

  Copyright (C) 2017 Mihai Pomarlan

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

  @author Mihai Pomarlan
  @license BSD
*/

:- module(knowrob_beliefstate,
    [
      mark_dirty_objects/1,
      print_debug_string/1,
      get_known_object_ids/1,
      get_known_assemblage_ids/1,
      get_object_color/2,
      get_object_mesh_path/2,
      get_object_transform/2,
      get_object_at_location/5,
      get_new_object_id/2,
      get_all_possible_grasps_on_object/2,
      get_all_possible_grasps_on_object/3,
      get_currently_possible_grasps_on_object/2,
      get_currently_possible_grasps_on_object/3,
      get_current_objects_in_gripper/2,
      get_current_grasps_on_object/2,
      get_objects_in_grasp/2,
      get_assemblages_with_object/2,
      get_objects_in_assemblage/2,
      get_objects_connected_to_object/2,
      get_pre_grasp_position/4,
      get_grasp_position/4,
      get_post_grasp_position/4,
      get_connection_transform/4,
      get_possible_grasps_on_object/2,
      get_possible_grasps_on_object/3,

      get_object_reference_frame/2,

      assert_object_at_location/3,
      assert_grasp_on_object/5,
      assert_ungrasp/1,
      assert_ungrasp/2,
      assert_assemblage_created/6,
      ensure_assemblage_transforms/5,
      assert_assemblage_destroyed/1,

      create_assembly_agenda/3,
      reset_beliefstate/0
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('util')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_paramserver')).
:- use_module(library('tf_prolog')).
:- use_module(library('random')).

:- load_foreign_library('libkr_beliefstate.so').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(beliefstate, 'http://knowrob.org/kb/knowrob_beliefstate.owl#',  [keep(true)]).

quote_id(X, O) :-
  atom_string(X, Oxx),
  string_concat('"', Oxx, Ox),
  string_concat(Ox, '"', O).

% TODO: send a ros service call to the ROS object state publisher node. Parameter is a list of object ids to mark as dirty.
%% mark_dirty_objects([]).

mark_dirty_objects(Objs) :-
  %% \+ =(Objs, []),
%  service_call_mark_dirty_objects(Objs),
  findall(O, (member(X, Objs), quote_id(X, O)), Os),
  atomic_list_concat(Os, ',', OsS),
  atom_string("'[", LB),
  string_concat(LB, OsS, PS),
  atom_string("]'", RB),
  string_concat(PS, RB, ParStr),
  atom_string("rosservice call /object_state_publisher/mark_dirty_object ", CmdKern),
  string_concat(CmdKern, ParStr, Cmd),
  thread_create(shell(Cmd), _, []).

print_debug_string(Str) :-
  call_ros_info(Str).

temporal_extent_active(TemporalObject) :-
  % A TemporalObject is active when it has no temporalExtent (ie. it is forever) ...
  \+ owl_has(TemporalObject, assembly:'temporalExtent', _).

temporal_extent_active(TemporalObject) :-
 % ... or when it has a temporalExtent to an Interval without endsAtTime.
 owl_has(TemporalObject, assembly:'temporalExtent', TempExt),
 \+ owl_has(TempExt, assembly:'endsAtTime', _).

create_object_with_temporal_extent(Type, Object) :-
  get_new_object_id(Type, Object),
  atom_string(ObjectAt, Object),
  rdf_assert(ObjectAt, rdf:type, Type),
  get_new_object_id('http://knowrob.org/kb/knowrob_assembly.owl#Interval', TempExt),
  atom_string(TempExtAt, TempExt),
  rdf_assert(TempExtAt, rdf:type, assembly:'Interval'),
  get_timepoint(T),
  rdf_assert(T, rdf:type, assembly:'TimePoint'),
  rdf_assert(T, paramserver:'hasValue', literal(type(xsd:'string', T))),
  rdf_assert(TempExtAt, assembly:'startsAtTime', T),
  rdf_assert(ObjectAt, assembly:'temporalExtent', TempExtAt),
  !.

get_gripper_tool_frame(Gr, ToolFrame) :-
  rdf_has(paramserver:'GripperToolFrameSymbol', paramserver:'standsFor', FNO),
  rdf_has(FNO, paramserver:validForGripperType, literal(type(xsd:'anyURI', Gr))),
  rdf_has(FNO, paramserver:hasValue, literal(type(xsd:'string', ToolFrame))).

get_object_reference_frame(OId, FN) :-
  rdf_has(paramserver:'ObjectReferenceFrameSymbol', paramserver:'standsFor', FNO),
  rdf_has(FNO, paramserver:validForObjectType, literal(type(xsd:'anyURI', OId))),
  rdf_has(FNO, paramserver:hasValue, literal(type(xsd:'string', FN))).

%% get_known_object_ids(-ObjectIds) is det.
%
% Returns a list of strings representing the individuals of type MechanicalPart that are known to the KB.
%
% @param ObjectIds    [anyURI*], the object ids
%
get_known_object_ids(ObjectIds) :-
  findall(J, rdfs_individual_of(J, 'http://knowrob.org/kb/knowrob_assembly.owl#AtomicPart'), ObjectIds).

get_active_assemblage(A) :- 
  rdfs_individual_of(A, assembly:'Assemblage'),
  rdf_has(A, assembly:'usesConnection', C),
  temporal_extent_active(C).

%% get_known_assemblage_ids(-AssemblageIds) is det.
%
% Returns a list of strings representing the individuals of type Assemblage that are known to the KB.
% Only active assemblages are returned: their Connection has no temporalExtent, or a temporalExtent to an Interval with no endsAtTime.
%
% @param AssemblageIds    [anyURI*], the assemblage ids
%
get_known_assemblage_ids(AssemblageIds) :-
  findall(J, get_active_assemblage(J), AssemblageIds).

%% get_object_color(+ObjectId, -Color) is det.
%
% Returns a color for ObjectId. The color is returned as
% [float red, green, blue, alpha], on a scale of 0-1.
% If there is no color given for an object in the knowledge base, then a default [0.5, 0.5, 0.5, 1]
% is returned.
%
% @param ObjectId    anyURI, the object id
% @param Color   the transform data
%
get_object_color(ObjectId, Color) :-
  rdf_has(ObjectId, paramserver:'hasColor', ColInd),
  rdf_has(ColInd, 'http://knowrob.org/kb/knowrob.owl#vector', literal(type(_, V))), 
  knowrob_math:parse_vector(V, Color), !.

get_object_color(ObjectId, Color) :-
  \+ rdf_has(ObjectId, paramserver:'hasColor', _),
  Color = [0.5, 0.5, 0.5, 1.0], !.

%% get_object_color(ObjectId, Color) :-
%%   rdf_has(ObjectId, paramserver:'hasColor', ColInd),
%%   rdf_has(ColInd, paramserver:'hasRed', RedInd),
%%   rdf_has(ColInd, paramserver:'hasGreen', GreenInd),
%%   rdf_has(ColInd, paramserver:'hasBlue', BlueInd),
%%   rdf_has(ColInd, paramserver:'hasAlpha', AlphaInd),
%%   rdf_has(RedInd, paramserver:'hasValue', literal(type(_, Red))),
%%   rdf_has(GreenInd, paramserver:'hasValue', literal(type(_, Green))),
%%   rdf_has(BlueInd, paramserver:'hasValue', literal(type(_, Blue))),
%%   rdf_has(AlphaInd, paramserver:'hasValue', literal(type(_, Alpha))),
%%   =(Color, [Red, Green, Blue, Alpha]).

%% get_object_mesh_path(+ObjectId, -FilePath) is det.
%
% Returns a path to a mesh file (stl or dae) for the ObjectId.
%
% @param ObjectId    anyURI, the object id
% @param FilePath    anyURI, the path (usually a package:// path)
%
get_object_mesh_path(ObjectId, FilePath) :-
  rdf_has(ObjectId, paramserver:'hasShape', ShapeCan),
  rdfs_individual_of(ShapeCan, paramserver:'TexturedShape'),
  rdf_has(ShapeCan, paramserver:'hasPath', literal(type(xsd:'anyURI', FP))),
  atom_string(FP, FilePath).  

%% get_object_transform(+ObjectId, -Transform) is det.
%
% Returns the currently active transform of ObjectId. The transform is returned as
% [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
% Transforms for an object are attached to a TemporalReification object (a TemporaryTransform), which also
% has an Interval attached to it. An active transform is one whose TemporalReification
% has an Interval without an end point. It should be the case that objects have 1 and only
% 1 active transform at any time.
%
% Note: an object may have several active transforms at a time, which may be referenced to either a TemporaryGrasp
% or an AssemblyConnection. And object can have only one active transform referenced to MapFrameSymbol, and if
% an object has such an active transform, it is the only one.
% 
% When there are several active transforms, the ones referenced to a TemporaryGrasp have precedence over those 
% referenced to an AssemblyConnection.
%
% Warning: if backtracking/findall on this predicate, transforms relative to all grippers, as well as transforms relative
% to other parts will be returned. Higher levels should beware of this-- if transforms relative to grippers are returned,
% it is better to use those as they indicate how the object may be moved.
%
% @param ObjectId    anyURI, the object id
% @param Transform   the transform data
%
get_object_transform(ObjectId, Transform) :-
  rdf_has(ObjectId, paramserver:'hasTransform', TempRei),
  temporal_extent_active(TempRei),
  rdf_has(TempRei, assembly:'hasReferencePart', Ref),
  get_object_transform(ObjectId, TempRei, Ref, Transform).

% The several branches here are meant to 'sort' how results are returned: grasp-referenced transforms first, connections next.
get_object_transform(ObjectId, TempRei, Ref, Transform) :-
  owl_individual_of(Ref, assembly:'TemporaryGrasp'),
  get_associated_transform(_, ObjectId, _, TempRei, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform).

get_object_transform(ObjectId, TempRei, Ref, Transform) :-
  owl_individual_of(Ref, assembly:'AssemblyConnection'),
  get_associated_transform(_, ObjectId, _, TempRei, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform).

get_object_transform(ObjectId, TempRei, Ref, Transform) :-
  owl_same_as(Ref, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol'),
  get_associated_transform(_, ObjectId, _, TempRei, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform).


translations_are_close(T1, T2, Dmax) :-
  nth0(0, T1, X1),
  nth0(1, T1, Y1),
  nth0(2, T1, Z1),
  nth0(0, T2, X2),
  nth0(1, T2, Y2),
  nth0(2, T2, Z2),
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

rotations_are_close(R1, R2, Dmax) :-
  % First compare the two quaternions as they are ...
  nth0(0, R1, X1),
  nth0(1, R1, Y1),
  nth0(2, R1, Z1),
  nth0(3, R1, W1),
  nth0(0, R2, X2),
  nth0(1, R2, Y2),
  nth0(2, R2, Z2),
  nth0(3, R2, W2),
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dw is W1 - W2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz + Dw*Dw,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

rotations_are_close(R1, R2, Dmax) :-
  % ... if that fails, compare R1 with -R2
  nth0(0, R1, X1),
  nth0(1, R1, Y1),
  nth0(2, R1, Z1),
  nth0(3, R1, W1),
  nth0(0, R2, X2),
  nth0(1, R2, Y2),
  nth0(2, R2, Z2),
  nth0(3, R2, W2),
  Dx is X1 + X2,
  Dy is Y1 + Y2,
  Dz is Z1 + Z2,
  Dw is W1 + W2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz + Dw*Dw,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

copy_sign(_, Sign, Val) :-
  SignSq is Sign*Sign,
  compare(<, SignSq, 0.00001),
  =(Val, 0).

copy_sign(Mag, Sign, Val) :-
  compare(<, Sign, 0),
  Val is -1*Mag.

copy_sign(Mag, Sign, Val) :-
  compare(<, 0, Sign),
  =(Val, Mag).

matrix_to_quaternion(M00, M01, M02, M10, M11, M12, M20, M21, M22, QX, QY, QZ, QW) :-
  Wu is sqrt(max(0, 1 + M00 + M11 + M22))/2,
  Xu is sqrt(max(0, 1 + M00 - M11 - M22))/2,
  Yu is sqrt(max(0, 1 - M00 + M11 - M22))/2,
  Zu is sqrt(max(0, 1 - M00 - M11 + M22))/2,
  Xs is M21 - M12,
  Ys is M02 - M20,
  Zs is M10 - M01,
  copy_sign(Xu, Xs, QX),
  copy_sign(Yu, Ys, QY),
  copy_sign(Zu, Zs, QZ),
  =(Wu, QW).

get_tf_transform(ReferenceFrame, TargetFrame, TFTransform) :-
  get_current_tf(ReferenceFrame, TargetFrame, Tx, Ty, Tz, Rx, Ry, Rz, Rw),
  =(TFTransform, [ReferenceFrame, TargetFrame, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]).
  

%% get_object_at_location(+ObjectType, +Transform, +TranThreshold, +RotThreshold, -ObjectId) is det.
%
% Checks whether one of the already known objects of ObjectType is located close to Transform.
% The check uses TranThreshold as a threshold for comparing translation distance, and similarly for
% rotation and RotThreshold.
%
% Transform is expected to be of the form [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]]
%
% If such an object is found, its id is returned via ObjectId.
%
% @param ObjectType      anyURI, the object type
% @param Transform       the transform data
% @param TranThreshold   double, a distance below which two translations are thought to be the same
% @param RotThreshold    double, a distance below which two rotations are thought to be the same;
%                        NOTE: this threshold is interpreted as a euclidean distance threshold in a space of quaternions where +Quat and -Quat are the same.
% @param ObjectId        anyURI, the object id
%
get_object_at_location(ObjectType, Transform, TranThreshold, RotThreshold, ObjectId) :-
  rdfs_individual_of(Object, ObjectType),
  get_object_reference_frame(Object, TargetFrame),
  nth0(1, ObjTransform, TargetFrame),
  nth0(0, Transform, ReferenceFrame),
  nth0(2, Transform, ArgTranslation),
  nth0(3, Transform, ArgRotation),
  get_tf_transform(ReferenceFrame, TargetFrame, TFTransform),
  nth0(2, TFTransform, ObjTranslation),
  nth0(3, TFTransform, ObjRotation),
  translations_are_close(ArgTranslation, ObjTranslation, TranThreshold),
  rotations_are_close(ArgRotation, ObjRotation, RotThreshold),
  =(ObjectId, Object).

%% get_new_object_id(+ObjectType, -ObjectId) is det.
%
% Generates a random string of the form <ObjectType>_{8*random selection of Alphanumeric characters}
%
% @param ObjectType      anyURI, the object type
% @param ObjectId        anyURI, the object id
%
get_new_object_id(ObjectType, ObjectId) :-
  atom_string(ObjectType, ObjectTypeStr),
  =(LCChs, ['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z']),
  =(UCChs, ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']),
  =(Digs, ['0','1','2','3','4','5','6','7','8','9']),
  append(LCChs, UCChs, Alphabet),
  append(Alphabet, Digs, Alphanumerics),
  random_select(C1, Alphanumerics, _),
  random_select(C2, Alphanumerics, _),
  random_select(C3, Alphanumerics, _),
  random_select(C4, Alphanumerics, _),
  random_select(C5, Alphanumerics, _),
  random_select(C6, Alphanumerics, _),
  random_select(C7, Alphanumerics, _),
  random_select(C8, Alphanumerics, _),
  string_concat(ObjectTypeStr, '_', ObjectId0),
  string_concat(ObjectId0, C1, ObjectId1),
  string_concat(ObjectId1, C2, ObjectId2),
  string_concat(ObjectId2, C3, ObjectId3),
  string_concat(ObjectId3, C4, ObjectId4),
  string_concat(ObjectId4, C5, ObjectId5),
  string_concat(ObjectId5, C6, ObjectId6),
  string_concat(ObjectId6, C7, ObjectId7),
  string_concat(ObjectId7, C8, ObjectId).

assemblage_has_part(Assemblage, Part) :-
  get_active_assemblage(Assemblage),
  owl_has(Assemblage, assembly:'hasPart', Part).

assemblage_has_part(Assemblage, Part) :-
  get_active_assemblage(Assemblage),
  owl_has(Assemblage, assembly:'hasSubassemblage', SubA),
  assemblage_has_part(SubA, Part).

%% get_assemblages_with_object(+Object, -Assemblages) is det.
%
% Returns a list of Assemblage Ids such that every Assemblage in the list hasPart Object
% Only active assemblages are returned: their Connection has no temporalExtent, or a temporalExtent to an Interval with no endsAtTime.
%
% @param Object      anyURI, the object id
% @param Assemblages [anyURI*], assemblages id
%
get_assemblages_with_object(Object, Assemblages) :-
  findall(A, assemblage_has_part(A, Object), AssemblagesL),
  list_to_set(AssemblagesL, Assemblages).

%% get_objects_in_assemblage(+Assemblage, -Objects) is det.
%
% Returns a list of Object Ids such that for every Object in the list we have Assemblage hasPart Object or Assemblage hasSubassemblage Assemblage2 hasPart Object,
% if Assemblage is actually of the Assemblage type. If it is instead an AtomicPart, returns a list containing only Assemblage.
% Only active assemblages are considered: their Connection has no temporalExtent, or a temporalExtent to an Interval with no endsAtTime.
%
% @param Assemblage anyURI, the assemblage id
% @param Objects    [anyURI*], object ids
%
get_objects_in_assemblage(Assemblage, Objects) :-
  owl_individual_of(Assemblage, assembly:'AtomicPart'),
  =(Objects, [Assemblage]),
  !.

get_objects_in_assemblage(Assemblage, Objects) :-
  \+ owl_individual_of(Assemblage, assembly:'AtomicPart'),
  findall(P, assemblage_has_part(Assemblage, P), DupList),
  list_to_set(DupList, Objects).

get_mobile_objects_in_assemblage(Assemblage, MobileParts) :-
  get_objects_in_assemblage(Assemblage, Parts),
  findall(X, (member(X, Parts), is_mobile_object(X)), MobilePartsL),
  list_to_set(MobilePartsL, MobileParts).

get_active_grasp_description(Object, Grasp) :-
  rdf_has(Object, assembly:'isGrasped', TempRei),
  temporal_extent_active(TempRei),
  rdf_has(TempRei, paramserver:'hasGripperType', literal(type(xsd:'anyURI', G))),
  rdf_has(TempRei, paramserver:'hasObjectType', literal(type(xsd:'anyURI', O))),
  rdf_has(TempRei, paramserver:'hasRobotType', literal(type(xsd:'anyURI', R))),
  rdf_has(TempRei, assembly:'hasSpecification', GraspSpec),
  get_associated_transform(G, O, R, GraspSpec, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform),
  % TODO: resolve whether the Robot "Type" is actually the ID; currently assuming it is.
  =(Grasp, [G, O, R, Transform, TempRei]).

get_object_in_gripper(G, O) :-
  owl_individual_of(O, assembly:'AtomicPart'),
  rdf_has(O, assembly:'isGrasped', TempRei),
  temporal_extent_active(TempRei),
  rdf_has(TempRei, paramserver:'hasGripperType', literal(type(xsd:'anyURI', G))).

%% get_current_grasps_on_object(+Object, -Grasps) is det.
%
% Returns a list of Grasp descriptions for every currently active grasp on Object.
% A grasp is active if its corresponding TemporaryGrasp object has as temporalExtention an Interval with no endsAtTime.
% A Grasp description is of form
% [string Gripper, string Object, string Robot, [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]], GraspId]
%
% @param Object      anyURI, the object id
% @param Grasps      [GraspDesc*], grasp descriptions
%
get_current_grasps_on_object(Object, Grasps) :-
  findall(G, get_active_grasp_description(Object, G), Grasps).

%% get_objects_in_grasp(+Grasp, -Objects) is det.
%
% Returns a list of ObjectIds for a particular active Grasp id.
% If the grasp is not active, then returns [].
%
% @param Grasp      anyURI, the TemporaryGrasp id
% @param Objects    [anyURI*], object ids
%
get_objects_in_grasp(Grasp, Objects) :-
  \+ temporal_extent_active(Grasp),
  =(Objects, []).

get_objects_in_grasp(Grasp, Objects) :-
  temporal_extent_active(Grasp),
  findall(O, rdf_has(O, 'http://knowrob.org/kb/knowrob_assembly.owl#isGrasped', Grasp), Objects).

%% get_current_objects_in_gripper(+Gripper, -Objects) is det.
%
% Returns a list of Object Ids for the Gripper, such that every Object is actively grasped right now.
% A grasp is active when its TemporaryGrasp object has temporalExtention an Interval with no endsAtTime.
% NOTE: only objects directly in the gripper are returned. Any objets they may be connected to via assemblages are not.
%
% @param Gripper      anyURI, the gripper id
% @param Objects      [anyURI*], grasp descriptions
%
get_current_objects_in_gripper(Gripper, Objects) :-
  findall(O, get_object_in_gripper(Gripper, O), Objects).

affordance_type_available(AvailableAffordances, NeededType) :-
  member(X, AvailableAffordances),
  owl_individual_of(X, NeededType).

affordance_type_missing(AvailableAffordances, NeededTypes) :-
  member(NeededType, NeededTypes),
  \+ affordance_type_available(AvailableAffordances, NeededType).

all_needed_affordances_available(AvailableAffordances, NeededTypes) :-
  \+ affordance_type_missing(AvailableAffordances, NeededTypes).

affordance_actively_blocked(A) :-
  owl_has(Blocker, assembly:'blocksAffordance', A),
  temporal_extent_active(Blocker).
  
get_affordance(Object, Affordance, Type) :-
  owl_has(Object, assembly:'hasAffordance', Affordance),
  owl_individual_of(Affordance, Type).
  
get_affordance(Object, Affordance) :-
  get_affordance(Object, Affordance, 'http://knowrob.org/kb/knowrob_assembly.owl#Affordance').

get_free_affordance(Object, Affordance, Type) :-
  get_affordance(Object, Affordance, Type),
  \+ affordance_actively_blocked(Affordance).

get_free_affordance(Object, Affordance) :-
  get_free_affordance(Object, Affordance, 'http://knowrob.org/kb/knowrob_assembly.owl#Affordance').

get_valid_grasp_for_object(Object, Gripper, AvailableAffordances, GS) :-
  rdfs_individual_of(GS, paramserver:'GraspSpecification'),
  rdf_has(GS, paramserver:'validForGripperType', literal(type(xsd:'anyURI', Gripper))),
  rdf_has(GS, paramserver:'validForObjectType', literal(type(xsd:'anyURI', OT))),
  findall(Aff, rdf_has(GS, assembly:'needsAffordanceType', literal(type(_, Aff))), NeededAffordanceTypes),
  all_needed_affordances_available(AvailableAffordances, NeededAffordanceTypes),
  owl_individual_of(Object, OT).

%% get_all_possible_grasps_on_object(+Object, -Grasps) is det.
%
% Returns a list of GraspSpecification Ids for Object.
%
% @param Object anyURI, the object id
% @param Grasps [anyURI*], GraspSpecification ids
%
get_all_possible_grasps_on_object(Object, Grasps) :-
  findall(A, get_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, _, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

%% get_all_possible_grasps_on_object(+Object, +Gripper, -Grasps) is det.
%
% Returns a list of GraspSpecification Ids for Object, when grasped with Gripper.
%
% @param Object  anyURI, the object id
% @param Gripper anyURI, the gripper id
% @param Grasps  [anyURI*], GraspSpecification ids
%
get_all_possible_grasps_on_object(Object, Gripper, Grasps) :-
  findall(A, get_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, Gripper, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

%% get_currently_possible_grasps_on_object(+Object, -Grasps) is det.
%
% Returns a list of GraspSpecification Ids for Object.
% A GraspSpecification is returned if all the affordances it needs are not blocked
% by an active grasp or connection.
%
% @param Object  anyURI, the object id
% @param Grasps  [anyURI*], grasps descriptions
%
get_currently_possible_grasps_on_object(Object, Grasps) :-
  findall(A, get_free_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, _, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

%% get_currently_possible_grasps_on_object(+Object, +Gripper, -Grasps) is det.
%
% Returns a list of GraspSpecification Ids for Object.
% A GraspSpecification is returned if all the affordances it needs are not blocked
% by an active grasp or connection.
%
% @param Object  anyURI, the object id
% @param Gripper anyURI, the gripper id
% @param Grasps  [anyURI*], grasps descriptions
%
get_currently_possible_grasps_on_object(Object, Gripper, Grasps) :-
  findall(A, get_free_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, Gripper, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

%% get_currently_possible_grasps_on_object(+Object, -Grasps) is det.
%
% Returns a list of GraspSpecification Ids for Object.
% A GraspSpecification is returned if all the affordances it needs are not blocked
% by an active grasp or connection.
%
% @param Object  anyURI, the object id
% @param Grasps  [anyURI*], grasps descriptions
%
get_possible_grasps_on_object(Object, Grasps) :-
  findall(A, get_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, _, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

get_possible_grasps_on_object(Object, Gripper, Grasps) :-
  findall(A, get_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  findall(G, get_valid_grasp_for_object(Object, Gripper, GraspAffordances, G), GraspsList),
  list_to_set(GraspsList, Grasps).

%% WARNING: the following functions assert entities and relations in the knowledge base.
%% As a result, one must be careful to avoid asserting things multiple times because of
%% Prologs backtracking mechanisms. The cut operator is used to this purpose.

deactivate_temporal_extension(TemporalObject) :-
  % A TemporalObject with a temporalExtent that has an endsAtTime is already inactive,
  owl_has(TemporalObject, assembly:'temporalExtent', TempExt),
  owl_has(TempExt, assembly:'endsAtTime', _),
  % so there is nothing else left to do to it. Avoid potentially dangerous backtracking via the cut operator.
  !.

deactivate_temporal_extension(TemporalObject) :-
  owl_has(TemporalObject, assembly:'temporalExtent', TempExt),
  \+ owl_has(TempExt, assembly:'endsAtTime', _),
  atom_string(TempExtAt, TempExt),
  get_timepoint(T),
  rdf_assert(T, rdf:type, assembly:'TimePoint'),
  rdf_assert(T, paramserver:'hasValue', literal(type(xsd:'string', T))),
  rdf_assert(TempExtAt, assembly:'endsAtTime', T),
  % Once we put something in the endsAtTime of the temporal extent, it is inactive.
  % Avoid potentially dangerous backtracking via the cut operator.
  !.

deactivate_temporal_extension(TemporalObject) :-
  \+ owl_has(TemporalObject, assembly:'temporalExtent', _),
  get_new_object_id('http://knowrob.org/kb/knowrob_assembly.owl#Interval', TempExt),
  atom_string(TempExtAt, TempExt),
  atom_string(TemporalObjectAt, TemporalObject),
  rdf_assert(TempExtAt, rdf:type, assembly:'Interval'),
  rdf_assert(TemporalObjectAt, assembly:'temporalExtent', TempExtAt),
  deactivate_temporal_extension(TemporalObject).
  % No cuts above, because the called version of deactivate_temporal_extension already contains a cut.

deactivate_temporal_extensions([]) :-
  !.

deactivate_temporal_extensions([TO|RestTOs]) :-
  deactivate_temporal_extension(TO),
  deactivate_temporal_extensions(RestTOs),
  !.

create_transform(Transform, TransformIdAt) :-
  rdf_instance_from_class('http://knowrob.org/kb/knowrob_paramserver.owl#Transform', TransformIdAt),

  nth0(0, Transform, ReferenceFrame),
  rdf_instance_from_class('http://knowrob.org/kb/knowrob_paramserver.owl#CoordinateFrameName', ReferenceFrameIdAt),
  rdf_assert(TransformIdAt, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasReferenceFrame', ReferenceFrameIdAt),
  rdf_assert(ReferenceFrameIdAt, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasValue', literal(type(xsd:'string', ReferenceFrame))),

  nth0(1, Transform, TargetFrame),
  rdf_instance_from_class('http://knowrob.org/kb/knowrob_paramserver.owl#CoordinateFrameName', TargetFrameIdAt),
  rdf_assert(TransformIdAt, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTargetFrame', TargetFrameIdAt),
  rdf_assert(TargetFrameIdAt, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasValue', literal(type(xsd:'string', TargetFrame))),
  
  nth0(2, Transform, Translation),
  atomic_list_concat(Translation, ' ', TranslationString),
  rdf_assert(TransformIdAt, 'http://knowrob.org/kb/knowrob.owl#translation', literal(type(xsd:'string', TranslationString))),
  
  nth0(3, Transform, Rotation),
  atomic_list_concat(Rotation, ' ', RotationString),
  rdf_assert(TransformIdAt, 'http://knowrob.org/kb/knowrob.owl#quaternion', literal(type(xsd:'string', RotationString))),
  !.

active_transform(O, T) :-
  rdf_has(O, paramserver:'hasTransform', T),
  temporal_extent_active(T).

active_referenced_transform(O, R, T) :-
  active_transform(O, T),
  rdf_has(T, assembly:'hasReferencePart', R).

active_mapref_transform(Object, MRT) :-
  active_referenced_transform(Object, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol', MRT).

ensure_no_active_mapref_transform(Object) :-
  \+ active_mapref_transform(Object, _),
  !.

ensure_no_active_mapref_transform(Object) :-
  active_mapref_transform(Object, MapRefTr),
  deactivate_temporal_extension(MapRefTr),
  % There will be at most one MapRef transform active at any time.
  !.

add_transform_to_object(Object, TransformData, Reference, TransformId) :-
  ensure_no_active_mapref_transform(Object),
  create_object_with_temporal_extent('http://knowrob.org/kb/knowrob_assembly.owl#TemporaryTransform', TempRei),
  atom_string(TempReiAt, TempRei),
  atom_string(ObjectAt, Object),
  atom_string(ReferenceAt, Reference),
  create_transform(TransformData, TransformId),
  atom_string(TransformIdAt, TransformId),
  rdf_assert(TempReiAt, paramserver:'hasTransform', TransformIdAt),
  rdf_assert(TempReiAt, 'http://knowrob.org/kb/knowrob_assembly.owl#hasReferencePart', ReferenceAt),
  rdf_assert(ObjectAt, paramserver:'hasTransform', TempReiAt),
  !.

add_transform_to_object(Object, TransformData, Reference) :-
  add_transform_to_object(Object, TransformData, Reference, TransformId),
  =(TransformId, _).

get_reference_frame(Ref, RefFrame) :-
  owl_individual_of(Ref, assembly:'TemporaryGrasp'),
  rdf_has(Ref, paramserver:'hasGripperType', literal(type(xsd:'anyURI', Gripper))),
  get_gripper_tool_frame(Gripper, RefFrame).

get_reference_frame(Ref, RefFrame) :-
  owl_individual_of(Ref, assembly:'AssemblyConnection'),
  rdf_has(Ref, paramserver:'hasTransform', T),
  rdf_has(T, paramserver:'hasReferenceFrame', F),
  rdf_has(F, paramserver:'hasValue', literal(type(xsd:'string', RefFrame))).

get_reference_frame(Ref, RefFrame) :-
  owl_same_as(Ref, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol'),
  %% TODO: currently we are using this FrameSymbol directly. In the future we may want to indirect on environment, for example.
  % rdf_has(Ref, paramserver:'standsFor', F),
  % rdf_has(F, paramserver:'hasValue', literal(type(xsd:'string', RefFrame))).
  rdf_has(Ref, paramserver:'hasValue', literal(type(xsd:'string', RefFrame))).

add_transform_to_object_by_reference(Object, Ref, TObj) :-
  get_reference_frame(Ref, RefFrame),
  nth0(0, TObj, OldRefFrame),
  get_tf_transform(RefFrame, OldRefFrame, RefTransform),
  multiply_transforms(RefTransform, TObj, TransformData),
  add_transform_to_object(Object, TransformData, Ref),
  !.

ensure_some_active_transform(Object, [], TransformData) :-
  add_transform_to_object_by_reference(Object, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol', TransformData),
  !.

ensure_some_active_transform(_, [Rem|_], _) :-
  \+ =(Rem, []),
  !.

remove_transform_from_object_by_reference(Object, Ref) :-
  findall(T, active_transform(Object, T), ActTrs),
  findall(RT, active_referenced_transform(Object, Ref, RT), RefActTrs),
  subtract(ActTrs, RefActTrs, RemainingActives),
  % We only need one active transform here
  get_object_transform(Object, TransformData),!,
  deactivate_temporal_extensions(RefActTrs),
  ensure_some_active_transform(Object, RemainingActives, TransformData),
  !.

multiply_transforms(LTr, RTr, UpdTr) :-
  nth0(0, LTr, RefFrame),
  nth0(1, RTr, TgFrame),
  nth0(2, LTr, LTranslation),
  nth0(2, RTr, RTranslation),
  nth0(3, LTr, LRotation),
  nth0(3, RTr, RRotation),
  nth0(0, LTranslation, Lx),
  nth0(1, LTranslation, Ly),
  nth0(2, LTranslation, Lz),
  nth0(0, RTranslation, Rx),
  nth0(1, RTranslation, Ry),
  nth0(2, RTranslation, Rz),
  nth0(0, LRotation, LQx),
  nth0(1, LRotation, LQy),
  nth0(2, LRotation, LQz),
  nth0(3, LRotation, LQw),
  nth0(0, RRotation, RQx),
  nth0(1, RRotation, RQy),
  nth0(2, RRotation, RQz),
  nth0(3, RRotation, RQw),
  NQw is LQw*RQw - LQx*RQx - LQy*RQy - LQz*RQz,
  NQx is LQw*RQx + LQx*RQw + LQy*RQz - LQz*RQy,
  NQy is LQw*RQy - LQx*RQz + LQy*RQw + LQz*RQx,
  NQz is LQw*RQz + LQx*RQy - LQy*RQx + LQz*RQw,
  RRx is 2*(Rx*(0.5 - LQy*LQy - LQz*LQz) + Ry*(LQx*LQy - LQw*LQz) + Rz*(LQw*LQy + LQx*LQz)),
  RRy is 2*(Rx*(LQw*LQz + LQx*LQy) + Ry*(0.5 - LQx*LQx - LQz*LQz) + Rz*(LQy*LQz - LQw*LQx)),
  RRz is 2*(Rx*(LQx*LQz - LQw*LQy) + Ry*(LQw*LQx + LQy*LQz) + Rz*(0.5 - LQx*LQx - LQy*LQy)),
  Nx is Lx + RRx,
  Ny is Ly + RRy,
  Nz is Lz + RRz,
  =(UpdTr, [RefFrame, TgFrame, [Nx, Ny, Nz], [NQx, NQy, NQz, NQw]]).

replace_object_transform(O, T, NT) :-
  % Get ref frame of new transform
  nth0(0, NT, NRefFrame),
  % Get the transform data associated to the TemporaryTransform T, and the reference object it has
  get_associated_transform(_, O, _, T, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Td),
  rdf_has(T, assembly:'hasReferencePart', Ref),
  % Get the reference frame of that
  nth0(0, Td, ORefFrame),
  % See how the new reference frame relates to the pre-existing one
  get_tf_transform(ORefFrame, NRefFrame, TON),
  % Update the transform that should exist between the old reference frame and the object
  multiply_transforms(TON, NT, UpdatedTransform),
  % Replace the TemporaryTransform with a new one
  deactivate_temporal_extension(T),
  add_transform_to_object(O, UpdatedTransform, Ref),
  !.  
%% replace_object_transform(O, T, NT) :-
%%   % Get ref frame of new transform
%%   nth0(0, NT, NRefFrame),
%%   % Get the transform data associated to the TemporaryTransform T, and the reference object it has
%%   get_associated_transform(_, O, _, T, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Td),
%%   rdf_has(T, assembly:'hasReferencePart', Ref),
%%   % Get the reference frame of that
%%   nth0(0, Td, ORefFrame),
%%   % See how the new reference frame relates to the pre-existing one
%%   get_tf_transform(ORefFrame, NRefFrame, TON),
%%   % Update the transform that should exist between the old reference frame and the object
%%   multiply_transforms(TON, NT, UpdatedTransform),
%%   % Replace the TemporaryTransform with a new one
%%   deactivate_temporal_extension(T),
%%   add_transform_to_object(O, UpdatedTransform, Ref),
%%   !.  

replace_object_transforms(_, [], _) :-
  !.

replace_object_transforms(Object, [T|RTs], NT) :-
  replace_object_transform(Object, T, NT),
  replace_object_transforms(Object, RTs, NT),
  !.

replace_object_transforms(Object, NewTransform) :-
  findall(T, active_transform(Object, T), ActTrs),
  replace_object_transforms(Object, ActTrs, NewTransform).

object_type_has_color_restriction(OType, ColorId) :-
  owl_subclass_of(OType, Super),
  rdfs_individual_of(Super, owl:'Restriction'),
  rdf_has(Super, owl:'onProperty', paramserver:'hasColor'),
  rdf_has(Super, owl:'hasValue', ColorId).

ensure_object_color_restriction_met(OType, _) :-
  \+ object_type_has_color_restriction(OType, _),
  !.

ensure_object_color_restriction_met(OType, OId) :-
  object_type_has_color_restriction(OType, ColorId),
  atom_string(OIdAt, OId),
  atom_string(ColorIdAt, ColorId),
  rdf_assert(OIdAt, paramserver:'hasColor', ColorIdAt),
  !.

get_object_property_exactly1_restriction(Type, Rel, Aff, Rest) :-
  % Also check whether sub properties of Rel are being asserted
  findall(SP, owl_has(SP, 'http://www.w3.org/2000/01/rdf-schema#subPropertyOf', Rel), SPs),
  member(RelX, [Rel|SPs]),
  % Find a superclass that is a restriction
  owl_subclass_of(Type, Super),
  rdfs_individual_of(Super, owl:'Restriction'),
  % of the form RelX exactly 1 AffordanceType
  rdf_has(Super, owl:'onProperty', RelX),
  rdf_has(Super, owl:'qualifiedCardinality', literal(type(_, '1'))),
  rdf_has(Super, owl:'onClass', Aff),
  =(Super, Rest).

get_object_property_value_restriction(Type, Rel, Aff, Rest) :-
  % Also check whether sub properties of Rel are being asserted
  findall(SP, owl_has(SP, 'http://www.w3.org/2000/01/rdf-schema#subPropertyOf', Rel), SPs),
  member(RelX, [Rel|SPs]),
  % Find a superclass that is a restriction
  %% owl_subclass_of(Type, Super),
  rdfs_subclass_of(Type, Super),
  rdfs_individual_of(Super, owl:'Restriction'),
  % of the form RelX hasValue Aff
  rdf_has(Super, owl:'onProperty', RelX),
  rdf_has(Super, owl:'hasValue', Aff),
  =(Super, Rest).

get_connection_transform(ConnectionType, ReferenceObject, SubRef, Transform) :-
  get_object_property_value_restriction(ConnectionType, 'http://knowrob.org/kb/knowrob_assembly.owl#usesTransform', _, Restr),
  get_associated_transform(_, ReferenceObject, _, Restr, _, 'http://www.w3.org/2002/07/owl#hasValue', TrMngl),
  =(TrMngl, [_, _, Trnsl, Rot]),
  get_object_reference_frame(ReferenceObject, ReferenceObjectFrame),
  get_object_reference_frame(SubRef, SubRefFrame),
  =(Transform, [ReferenceObjectFrame, SubRefFrame, Trnsl, Rot]).

create_object_affordances(_, []).

create_object_affordances(Object, [Aff|Rest]) :-
  get_new_object_id(Aff, AffId),
  atom_string(AffIdAt, AffId),
  atom_string(ObjectAt, Object),
  rdf_assert(AffIdAt, rdf:type, Aff),
  rdf_assert(ObjectAt, assembly:'hasAffordance', AffIdAt),
  create_object_affordances(Object, Rest),
  !.

%% WARNING: so far assume that all affordances on an object have unique types
ensure_object_affordances(ObjectType, ObjectId) :-
  findall(A, get_object_property_exactly1_restriction(ObjectType, 'http://knowrob.org/kb/knowrob_assembly.owl#hasAffordance', A, _), AffordanceTypes),
  create_object_affordances(ObjectId, AffordanceTypes),
  !.

get_object_type_shapedata(OT, S) :-
  % Find a superclass that is a restriction
  owl_subclass_of(OT, Super),
  rdfs_individual_of(Super, owl:'Restriction'),
  % of the form hasShape value Shape
  rdf_has(Super, owl:'onProperty', paramserver:'hasShape'),
  rdf_has(Super, owl:'hasValue', S).

get_object_type_shapedatas(ObjectType, Shapes) :-
  findall(S, get_object_type_shapedata(ObjectType, S), Shapes).

link_object_to_shapes(_, []).

link_object_to_shapes(Object, [Shape|Rest]) :-
  atom_string(ObjectAt, Object),
  atom_string(ShapeAt, Shape),
  rdf_assert(ObjectAt, paramserver:'hasShape', ShapeAt),
  link_object_to_shapes(Object, Rest),
  !.

ensure_object_shape_data(ObjectType, ObjectId) :-
  get_object_type_shapedatas(ObjectType, Shapes),
  link_object_to_shapes(ObjectId, Shapes),
  !.

remove_supertypes([], Cr, Cr).

remove_supertypes([T|Rts], Cr, Result) :-
  \+ (member(X, Rts), owl_subclass_of(X, T)),
  remove_supertypes(Rts, [T|Cr], Result).

remove_supertypes([T|Rts], Cr, Result) :-
  member(X, Rts), 
  owl_subclass_of(X, T),
  remove_supertypes(Rts, Cr, Result).

update_affordance_relations(_, _, [], _).

update_affordance_relations(TempRei, Rel, [Type|RestTypes], Affs) :-
  select(A, Affs, RestAffs),
  owl_individual_of(A, Type),
  atom_string(TempReiAt, TempRei),
  atom_string(RelAt, Rel),
  atom_string(AAt, A),!,
  rdf_assert(TempReiAt, RelAt, AAt),!,
  update_affordance_relations(TempRei, Rel, RestTypes, RestAffs),
  !.

create_temporary_grasp(G, O, R, GraspSpecification, GraspRei) :-
  create_object_with_temporal_extent('http://knowrob.org/kb/knowrob_assembly.owl#TemporaryGrasp', GraspRei),
  atom_string(GraspReiAt, GraspRei),
  atom_string(OAt, O),
  atom_string(GraspSpecificationAt, GraspSpecification),
  rdf_assert(GraspReiAt, paramserver:'hasGripperType', literal(type(xsd:'anyURI', G))),
  rdf_assert(GraspReiAt, paramserver:'hasObjectType', literal(type(xsd:'anyURI', O))),
  rdf_assert(GraspReiAt, paramserver:'hasRobotType', literal(type(xsd:'anyURI', R))),
  rdf_assert(GraspReiAt, assembly:'hasSpecification', GraspSpecificationAt),
  rdf_assert(OAt, assembly:'isGrasped', GraspReiAt),
  findall(B, owl_has(GraspSpecification, 'http://knowrob.org/kb/knowrob_assembly.owl#blocksAffordanceType', literal(type(_, B))), BATs),
  findall(N, owl_has(GraspSpecification, 'http://knowrob.org/kb/knowrob_assembly.owl#needsAffordanceType', literal(type(_, N))), NATs),
  remove_supertypes(BATs, [], BlockedAffordanceTypes),
  remove_supertypes(NATs, [], NeededAffordanceTypes),
  findall(A, get_affordance(O, A), Affordances),
  update_affordance_relations(GraspRei, 'http://knowrob.org/kb/knowrob_assembly.owl#blocksAffordance', BlockedAffordanceTypes, Affordances),
  update_affordance_relations(GraspRei, 'http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeededAffordanceTypes, Affordances),
  !.

collect_assemblage_components([], CrComponents, Result) :-
  list_to_set(CrComponents, Result),!.

collect_assemblage_components([O|RestObjs], CrComponents, Result) :-
  owl_individual_of(O, assembly:'AtomicPart'),
  append(CrComponents, [O], NewComps),
  collect_assemblage_components(RestObjs, NewComps, Result).

collect_assemblage_components([O|RestObjs], CrComponents, Result) :-
  owl_individual_of(O, assembly:'Assemblage'),
  get_objects_in_assemblage(O, Os),
  append(CrComponents, Os, NewComps),
  collect_assemblage_components(RestObjs, NewComps, Result).

collect_available_affordances([], CrAffs, Affs) :-
  list_to_set(CrAffs, Affs).

collect_available_affordances(Os, CrAffs, Affs) :-
  \+ =(Os, []),
  nth0(0, Os, O, RestOs),
  findall(A, get_free_affordance(O, A), OAffs),
  append(CrAffs, OAffs, NewAffs), 
  collect_available_affordances(RestOs, NewAffs, Affs).

get_participating_object_available_affordances(Objects, Affordances) :-
  collect_available_affordances(Objects, [], Affordances).

create_connection(ConnectionType, AssemblageComponents, Connection) :-
  create_object_with_temporal_extent(ConnectionType, Connection),
  findall(BA, get_object_property_exactly1_restriction(ConnectionType, 'http://knowrob.org/kb/knowrob_assembly.owl#blocksAffordance', BA, _), BATs),
  findall(NA, get_object_property_exactly1_restriction(ConnectionType, 'http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NA, _), NATs),
  remove_supertypes(BATs, [], BlockedAffordanceTypes),
  remove_supertypes(NATs, [], NeededAffordanceTypes),
  get_participating_object_available_affordances(AssemblageComponents, Affordances),
  update_affordance_relations(Connection, 'http://knowrob.org/kb/knowrob_assembly.owl#blocksAffordance', BlockedAffordanceTypes, Affordances),
  update_affordance_relations(Connection, 'http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeededAffordanceTypes, Affordances),
  !.

%% Retrieves the reference part for an object.
get_reference_part(Object, ReferencePart) :-
  owl_individual_of(Object, assembly:'AtomicPart'),
  =(ReferencePart, Object).

get_reference_part(Object, ReferencePart) :-
  owl_individual_of(Object, assembly:'Assemblage'),
  rdf_has(Object, assembly:'usesConnection', C),
  rdf_has(C, assembly:'hasReferencePart', ReferencePart).

%% get_objects_connected_to_object(+Object, ConnObjs) is det.
%
% Returns a list of AtomicParts connected to Object, via inclusion in a subsuming assemblage; Object itself will be in the list.
%
% @param Object        anyURI representing the Object id
% @param ConnObjs      [anyURI*] list of object ids.
%
get_objects_connected_to_object(Object, [Object]) :-
  get_assemblages_with_object(Object, []), !.

get_objects_connected_to_object(Object, ConnObjs) :-
  get_assemblages_with_object(Object, SuperAssemblages),
  collect_assemblage_components(SuperAssemblages, [], ConnObjsIn),
  object_set_least_fixed_point(ConnObjsIn, ConnObjs), !.

is_mobile_object(O) :-
  \+ owl_individual_of(O, assembly:'FixedPart'),
  owl_individual_of(O, assembly:'AtomicPart'),!.

get_mobile_objects_connected_to_object(Object, ConnMobileObjs) :-
  get_objects_connected_to_object(Object, ConnObjs),
  findall(O, (member(O, ConnObjs), is_mobile_object(O)), ConnMobileObjsL),
  list_to_set(ConnMobileObjsL, ConnMobileObjs).

continue_fixed_point(OIn, OInt, OInt) :- 
  same_length(OIn, OInt),
  subset(OIn, OInt),
  !.

continue_fixed_point(_, OInt, Out) :-
  object_set_least_fixed_point(OInt, Out).

object_set_least_fixed_point(OsIn, OsOut) :-
  expand_object_set(OsIn, OsOutInt),
  continue_fixed_point(OsIn, OsOutInt, OsOut).

expand_object_set(OsIn, OsOut) :-
  get_assemblages_with_objects(OsIn, [], AsIn),
  collect_assemblage_components(AsIn, [], OsOutL),
  list_to_set(OsOutL, OsOut).

%% Returns a list of Assemblages that contain at least one of Objects
get_assemblages_with_objects([], CrA, As) :-
  list_to_set(CrA, As).

get_assemblages_with_objects([O|RO], CrA, As) :-
  get_assemblages_with_object(O, NA),
  append(CrA, NA, NCrA),
  get_assemblages_with_objects(RO, NCrA, As).

%% Returns the reference part of A (an assemblage or atomic part), if this reference part is mobile. Otherwise, returns nil.
get_mobile_reference(A, A) :-
  owl_individual_of(A, assembly:'AtomicPart'),
  \+ owl_individual_of(A, assembly:'FixedPart').
  
get_mobile_reference(A, MRP) :-
  rdf_has(A, assembly:'usesConnection', C),
  rdf_has(C, assembly:'hasReferencePart', MRP),
  \+ owl_individual_of(MRP, assembly:'FixedPart').
  
%% Returns an active transform for O that uses a TemporaryGrasp as a reference
active_grasp_referenced_transform(O, GRT) :-
  active_referenced_transform(O, R, GRT),
  owl_individual_of(R, assembly:'TemporaryGrasp').

%% Test that some object in COs isGrasped by Gr
connected_grasp(COs, Gr) :-
  member(CO, COs),
  rdf_has(CO, assembly:'isGrasped', Gr).

%% For a Tr that references a TemporaryGrasp, return the grasp in Tr if the grasp is inactive, or none of the COs isGrasped by it
grasp_reference_invalid(_, Tr, Gr) :-
  rdf_has(Tr, assembly:'hasReferencePart', Gr),
  \+ temporal_extent_active(Gr).

grasp_reference_invalid(COs, Tr, Gr) :-
  rdf_has(Tr, assembly:'hasReferencePart', Gr),
  temporal_extent_active(Gr),
  \+ connected_grasp(COs, Gr).

%% Retrieves a list of all TemporaryGrasp that Object uses as references for active TemporaryTransforms, such that each of these
%  TemporaryGrasps is either inactive or corresponds to an object that is not connected to Object
get_invalid_grasps_on_object(Object, InvalidGrasps) :-
  % Find all active transforms for Object that reference a grasp
  findall(GRT, active_grasp_referenced_transform(Object, GRT), GRTransforms),
  % Find all mobile connected objects to Object
  get_mobile_objects_connected_to_object(Object, ConnObjs),
  % From among the grasps used as reference by the active transforms, select those that are inactive or not on a connected object
  findall(G, (member(T, GRTransforms), grasp_reference_invalid(ConnObjs, T, G)), InvalidGraspsL),
  % Since several active transforms may reference the same grasp, explicitly coerce to a set
  list_to_set(InvalidGraspsL, InvalidGrasps).

%% Remove a grasp-referenced transformation from a list of objects
remove_grasp([], _) :-
  !.

remove_grasp([Object|RestObjects], Grasp) :-
  remove_transform_from_object_by_reference(Object, Grasp),
  remove_grasp(RestObjects, Grasp),
  !.

%% Removes transformations referenced to one of a list of grasps from a list of objects
remove_grasp_list(_, []) :-
  !.

remove_grasp_list(ObjList, GraspList) :-
  \+ =(GraspList, []),
  nth0(0, GraspList, Grasp, RestGrasps),
  remove_grasp(ObjList, Grasp),
  remove_grasp_list(ObjList, RestGrasps),
  !.

%% Applies a grasp to a list of objects; that is, creates TemporaryTransform objects referenced to the Grasp (if it is active).
apply_grasp([], _) :-
  !.

apply_grasp(_, Grasp) :-
  \+ temporal_extent_active(Grasp),
  !.

apply_grasp([Object|RestObjects], Grasp) :-
  temporal_extent_active(Grasp),
  get_object_transform(Object, TObj),
  add_transform_to_object_by_reference(Object, Grasp, TObj),
  apply_grasp(RestObjects, Grasp),
  !.

%% Applies a list of grasps to a list of objects; that means, creates TemporaryTransform objects referenced to the Grasps
apply_grasp_list(_, []) :-
  !.

apply_grasp_list(Objects, [Grasp|RestGrasps]) :-
  apply_grasp(Objects, Grasp),
  apply_grasp_list(Objects, RestGrasps),
  !.

%% For a list of objects directly connected via an 'isGrasped' relation to Grasp, remove the active TemporaryTransform using that grasp as reference.
% Also, for each such object, find all linked mobile reference parts and also remove any active TemporaryTransforms referencing Grasp as well.
ungrasp_objects([], _, CrDirtyObjects, DirtyObjects) :-
  list_to_set(CrDirtyObjects, DirtyObjects),
  !.

ungrasp_objects([Object|RestObjects], Grasp, CrDirtyObjects, DirtyObjects) :-
  remove_transform_from_object_by_reference(Object, Grasp),
  get_mobile_objects_connected_to_object(Object, MRPsO),
  delete(MRPsO, Object, MRPs),
  remove_grasp(MRPs, Grasp),
  append(MRPs, [Object], NewL),
  append(CrDirtyObjects, NewL, NewCrDirtyObjects),
  ungrasp_objects(RestObjects, Grasp, NewCrDirtyObjects, DirtyObjects),
  !.

%% Find all the active TemporaryTransforms that use an active TemporaryGrasp as reference, then collect all these TemporaryGrasps
get_indirect_grasps_on_object(Object, Grasps) :-
  findall(G, (rdf_has(Object, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', T), temporal_extent_active(T), rdf_has(T, 'http://knowrob.org/kb/knowrob_assembly.owl#hasReferencePart', G)), GraspList),
  list_to_set(GraspList, Grasps).

%% Ensures that an Assemblage has an assembly:'hasSubassemblage' relation to its component assemblages
assert_subassemblage(_, Component) :-
  owl_individual_of(Component, assembly:'AtomicPart'),
  !.

assert_subassemblage(Assemblage, Component) :-
  owl_individual_of(Component, assembly:'Assemblage'),
  atom_string(AssemblageAt, Assemblage),
  atom_string(ComponentAt, Component),!,
  rdf_assert(AssemblageAt, assembly:'hasSubassemblage', ComponentAt),
  !.


%% assert_object_at_location(+ObjectType, +ObjectId, +Transform) is det.
%
% Ensures that an object of the given type and id will exist in the knowledge base, with its active transform given by Transform.
% Translations are expected to be in meters.
% The target frame of Transform MUST equal ObjectId.
%
% @param ObjectType    anyURI representing the Object type
% @param ObjectId      anyURI representing the Object id
% @param Transform     [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]]
%
% If the object exists already, simply update its active transform.
assert_object_at_location(ObjectType, ObjectId, Transform) :-
  owl_individual_of(ObjectId, ObjectType),!,
  get_object_reference_frame(ObjectId, TargetFrameAtom),
  atom_string(TargetFrameAtom, TargetFrameStr),
  nth0(1, Transform, TargetFrameStr),!,
  replace_object_transforms(ObjectId, Transform),
  mark_dirty_objects([ObjectId]),
  !.

% If this is a new object, then we need to create a lot of things ...
% WARNING: an assumption is made about the status of newly created objects: a new object is free, ie. outside of grasps or assemblies.
% You can use subsequent queries to assert grasps and asseblage status.
assert_object_at_location(ObjectType, ObjectId, Transform) :-
  % Extract what the name for the object frame should be ...
  nth0(1, Transform, ObjectFrame),
  % Assert that the object exists ...
  atom_string(ObjectIdAt, ObjectId),
  rdf_assert(ObjectIdAt, rdf:type, ObjectType),
  % ... and assert that it has the given transform ...
  add_transform_to_object(ObjectId, Transform, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol'),
  % ... update paramserver:'ObjectReferenceFrameSymbol' to contain the frame name for this object ...
  get_new_object_id('http://knowrob.org/kb/knowrob_paramserver.owl#CoordinateFrameName', ObjFrameName),
  atom_string(ObjFrameNameAt, ObjFrameName),
  rdf_assert(ObjFrameNameAt, rdf:type, 'http://knowrob.org/kb/knowrob_paramserver.owl#CoordinateFrameName'),
  rdf_assert(ObjFrameNameAt, paramserver:'hasValue', literal(type(xsd:'string', ObjectFrame))),
  rdf_assert(ObjFrameNameAt, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectId))),
  rdf_assert(paramserver:'ObjectReferenceFrameSymbol', paramserver:'standsFor', ObjFrameNameAt),
  % ... assert color for the object in accordance to its class restrictions ...
  ensure_object_color_restriction_met(ObjectType, ObjectId),
  % ... assert shapeData for the object in accordance to its class restrictions ...
  ensure_object_shape_data(ObjectType, ObjectId),
  % ... create affordances for the object in accordance to its class restrictions ...
  ensure_object_affordances(ObjectType, ObjectId),
  mark_dirty_objects([ObjectId]),
  !.

%% assert_grasp_on_object(+Gripper, +Object, +Robot, +GraspSpecification, -GraspRei) is det.
%
% Updates the knowledge base with a new TemporaryGrasp object to represent a grasp on an object.
% DOES feasibility check: if the affordances required by the GraspSpecification are actively blocked,
% this fails without changing the knowledgebase.
%
% @param Gripper            anyURI representing the Gripper Id/type (something to disambiguate FrameSymbols with)
% @param Object             anyURI representing the Object Id
% @param Robot              anyURI representing the Robot Id/type (-''-)
% @param GraspSpecification anyURI representing a GraspSpecification individual
% @param GraspRei           anyURI representing a newly created TemporaryGrasp individual
%
assert_grasp_on_object(Gripper, Object, Robot, GraspSpecification, GraspRei) :-
  findall(A, get_free_affordance(Object, A, 'http://knowrob.org/kb/knowrob_assembly.owl#GraspingAffordance'), GraspAffordances),
  get_valid_grasp_for_object(Object, Gripper, GraspAffordances, GraspSpecification),
  create_temporary_grasp(Gripper, Object, Robot, GraspSpecification, GraspRei),
  get_associated_transform(Gripper, Object, Robot, GraspSpecification, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasGraspTransform', Transform),
  add_transform_to_object(Object, Transform, GraspRei),
  get_mobile_objects_connected_to_object(Object, MobileParts),
  % We need two lists here: one that definitely does NOT contain Object, and one that does
  delete(MobileParts, Object, MRPs),
  append(MRPs, [Object], DirtyObjects),
  atom_string(GraspReiAt, GraspRei),
  apply_grasp(MRPs, GraspReiAt),
  mark_dirty_objects(DirtyObjects),
  !.

get_pre_grasp_position(Gripper, ObjectId, GraspSpecification, T) :-
  get_associated_transform(Gripper, ObjectId, _ , GraspSpecification, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasPregraspTransform', T).

get_grasp_position(Gripper, ObjectId, GraspSpecification, T) :-
  get_associated_transform(Gripper, ObjectId, _ , GraspSpecification, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasGraspTransform', T).

get_post_grasp_position(Gripper, ObjectId, GraspSpecification, T) :-
  get_associated_transform(Gripper, ObjectId, _ , GraspSpecification, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasPostgraspTransform', T).

%% assert_ungrasp(+ObjectId, +GripperId) is det.
%
% Ensures that the Grasp for ObjectId becomes inactive and releases any objects it affects.
% GripperId should be used, if the Object is held with multiple hands.
% 
%
% @param Grasp  anyURI, the Grasp id
%
assert_ungrasp(ObjectId) :-
  assert_ungrasp(ObjectId, _).

assert_ungrasp(ObjectId, GripperId) :-
  assert_ungrasp_without_service_call(ObjectId, GripperId, DirtyObjects),
  mark_dirty_objects(DirtyObjects),
  !.

assert_ungrasp_without_service_call(ObjectId, GripperId, DirtyObjects) :-
  get_current_grasps_on_object(ObjectId, GraspsList),
  member(Grasp, GraspsList),
  nth0(0, Grasp, GripperId),
  nth0(1, Grasp, ObjectId),
  nth0(4, Grasp, GraspId),
  owl_individual_of(GraspId, assembly:'TemporaryGrasp'),
  % First, get all the objects in the Grasp (note: if Grasp is already destroyed, this list is empty),
  get_objects_in_grasp(GraspId, Objects),
  % deactivate the grasp for each object; must also remove it from any mobile reference parts linked to the objects
  ungrasp_objects(Objects, GraspId, [], DirtyObjects),
  % finally deactivate the grasp itself
  deactivate_temporal_extension(GraspId),
  !.

%% assert_assemblage_created(+AssemblageType, +ConnectionType, +ReferenceObject, +PrimaryObject, +SecondaryObject, -Assemblage) is det.
%
% Updates the knowledge base with a new Assemblage object to represent that certain previously existing objects were put together.
% WARNING: No feasibility check is performed, as in, no check that required affordances are actually available.
% (TODO: maybe fix this, and insert a predicate to check aff. availability)
%
% @param AssemblageType     anyURI representing the AssemblageType
% @param ConnectionType     anyURI representing the ConnectionType
% @param ReferenceObject    anyURI an Object Id which must be an AtomicPart and is used as the reference for the assemblage connection; should be equal to either one of PrimaryObject
%                           or SecondaryObject, or one of their reference parts
% @param PrimaryObject      anyURI an Object Id, which can be an individual of type MechanicalPart (Atomic and Assemblage included); one of the parts of the assemblage
% @param SecondaryObject    anyURI an Object Id, which can be an individual of type MechanicalPart (Atomic and Assemblage included); one of the parts of the assemblage
% @param Assemblage         anyURI representing a newly created Assemblage individual
%
assert_assemblage_created(AssemblageType, ConnectionType, ReferenceObject, PrimaryObject, SecondaryObject, Assemblage) :-
  % Some basic sanity checks ...
  nonvar(ReferenceObject),
  nonvar(PrimaryObject),
  nonvar(SecondaryObject),
  owl_subclass_of(AssemblageType, 'http://knowrob.org/kb/knowrob_assembly.owl#Assemblage'),
  owl_subclass_of(ConnectionType, 'http://knowrob.org/kb/knowrob_assembly.owl#AssemblyConnection'),
  owl_individual_of(ReferenceObject, 'http://knowrob.org/kb/knowrob_assembly.owl#AtomicPart'),
  % Get active reference parts of each assembly, and check that ReferenceObject is one of them
  get_reference_part(PrimaryObject, PrimRef),
  get_reference_part(SecondaryObject, SecRef),
  member(ReferenceObject, [PrimRef, SecRef]),
  % Get the reference object that will be secondary in the connection
  delete([PrimRef, SecRef], ReferenceObject, [SubRef]),
  % Get mobile reference parts of each assembly
  get_mobile_objects_connected_to_object(ReferenceObject, MRPPrimary),
  get_mobile_objects_connected_to_object(SubRef, MRPSecondary),
  % For each of Primary and SecondaryObject, get the grasps it does not share with the other
  % Note: we CANNOT use the assemblage References here, since either of these may be a fixed part
  % to which grasps will never apply
  % However, all mobile reference parts in an assemblage will reference the same active TemporaryGrasp
  nth0(0, MRPPrimary, POR),
  nth0(0, MRPSecondary, SOR),
  get_indirect_grasps_on_object(POR, PGrasps),!,
  get_indirect_grasps_on_object(SOR, SGrasps),!,
  subtract(PGrasps, SGrasps, PSpecGrasps),
  subtract(SGrasps, PGrasps, SSpecGrasps),
  % Create the AssemblyConnection object; we need to know all the objects in the assemblies here
  collect_assemblage_components([PrimaryObject, SecondaryObject], [], AssemblageComponents),
  create_connection(ConnectionType, AssemblageComponents, C),!,
  atom_string(CAt, C),
  atom_string(ReferenceObjectAt, ReferenceObject),
  rdf_assert(CAt, assembly:'hasReferencePart', ReferenceObjectAt),
  % Add the connection transform to the connection itself and to the SubRef
  get_connection_transform(ConnectionType, ReferenceObject, SubRef, Transform),
  add_transform_to_object(SubRef, Transform, C, TransformId),!,
  atom_string(TransformIdAt, TransformId),
  rdf_assert(CAt, paramserver:'hasTransform', TransformIdAt),
  % To each of Primary, SecondaryObject: apply the grasps specific to the other
  apply_grasp_list(MRPPrimary, SSpecGrasps),!,
  apply_grasp_list(MRPSecondary, PSpecGrasps),!,
  % Finally, create the assembly
  get_new_object_id(AssemblageType, Assemblage),
  atom_string(AssemblageAt, Assemblage),
  rdf_assert(AssemblageAt, rdf:type, AssemblageType),!,
  rdf_assert(AssemblageAt, assembly:'usesConnection', CAt),!,
  assert_subassemblage(Assemblage, PrimaryObject),!,
  assert_subassemblage(Assemblage, SecondaryObject),!,
  mark_dirty_objects(AssemblageComponents),
  !.


%% ensure_assemblage_transforms(+Assemblage, +ConnectionType, +ReferenceObject, +PrimaryObject, +SecondaryObject) is det.
%
% Updates individuals representing parts of an Assemblage by making sure that:
%   - for each of the two objects (sub-Assemblages or AtomicParts) brought together in the Assemblage, the reference part
%   is either ReferenceObject or has a transform in the frame of ReferenceObject, where this transform hasReference the
%   Assemblage connection
%   - there is one consistent set of tranforms referenced to grasps for every mobile component AtomicPart in Assemblage
%   (ie., every AtomicPart that is a component of either Primary or SecondaryObject and is not an individual of FixedPart)
%
% ASSUMPTION: Assemblage already exists and was created by putting together PrimaryObject and SecondaryObject, with
% ReferenceObject as reference part. This means that if someone calls get_objects_in_assemblage with the Assemblage as
% parameter, they will receive a set of objects that is the union of those in PrimaryObject, SecondaryObject.
%
% @param Assemblage         anyURI representing the Assemblage
% @param ConnectionType     anyURI representing the ConnectionType
% @param ReferenceObject    anyURI an Object Id which must be an AtomicPart and is used as the reference for the assemblage connection; should be equal to either one of PrimaryObject
%                           or SecondaryObject, or one of their reference parts
% @param PrimaryObject      anyURI an Object Id, which can be an individual of type MechanicalPart (Atomic and Assemblage included); one of the parts of the assemblage
% @param SecondaryObject    anyURI an Object Id, which can be an individual of type MechanicalPart (Atomic and Assemblage included); one of the parts of the assemblage
%
ensure_assemblage_transforms(Assemblage, ConnectionType, ReferenceObject, PrimaryObject, SecondaryObject) :-
  % Some basic sanity checks ...
  nonvar(Assemblage),
  nonvar(ConnectionType),
  nonvar(ReferenceObject),
  nonvar(PrimaryObject),
  nonvar(SecondaryObject),
  owl_individual_of(Assemblage, 'http://knowrob.org/kb/knowrob_assembly.owl#Assemblage'),
  owl_individual_of(PrimaryObject, 'http://knowrob.org/kb/knowrob_assembly.owl#MechanicalPart'),
  owl_individual_of(SecondaryObject, 'http://knowrob.org/kb/knowrob_assembly.owl#MechanicalPart'),
  owl_individual_of(ReferenceObject, 'http://knowrob.org/kb/knowrob_assembly.owl#AtomicPart'),
  % Get active reference parts of each assembly, and check that ReferenceObject is one of them
  get_reference_part(PrimaryObject, PrimRef),
  get_reference_part(SecondaryObject, SecRef),
  member(ReferenceObject, [PrimRef, SecRef]),
  % Get the reference object that will be secondary in the connection
  delete([PrimRef, SecRef], ReferenceObject, [SubRef]),
  % Get mobile reference parts of each assembly
  get_mobile_objects_in_assemblage(PrimaryObject, MRPPrimary),
  get_mobile_objects_in_assemblage(SecondaryObject, MRPSecondary),
  % Note: we CANNOT use the assemblage references PrimRef, SecRef here, since either of these may be a fixed part
  % to which grasps will never apply
  % Inside one of the subassemblages however all mobile parts should share the same grasps.
  nth0(0, MRPPrimary, POR),
  nth0(0, MRPSecondary, SOR),
  get_indirect_grasps_on_object(POR, PGrasps),!,
  get_indirect_grasps_on_object(SOR, SGrasps),!,
  subtract(PGrasps, SGrasps, PSpecGrasps),
  subtract(SGrasps, PGrasps, SSpecGrasps),
  % Unlike assert_assemblage_created, we assume we already have the components placed in the assemblage at this point
  get_objects_in_assemblage(Assemblage, AssemblageComponents),
  % Unlike assert_assemblage_created, we assume the connection already exists for assemblage-- so use it :)
  rdf_has(Assemblage, assembly:'usesConnection', C),
  atom_string(CAt, C),
  atom_string(ReferenceObjectAt, ReferenceObject),
  % TOCHECK: The Connection should already have a reference object associated with it, so the next line is commented out
  %% rdf_assert(CAt, assembly:'hasReferencePart', ReferenceObjectAt),
  % Add the connection transform to the connection itself and to the SubRef
  ensure_subref_transform(ConnectionType, C, ReferenceObject, SubRef, TransformId),
  ensure_connection_transform(C, TransformId),
  % To each of Primary, SecondaryObject: apply the grasps specific to the other
  % Note: if somehow this function was called previously on this Assemblage, then all its mobile components already have the same
  % set of grasps, meaning the SSpecGrasps, PSpecGrasps sets are empty and nothing will change.
  apply_grasp_list(MRPPrimary, SSpecGrasps),!,
  apply_grasp_list(MRPSecondary, PSpecGrasps),!,
  % TOCHECK: The consistency-check control loop probably takes care of this, so the following lines are commented out
  %% assert_subassemblage(Assemblage, PrimaryObject),!,
  %% assert_subassemblage(Assemblage, SecondaryObject),!,
  % Still need to republish transforms and viz. markers though
  mark_dirty_objects(AssemblageComponents),
  !.

ensure_subref_transform(_, Connection, _, SubRef, TransformId) :-
  % There is an assumption here: that if an active transform exists for SubRef and it is referenced to Connection, then it is a transform
  % with the correct ReferenceObject as the reference
  active_referenced_transform(SubRef, Connection, TempRei),
  rdf_has(TempRei, paramserver:'hasTransform', TransformId),
  !.

ensure_subref_transform(ConnectionType, Connection, ReferenceObject, SubRef, TransformId) :-
  get_connection_transform(ConnectionType, ReferenceObject, SubRef, Transform),
  add_transform_to_object(SubRef, Transform, Connection, TransformId),
  !.

ensure_connection_transform(Connection, _) :-
  % There is an assumption here: that once there is a hasTransform from this connection it is the correct one. 
  rdf_has(Connection, paramserver:'hasTransform', _),
  !.

ensure_connection_transform(Connection, TransformId) :-
  % TOCHECK: The consistency-check control loop probably does not assert a transform for the connection, we put that in here
  atom_string(CAt, Connection),
  atom_string(TransformIdAt, TransformId),
  rdf_assert(CAt, paramserver:'hasTransform', TransformIdAt),
  !.

%% assert_assemblage_destroyed(+Assemblage) is det.
%
% Finds the Connection used by Assemblage, and if its temporalExtent lacks an endsAtTime,
% it asserts a new TimePoint to be its end time.
%
% @param Assemblage  anyURI, the Assemblage id
%
assert_assemblage_destroyed(Assemblage) :-
  assert_assemblage_destroyed_without_service_call(Assemblage, DirtyObjects),
  mark_dirty_objects(DirtyObjects),
  !.

assert_assemblage_destroyed_without_service_call(Assemblage, DirtyObjects) :-
  owl_individual_of(Assemblage, assembly:'Assemblage'),
  % First, get all the objects in the Assemblage (note: if Assemblage is already destroyed, this list is empty),
  get_objects_in_assemblage(Assemblage, DirtyObjects),
  % then the Assemblage Connection, and deactivate it,
  owl_has(Assemblage, assembly:'usesConnection', Connection),
  % so that the Assemblage is now destroyed;
  deactivate_temporal_extension(Connection),
  % retrieve ReferenceObject and SubObject
  rdf_has(Connection, paramserver:'hasTransform', TransformId),
  rdf_has(TransformId, paramserver:'hasReferenceFrame', RefFrId),
  rdf_has(TransformId, paramserver:'hasTargetFrame', TgFrId),
  rdf_has(RefFrId, paramserver:'hasValue', literal(type(xsd:'string', ReferenceObjectFrame))),
  rdf_has(TgFrId, paramserver:'hasValue', literal(type(xsd:'string', SubRefFrame))),
  get_object_reference_frame(ReferenceObject, ReferenceObjectFrame),
  get_object_reference_frame(SubRef, SubRefFrame),
  remove_transform_from_object_by_reference(SubRef, Connection),
  % Now that the assemblage is no more, we can get two different lists of mobile reference parts starting from
  % those two objects
  get_mobile_objects_connected_to_object(ReferenceObject, MRPPrimary),
  get_mobile_objects_connected_to_object(SubRef, MRPSecondary),
  % However, since they were just in an assembly, they will all have the same grasps active
  % Note as above that we need a mobile reference part to get active grasps; ReferenceObject and SubRef may be FixedParts to which
  % no grasps are applied, so we need to make sure we get a mobile reference part
  nth0(0, MRPPrimary, POR),
  nth0(0, MRPSecondary, SOR),
  % We now need to find which grasps are invalid on each mobile reference: these grasps are not on an object linked to the reference via active Connections
  get_invalid_grasps_on_object(POR, PInvalidGrasps),
  get_invalid_grasps_on_object(SOR, SInvalidGrasps),
  remove_grasp_list(MRPPrimary, PInvalidGrasps),
  remove_grasp_list(MRPSecondary, SInvalidGrasps),
  !.

get_leaf_subtype(SuperType, LeafSubType) :-
  =(SuperType, LeafSubType).
%  owl_subclass_of(LeafSubType, SuperType),
%  \+ owl_subclass_of(_, LeafSubType).

get_assemblage_type_possible_connections(ConnectionDescription, Connections) :-
  owl_has(ConnectionDescription, owl:onClass, Class),
  \+ owl_same_as(Class, assembly:'AssemblyConnection'),
  \+ rdf_has(Class, owl:intersectionOf, _),
  findall(L, get_leaf_subtype(Class, L), Connections).

get_assemblage_type_possible_connections(ConnectionDescription, Connections) :-
  owl_has(ConnectionDescription, owl:onClass, Description),
  \+ owl_same_as(Description, assembly:'AssemblyConnection'),
  rdf_has(Description, owl:intersectionOf, I),
  rdfs_list_to_prolog_list(I, IntersectionTerms),
  member(ConnectionType, IntersectionTerms),
  owl_subclass_of(ConnectionType, assembly:'AssemblyConnection'),
  \+ owl_individual_of(ConnectionType, owl:'Restriction'),
  findall(L, get_leaf_subtype(ConnectionType, L), Connections).

get_assemblage_atomic_parts(AssemblageType, PrimaryAtomicParts, SecondaryAtomicParts) :-
  findall(X, get_object_property_exactly1_restriction(AssemblageType, 'http://knowrob.org/kb/knowrob_assembly.owl#hasPart', X, _), Parts),
  assign_ps_atomic_parts(Parts, PrimaryAtomicParts, SecondaryAtomicParts).

assign_ps_atomic_parts([], [], []).

assign_ps_atomic_parts([P], PPs, []) :-
  findall(Y, get_leaf_subtype(P, Y), PPs).

assign_ps_atomic_parts([P, S], PPs, SPs) :-
  findall(Y, get_leaf_subtype(P, Y), PPs),
  findall(X, get_leaf_subtype(S, X), SPs).

get_assemblage_subassemblages(ConnectionDescription, [], []) :-
  owl_has(ConnectionDescription, owl:onClass, Description),
  owl_individual_of(Description, assembly:'AssemblyConnection').

get_assemblage_subassemblages(ConnectionDescription, [], []) :-
  owl_has(ConnectionDescription, owl:onClass, Description),
  \+ owl_individual_of(Description, assembly:'AssemblyConnection'),
  \+ rdf_has(Description, owl:intersectionOf, _).

get_assemblage_subassemblages(ConnectionDescription, PAs, SAs) :-
  owl_has(ConnectionDescription, owl:onClass, Description),
  \+ owl_individual_of(Description, assembly:'AssemblyConnection'),
  get_assemblage_subassemblages_internal(Description, PAs, SAs).

get_required_linked_subassemblage(Restr, SA) :-
  rdfs_individual_of(Restr, owl:'Restriction'),!,
  owl_has(Restr, owl:onProperty, assembly:'linksAssemblage'),
  owl_has(Restr, owl:someValuesFrom, SA).

get_assemblage_subassemblages_internal(Description, PAs, SAs) :-
  rdf_has(Description, owl:intersectionOf, I),
  rdfs_list_to_prolog_list(I, IntersectionTerms),
  findall(X, (member(IT, IntersectionTerms), get_required_linked_subassemblage(IT, X)), ATs),
  assign_ps_subassemblages(ATs, PAs, SAs).

assign_ps_subassemblages([], [], []).

assign_ps_subassemblages([SA], [], SAs) :-
  findall(X, get_leaf_subtype(SA, X), SAs).

assign_ps_subassemblages([PA, SA], PAs, SAs) :-
  findall(Y, get_leaf_subtype(PA, Y), PAs),
  findall(X, get_leaf_subtype(SA, X), SAs).

create_assembly_agenda_internal(AssemblageType, AvailableAtomicParts, [], NewAvailableAtomicParts, AssemblageName) :-
  owl_subclass_of(AssemblageType, assembly:'AtomicPart'),
  member(AssemblageName, AvailableAtomicParts),
  owl_individual_of(AssemblageName, AssemblageType),!,
  delete(AvailableAtomicParts, AssemblageName, NewAvailableAtomicParts).

create_assembly_agenda_internal(AssemblageType, AvailableAtomicParts, Agenda, NewAvailableAtomicParts, AssemblageName) :-
  \+ owl_subclass_of(AssemblageType, assembly:'AtomicPart'),
  get_object_property_exactly1_restriction(AssemblageType, 'http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', _, ConnectionDescription),
  get_assemblage_type_possible_connections(ConnectionDescription, Connections),
  get_assemblage_atomic_parts(AssemblageType, PAPs, SAPs),
  get_assemblage_subassemblages(ConnectionDescription, PAs, SAs),
  append(PAPs, PAs, PrimaryAssemblages),
  append(SAPs, SAs, SecondaryAssemblages),
  member(Connection, Connections),
  member(PrimaryType, PrimaryAssemblages),
  member(SecondaryType, SecondaryAssemblages),
  create_assembly_agenda_internal(PrimaryType, AvailableAtomicParts, PrimaryAgenda, AvailableAfterPrimary, PrimaryObject),
  create_assembly_agenda_internal(SecondaryType, AvailableAfterPrimary, SecondaryAgenda, NewAvailableAtomicParts, SecondaryObject),
  append(PrimaryAgenda, SecondaryAgenda, SubAssemblyAgenda),
  get_new_object_id('Assemblage', AssemblageName),
  append(SubAssemblyAgenda, [[AssemblageType, Connection, PrimaryObject, SecondaryObject, AssemblageName]], Agenda).

%% create_assembly_agenda(+AssemblageType, +AvailableAtomicParts, -Agenda) is det.
create_assembly_agenda(AssemblageType, AvailableAtomicParts, Agenda) :-
  create_assembly_agenda_internal(AssemblageType, AvailableAtomicParts, Agenda, _, _).

%% reset_beliefstate() :-
%%   (assert_assemblage_destroyed_without_service_call(_,_) -> true; true),
%%   (assert_ungrasp_without_service_call(_,_,_) -> true; true),
%%   findall(ObjectId, 
%%     (
%%       rdf_has(ObjectId, paramserver:'hasInitialTransform', TempRei), 
%%       rdf_retractall(ObjectId,'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform',_), 
%%       rdf_retractall(TempRei,'http://knowrob.org/kb/knowrob_assembly.owl#temporalExtent',_),
%%       rdf_has(ObjectId, paramserver:'hasInitialTransform', TempRei), 
%%       rdf_assert(ObjectId,'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform',TempRei)
%%       ), 
%%     ObjectIds),
%%   mark_dirty_objects(ObjectIds).

reset_beliefstate() :-
  (assert_assemblage_destroyed_without_service_call(_,_) -> true; true),
  (assert_ungrasp_without_service_call(_,_,_) -> true; true),
  findall(ObjectId, 
    (
      rdfs_individual_of(ObjectId, 'http://knowrob.org/kb/knowrob_assembly.owl#AtomicPart'),
      rdf_has(ObjectId, paramserver:'hasTransform', TempRei), 
      rdf_retractall(ObjectId,'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform',_), 
      rdf_retractall(TempRei,'http://knowrob.org/kb/knowrob_assembly.owl#temporalExtent',_),
      (rdf_has(ObjectId, paramserver:'hasInitialTransform', InitialTempRei) -> 
        rdf_assert(ObjectId,'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', InitialTempRei);
        (rdf_retractall(ObjectId,_,_),false))
      ), 
    DirtyObjectIds),
  mark_dirty_objects(DirtyObjectIds).
