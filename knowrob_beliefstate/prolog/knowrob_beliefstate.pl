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
      get_known_object_ids/1,
      get_object_color/2,
      get_object_mesh_path/2,
      get_object_transform/2,
      get_object_at_location/5,
      get_new_object_id/2
%      get_all_possible_grasps_on_object/2,
%      get_currently_possible_grasps_on_object/3,
%      get_current_grasps_on_object/2,
%      get_known_assemblage_ids/1,
%      get_assemblages_with_object/2,
%      get_objects_in_assemblage/2,
%      assert_free_object_at_location/3,
%      assert_grasp_on_object/2,
%      assert_ungrasp_on_object/2,
%      assert_assemblage_created/3,
%      assert_assemblage_destroyed/1
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
:- use_module(library('knowrob_mongo_tf')).
:- use_module(library('random')).


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(beliefstate, 'http://knowrob.org/kb/knowrob_beliefstate.owl#',  [keep(true)]).


%% get_known_object_ids(-ObjectIds) is det.
%
% Returns a list of strings representing the individuals of type MechanicalPart that are known to the KB.
%
% @param ObjectIds    [anyURI*], the object ids
%
get_known_object_ids(ObjectIds) :-
  findall(J, rdfs_individual_of(J, 'http://knowrob.org/kb/knowrob_assembly.owl#AtomicPart'), ObjectIds).

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
  \+ rdf_has(ObjectId, paramserver:'hasColor', _),
  =(Color, [0.5, 0.5, 0.5, 1.0]).

get_object_color(ObjectId, Color) :-
  rdf_has(ObjectId, paramserver:'hasColor', ColInd),
  rdf_has(ColInd, paramserver:'hasRed', RedInd),
  rdf_has(ColInd, paramserver:'hasGreen', GreenInd),
  rdf_has(ColInd, paramserver:'hasBlue', BlueInd),
  rdf_has(ColInd, paramserver:'hasAlpha', AlphaInd),
  rdf_has(RedInd, paramserver:'hasValue', literal(type(_, Red))),
  rdf_has(GreenInd, paramserver:'hasValue', literal(type(_, Green))),
  rdf_has(BlueInd, paramserver:'hasValue', literal(type(_, Blue))),
  rdf_has(AlphaInd, paramserver:'hasValue', literal(type(_, Alpha))),
  =(Color, [Red, Green, Blue, Alpha]).

%% get_object_mesh_path(+ObjectId, -FilePath) is det.
%
% Returns a path to a mesh file (stl or dae) for the ObjectId.
%
% @param ObjectId    anyURI, the object id
% @param FilePath    anyURI, the path (usually a package:// path)
%
get_object_mesh_path(ObjectId, FilePath) :-
  rdf_has(ObjectId, paramserver:'hasShape', ShapeCan),
  rdfs_individual_of(ShapeCan, paramserver:'Shape'),
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
% @param ObjectId    anyURI, the object id
% @param Transform   the transform data
%
get_object_transform(ObjectId, Transform) :-
  rdf_has(ObjectId, paramserver:'hasTransform', TempRei),
  rdf_has(TempRei, paramserver:'temporalExtent', TempExt),
  \+ rdf_has(TempExt, paramserver:'endsAtTime', _),
  get_associated_transform(_, _, _, TempRei, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform).


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
  get_timepoint(TimePoint),
  mng_lookup_transform(ReferenceFrame, TargetFrame, TimePoint, MTF),
  nth0(3, MTF, X),
  nth0(7, MTF, Y),
  nth0(11, MTF, Z),
  nth0(0, MTF, M00),
  nth0(1, MTF, M01),
  nth0(2, MTF, M02),
  nth0(4, MTF, M10),
  nth0(5, MTF, M11),
  nth0(6, MTF, M12),
  nth0(8, MTF, M20),
  nth0(9, MTF, M21),
  nth0(10, MTF, M22),
  matrix_to_quaternion(M00, M01, M02, M10, M11, M12, M20, M21, M22, QX, QY, QZ, QW),
  =(TFTransform, [ReferenceFrame, TargetFrame, [X, Y, Z], [QX, QY, QZ, QW]]).
  

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
  get_object_transform(Object, ObjTransform),
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

