/** <module> knowrob_beliefstate

  Copyright (C) 2017 Mihai Pomarlan, Daniel Beßler

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

  @author Mihai Pomarlan, Daniel Beßler
  @license BSD
*/

:- module(knowrob_beliefstate,
    [
      get_known_object_ids/1,
      get_object_color/2,
      get_object_mesh_path/2,
      get_object_transform/2,
      
      belief_at/2,
      belief_at/3,
      belief_at_update/1,
      belief_at_location/4,
      belief_at_internal/2,
      belief_at_internal/3,
      belief_at_invert_topology/2,
      belief_at_global/2
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('util')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_math')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_paramserver')).
:- use_module(library('random')).

%:- load_foreign_library('libkr_beliefstate.so').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(beliefstate, 'http://knowrob.org/kb/knowrob_beliefstate.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:-  rdf_meta
    belief_at(r,+,r),
    belief_at(r,+),
    belief_at_internal(r,+,r),
    belief_at_internal(r,+),
    belief_at_invert_topology(r,r),
    belief_at_global(r,-),
    belief_at_update(t).

quote_id(X, O) :-
  atom_string(X, Oxx),
  string_concat('"', Oxx, Ox),
  string_concat(Ox, '"', O).

%% belief_at_update(-ObjectIds) is det.
%
belief_at_update(ObjectIds) :-
  findall(O, (member(X, ObjectIds), quote_id(X, O)), Os),
  atomic_list_concat(Os, ',', OsS),
  atom_string("'[", LB),
  string_concat(LB, OsS, PS),
  atom_string("]'", RB),
  string_concat(PS, RB, ParStr),
  atom_string("rosservice call /object_state_publisher/mark_dirty_object ", CmdKern),
  string_concat(CmdKern, ParStr, Cmd),
  thread_create(shell(Cmd), _, []).

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
  object_color(ObjectId,V), !,
  knowrob_math:parse_vector(V, Color).
get_object_color(ObjectId, Color) :-
  Color = [0.5, 0.5, 0.5, 1.0], !.

%% get_object_mesh_path(+ObjectId, -FilePath) is det.
%
% Returns a path to a mesh file (stl or dae) for the ObjectId.
%
% @param ObjectId    anyURI, the object id
% @param FilePath    anyURI, the path (usually a package:// path)
%
get_object_mesh_path(ObjectId, FilePath) :-
  once((
    owl_has(ObjectId, paramserver:'hasShape', ShapeCan),
    rdf_has(ShapeCan, paramserver:'hasPath', literal(type(xsd:'anyURI', FP)))
  )),
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
  belief_at(ObjectId, Transform), !.

translations_are_close([X1,Y1,Z1], [X2,Y2,Z2], Dmax) :-
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

rotations_are_close([X1,Y1,Z1,W1], [X2,Y2,Z2,W2], Dmax) :-
  % First compare the two quaternions as they are ...
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dw is W1 - W2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz + Dw*Dw,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

rotations_are_close([X1,Y1,Z1,W1], [X2,Y2,Z2,W2], Dmax) :-
  % ... if that fails, compare R1 with -R2
  Dx is X1 + X2,
  Dy is Y1 + Y2,
  Dz is Z1 + Z2,
  Dw is W1 + W2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz + Dw*Dw,
  DmaxSq is Dmax*Dmax,
  compare(<, Dsq, DmaxSq).

% TODO(DB) use tf prolog
get_tf_transform(ReferenceFrame, TargetFrame,
                [ReferenceFrame, TargetFrame, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]) :-
  get_current_tf(ReferenceFrame, TargetFrame, Tx, Ty, Tz, Rx, Ry, Rz, Rw).
  

%% belief_at_location(+ObjectType, +Transform, +Thresholds, -ObjectId) is det.
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
% @param Thresholds   a distance below which two translations are thought to be the same, and
%                     a distance below which two rotations are thought to be the same;
%                     NOTE: this threshold is interpreted as a euclidean distance threshold in a space of quaternions where +Quat and -Quat are the same.
% @param ObjectId        anyURI, the object id
%
belief_at_location(ObjectType,
                  [ReferenceFrame,_,ArgTranslation,ArgRotation],
                  [TranThreshold, RotThreshold],
                   ObjectId) :-
  rdfs_individual_of(ObjectId, ObjectType),
  rdf_has(ObjectId, srdl2comp:'urdfName', literal(TargetFrame)),
  get_tf_transform(ReferenceFrame, TargetFrame, [_,_,ObjTranslation,ObjRotation]),
  translations_are_close(ArgTranslation, ObjTranslation, TranThreshold),
  rotations_are_close(ArgRotation, ObjRotation, RotThreshold).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Beliefs about the spatial location of things

create_transform(Translation, Rotation, TransformId) :-
  rdf_instance_from_class('http://knowrob.org/kb/knowrob_paramserver.owl#Transform', TransformId),
  atomic_list_concat(Translation, ' ', Translation_atom),
  atomic_list_concat(Rotation, ' ', Rotation_atom),
  rdf_assert(TransformId, knowrob:'translation', literal(type(xsd:string,Translation_atom))),
  rdf_assert(TransformId, knowrob:'quaternion', literal(type(xsd:string,Rotation_atom))).

transform_reference_frame(TransformId, Ref) :-
  rdf_has(TransformId, knowrob:'relativeTo', RefObjId),
  rdf_has(RefObjId, srdl2comp:'urdfName', literal(Ref)), !.
transform_reference_frame(_TransformId, 'map').

%% belief_at(+Obj, +TransformData, +RelativeTo) is det.
%% belief_at(+Obj, +TransformData) is det.
%
belief_at(Obj, TransformData, RelativeTo) :-
  ground(TransformData), !,
  belief_at_internal(Obj, TransformData, RelativeTo),
  belief_at_update([Obj]).
belief_at(Obj, TransformData, RelativeTo) :-
  belief_at(Obj, TransformData),
  rdf_has(Obj, paramserver:'hasTransform', TransformId),
  rdf_has(TransformId, knowrob:'relativeTo', RelativeTo).

belief_at(Obj, TransformData) :-
  ground(TransformData), !,
  belief_at_internal(Obj, TransformData),
  belief_at_update([Obj]).
belief_at(Obj, [ReferenceFrame, TargetFrame, Translation, Rotation]) :-
  rdf_has(Obj, paramserver:'hasTransform', TransformId),
  rdf_has(Obj, srdl2comp:'urdfName', literal(TargetFrame)),
  transform_data(TransformId, (Translation, Rotation)),
  transform_reference_frame(TransformId, ReferenceFrame).

belief_at_internal(Obj, TransformData, RelativeTo) :-
  belief_at_internal_(Obj, TransformData, TransformId),
  rdf_assert(TransformId, knowrob:'relativeTo', RelativeTo).
belief_at_internal(Obj, TransformData) :-
  belief_at_internal_(Obj, TransformData, _).
belief_at_internal_(Obj, (Translation, Rotation), TransformId) :-
  rdf_retractall(Obj, paramserver:'hasTransform', _),
  create_transform(Translation, Rotation, TransformId),
  rdf_assert(Obj, paramserver:'hasTransform', TransformId).

belief_at_invert_topology(Child, Parent) :-
  rdf_has(Child, paramserver:'hasTransform', TransformId),
  rdf_has(TransformId, knowrob:'relativeTo', Parent),
  transform_data(TransformId, ([TX,TY,TZ],Q)),
  quaternion_inverse(Q,Q_inv),
  X is -TX, Y is -TY, Z is -TZ,
  quaternion_transform(Q_inv,[X,Y,Z],T_inv),
  belief_at_internal(Parent, (T_inv,Q_inv), Child).

belief_at_global(Obj, GlobalPose) :-
  rdf_has(Obj, srdl2comp:'urdfName', literal(ChildFrame)),
  rdf_has(Obj, paramserver:'hasTransform', TransformId),
  transform_data(TransformId, (T,Q)),
  ( rdf_has(TransformId, knowrob:'relativeTo', Parent) -> (
    belief_at_global(Parent, GlobalTransform),
    rdf_has(Parent, srdl2comp:'urdfName', literal(ParentFrame)),
    transform_multiply(GlobalTransform, [ParentFrame,ChildFrame,T,Q], GlobalPose)
  ) ; GlobalPose=['map',ChildFrame,T,Q]).
