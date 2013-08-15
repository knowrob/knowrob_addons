%%
%% Copyright (C) 2013 by Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- module(knowrob_mongo,
    [
      mng_lookup_transform/4,
      mng_latest_designator_before_time/3,
      mng_tf_pose_at_time/4,
      mng_tf_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_coordinates')).

:- owl_parser:owl_parse('../owl/knowrob_mongo.owl', false, false, true).


:-  rdf_meta
    mng_lookup_transform(+,+,r,-),
    mng_latest_designator_before_time(r,-,-),
    mng_tf_pose(r, r),
    mng_tf_pose_at_time(r, +, r, r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#', [keep(true)]).



%% mng_latest_designator_before_time(+TimePoint, -Type, -Pose) is nondet.
%
% @param TimePoint  Instance of knowrob:TimePoint
% @param Type       'Type' property of the designator
% @param PoseList   Object pose from designator as list[16]
%
mng_latest_designator_before_time(TimePoint, Type, PoseList) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'latestUIMAPerceptionBefore', [Time], Designator),

  jpl_call(Designator, 'get', ['type'], Type),

  jpl_call(Designator, 'get', ['pose'], StampedPose),
  jpl_call(StampedPose, 'getMatrix4d', [], PoseMatrix4d),
  knowrob_coordinates:matrix4d_to_list(PoseMatrix4d, PoseList).


%% mng_lookup_transform(+Target, +Source, +TimePoint, -Transform) is nondet.
%
% @param Target     Target frame ID
% @param Source     Source frame ID
% @param TimePoint  Instance of knowrob:TimePoint
% @param Transform  Transformation matrix as list[16]
%
mng_lookup_transform(Target, Source, TimePoint, Transform) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'lookupTransform', [Target, Source, Time], StampedTransform),

  jpl_call(StampedTransform, 'getMatrix4', [], TransformMatrix4d),
  knowrob_coordinates:matrix4d_to_list(TransformMatrix4d, Transform).


%% mng_tf_pose(+RobotPart, -Pose) is nondet.
%
% Read the pose of RobotPart in /map coordinates from logged tf data, default to 'now'
%
% @param RobotPart  Instance of a robot part with the 'urdfName' property set
% @param Pose       Instance of a knowrob:RotationMatrix3D with the pose data
%
mng_tf_pose(RobotPart, Pose) :-

  get_timepoint(TimePoint),
  mng_tf_pose_at_time(RobotPart, '/map', TimePoint, Pose).


%% mng_tf_map_pose_at_time(+RobotPart, +Frame, +TimePoint, +Pose) is nondet.
%
% Read the pose of RobotPart in the given coordinate frame from logged tf data
%
% @param RobotPart  Instance of a robot part with the 'urdfName' property set
% @param Frame      Atom with tf frame ID in which the pose shall be returned (e.g. '/map')
% @param TimePoint  Instance of knowrob:TimePoint
% @param Pose       Instance of a knowrob:RotationMatrix3D with the pose data
%
mng_tf_pose_at_time(RobotPart, Frame, TimePoint, Pose) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),
  TimeInt is round(Time),

  owl_has(RobotPart, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName', literal(SourceFrameID)),
  atom_concat('/', SourceFrameID, SourceFrame),

  knowrob_coordinates:list_to_matrix4d([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], MatrixIn),
  jpl_new('ros.communication.Time', [], TimeIn),
  jpl_set(TimeIn, 'secs', TimeInt),
  jpl_new('tfjava.Stamped', [MatrixIn, SourceFrame, TimeIn], StampedIn),

  knowrob_coordinates:list_to_matrix4d([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], MatrixOut),
  jpl_new('ros.communication.Time', [], TimeOut),
  jpl_set(TimeOut, 'secs', TimeInt),
  jpl_new('tfjava.Stamped', [MatrixOut, '/base_link', TimeOut], StampedOut),


  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'transformPose', [Frame, StampedIn, StampedOut], @(true)),

  jpl_call(StampedOut, 'getData', [], MatrixOut2),
  knowrob_coordinates:matrix4d_to_list(MatrixOut2, PoseList),
  create_pose(PoseList, Pose),

  % TODO: set previousDetectionOfObject / latestDetectionOfObject
  create_perception_instance(['Proprioception'], Perception),
  set_object_perception(RobotPart, Perception),
  rdf_assert(Perception, knowrob:eventOccursAt, Pose),

  % set time point for pose
  jpl_get(TimeOut, 'secs', PoseTime),
  term_to_atom(PoseTime, PoseTimeAtom),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob_mongo.owl#timepoint_', PoseTimeAtom, PoseTimePoint),

  rdf_assert(PoseTimePoint, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#TimePoint'),
  rdf_assert(Perception, knowrob:startTime, PoseTimePoint).



%% obj_visible_in_camera(+Obj, ?Camera, +TimePoint) is nondet.
%
% Check if Obj is visible by Camera at time TimePoint.
%
obj_visible_in_camera(Obj, Camera, TimePoint) :-

  findall(Camera, owl_individual_of(Camera, srdl2comp:'Camera'), Cameras),
  member(Camera, Cameras),

  % Read camera properties: horizontal field of view, aspect ratio -> vertical field of view
  once(owl_has(Camera, srdl2comp:hfov, literal(type(_, HFOVa)))),
  term_to_atom(HFOV, HFOVa),

  once(owl_has(Camera, srdl2comp:imageSizeX, literal(type(_, ImgXa)))),
  term_to_atom(ImgX, ImgXa),

  once(owl_has(Camera, srdl2comp:imageSizeY, literal(type(_, ImgYa)))),
  term_to_atom(ImgY, ImgYa),

  VFOV is ImgY / ImgX * HFOV,


  % Read object pose w.r.t. camera
  once(owl_has(Camera, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName', literal(CamFrameID))),
  atom_concat('/', CamFrameID, CamFrame),

  mng_tf_pose_at_time(Obj, CamFrame, TimePoint, RelObjPose),

  owl_has(RelObjPose, knowrob:m03, literal(type(_,ObjX))),
  owl_has(RelObjPose, knowrob:m13, literal(type(_,ObjY))),
  owl_has(RelObjPose, knowrob:m23, literal(type(_,ObjZ))),

  BearingX is atan2(ObjY, ObjX) / pi * 180,
  BearingY is atan2(ObjZ, ObjX) / pi * 180,

  BearingX < HFOV,
  BearingY < VFOV.






