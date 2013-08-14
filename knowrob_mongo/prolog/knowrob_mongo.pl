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
      mng_latest_designator_before_time/3
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
    mng_latest_designator_before_time(r,-,-).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(srdl2comp, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#', [keep(true)]).




%% mng_lookup_transform(+Target, +Source, +TimePoint, -Transform) is nondet.
%
mng_lookup_transform(Target, Source, TimePoint, Transform) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'lookupTransform', [Target, Source, Time], StampedTransform),

  jpl_call(StampedTransform, 'getMatrix4', [], TransformMatrix4d),
  knowrob_coordinates:matrix4d_to_list(TransformMatrix4d, Transform).



%% mng_latest_designator_before_time(+TimePoint, -Type, -Pose) is nondet.
%
mng_latest_designator_before_time(TimePoint, Type, Pose) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'latestUIMAPerceptionBefore', [Time], Designator),

  jpl_call(Designator, 'get', ['type'], Type),

  jpl_call(Designator, 'get', ['pose'], StampedPose),
  jpl_call(StampedPose, 'getMatrix4d', [], PoseMatrix4d),
  knowrob_coordinates:matrix4d_to_list(PoseMatrix4d, Pose).


% read the pose of RobotPart from tf
mng_tf_pose(RobotPart, Pose) :-

  owl_has(RobotPart, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName', literal(SourceFrameID)),
  atom_concat('/', SourceFrameID, SourceFrame),

  % TODO: handle time correctly
  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),

  knowrob_coordinates:list_to_matrix4d([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], MatrixIn),
  jpl_new('ros.communication.Time', [], TimeIn),
  jpl_set(TimeIn, 'secs',  1376484439 ),

  knowrob_coordinates:list_to_matrix4d([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], MatrixOut),
  jpl_new('ros.communication.Time', [], TimeOut),
  jpl_set(TimeOut, 'secs',  1376484439 ),

  jpl_new('tfjava.Stamped', [MatrixIn, SourceFrame, TimeIn], StampedIn),
  jpl_new('tfjava.Stamped', [MatrixOut, '/base_link', TimeOut], StampedOut),

  jpl_call(DB, 'transformPose', ['/odom_combined', StampedIn, StampedOut], _),

  jpl_call(StampedOut, 'getData', [], MatrixOut2),
  knowrob_coordinates:matrix4d_to_list(MatrixOut2, PoseList),
  create_pose(PoseList, Pose),

  create_perception_instance(['Proprioception'], Perception),
  set_object_perception(RobotPart, Perception),
  rdf_assert(Perception, knowrob:eventOccursAt, Pose).





