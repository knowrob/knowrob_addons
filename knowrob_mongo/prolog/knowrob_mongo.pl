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
      mng_object_pose/2,
      mng_lookup_transform/4
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


% :-  rdf_meta
%     plan_constraint_templates(r,r).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(srdl2comp, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#', [keep(true)]).

%% mng_object_pose(+Obj, -Pose) is nondet.
%
% Compute object poses from mongo DB log data
%
% @param Obj OWL identifier of an object instance
% @param Pose Concatenation X-Y of pose coordinates
%
mng_object_pose(Obj, X-Y) :-

  owl_has(Obj, srdl2comp:urdfName, literal(UrdfName)),
  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'getPose', [UrdfName], PoseArray),
  jpl_array_to_list(PoseArray, [X,Y]).



mng_lookup_transform(Target, Source, TimePoint, Transform) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'lookupTransform', [Target, Source, Time], StampedTransform),

  jpl_call(StampedTransform, 'getMatrix4', [], TransformMatrix4d),
  knowrob_coordinates:matrix4d_to_list(TransformMatrix4d, Transform).



mng_designator(TimePoint, Type, Pose) :-

  rdf_split_url(_, TimePointLocal, TimePoint),
  atom_concat('timepoint_', TimeAtom, TimePointLocal),
  term_to_atom(Time, TimeAtom),

  jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], DB),
  jpl_call(DB, 'latestUIMAPerceptionBefore', [Time], Designator),

  jpl_call(Designator, 'get', ['type'], Type),
  
  jpl_call(Designator, 'get', ['pose'], StampedPose),
  jpl_call(StampedPose, 'getMatrix4d', [], PoseMatrix4d),
  knowrob_coordinates:matrix4d_to_list(PoseMatrix4d, Pose).


