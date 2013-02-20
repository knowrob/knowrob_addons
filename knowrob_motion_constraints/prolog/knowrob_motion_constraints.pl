%%
%% Copyright (C) 2013 by Moritz Tenorth
%%
%% This module provides methods for representing and reasoning about
%% constraint-based motion specifications in KnowRob.
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

:- module(knowrob_motion_constraints,
    [
      plane_annotation_side_vector/3,
      print_annotation_normal_vectors/1,
      generate_obj_parts/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_mesh_reasoning')).

:- owl_parser:owl_parse('../owl/spatula-test.owl', false, false, true).


:-  rdf_meta
    print_annotation_normal_vectors(r),
    generate_obj_parts(r, -), 
    plane_annotation_side_vector(r,-,-).
%     storagePlaceForBecause(r,r,r),
%     current_object_pose(r,-).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


generate_obj_parts(Obj, Parts) :-
  findall(Part, comp_physical_parts(Obj, Part), Parts).


% comp_physical_parts(knowrob:'Spatula_OdhJed', Parts).

% current_object_pose(knowrob:'Spatula_OdhJed', SpPose).

% add_object(knowrob:'Spatula_OdhJed',_).


print_annotation_normal_vectors(Obj) :-

    current_object_pose(Obj, SpPose),

    % get all normal vectors
    annotation_plane_normal(PartInst, NormalVec), 

    % read relative poses, and transform to global coordinates
    knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J), 
    annotation_pose_list(J, Pose), 
    pose_into_global_coord(SpPose, Pose, PoseGl), 
    PoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_], 

    % create a cylinder instance for visualization in direction of the 
    % normal vector
    create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', 
                             [1,0,0,X,0,1,0,Y,0,0,1,Z,0,0,0,1],  
                             ['VisualPerception'], Cyl),  
    rdf_assert(Cyl, knowrob:longitudinalDirection, NormalVec), 
    add_object(Cyl, _), highlight_object(Cyl, _).


% create side vector for plane annotation
% TODO: take orientation of spatula into account -> rotate vectors

% short edges
plane_annotation_side_vector(Obj, SideVecPos, SideVecDir) :-

  current_object_pose(Obj, SpPose),
  annotation_plane_shortside(PartInst, SideVecDir),
  annotation_plane_longside(PartInst, LongSide),
  knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J), 

  annotation_pose_list(J, Pose),
  pose_into_global_coord(SpPose, Pose, PoseGl),
  PoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_],

  knowrob_coordinates:list_to_vector3d([X,Y,Z], PlaneCenter),
  rdf_has(LongSide, knowrob:vectorX, literal(type(xsd:'float',LongSideX))),
  rdf_has(LongSide, knowrob:vectorY, literal(type(xsd:'float',LongSideY))),
  rdf_has(LongSide, knowrob:vectorZ, literal(type(xsd:'float',LongSideZ))),

  knowrob_coordinates:list_to_vector3d([LongSideX,LongSideY,LongSideZ], DirVec),
  jpl_new('javax.vecmath.Vector3d', [DirVec], SideCenterVec),
  jpl_call(SideCenterVec, scaleAdd, [0.5, PlaneCenter], _),

  knowrob_coordinates:vector3d_to_list(SideCenterVec, SideVecPos), 
  SideVecPos=[PX,PY,PZ], 
  create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', 
                           [1,0,0,PX,0,1,0,PY,0,0,1,PZ,0,0,0,1],  
                           ['VisualPerception'], Cyl),

  rdf_assert(Cyl, knowrob:longitudinalDirection, SideVecDir),
  add_object(Cyl, _).

