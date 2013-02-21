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
      generate_obj_parts/2,
      object_feature/4
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
    plane_annotation_side_vector(r,-,-),
    object_feature(r, ?, ?, ?).
%     storagePlaceForBecause(r,r,r),
%     current_object_pose(r,-).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% local object poses

object_feature(ObjClass, FeatureType, FeaturePos, FeatureDir) :-

    ( (owl_individual_of(ObjInst, ObjClass),!);
      (rdf_instance_from_class(ObjClass, ObjInst))),

    comp_physical_parts(ObjInst, PartInst),

    (
     (sphere_annotation_position(PartInst, FeaturePos, FeatureDir),
       FeatureType = 2) % POINT
     (cone_annotation_dir_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 0) ; % LINE
     (plane_annotation_side_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 0) ; % LINE
     (plane_annotation_normal_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 1) ; % PLANE
    ).



% point features for center point of sphere annotations
sphere_annotation_position(PartInst, FeaturePos, FeatureDir) :-
    owl_individual_of(PartInst, 'http://ias.cs.tum.edu/kb/knowrob.owl#Sphere'),
    annotation_pose_list(PartInst, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_]),
    FeaturePos = [X,Y,Z],
    FeatureDir = [0,0,0].

% line feature for center line of cone annotations
cone_annotation_dir_vector(PartInst, FeaturePos, FeatureDir) :-

    owl_individual_of(PartInst, 'http://ias.cs.tum.edu/kb/knowrob.owl#Cone'),
    annotation_pose_list(PartInst, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_])
    FeaturePos = [X,Y,Z],

    annotation_cone_direction(PartInst, Direction),
    rdf_has(Direction, knowrob:vectorX, literal(type(xsd:'float',DirX))),
    rdf_has(Direction, knowrob:vectorY, literal(type(xsd:'float',DirY))),
    rdf_has(Direction, knowrob:vectorZ, literal(type(xsd:'float',DirZ))),
    FeatureDir = [DirX, DirY, DirZ].



% plane annotation: center point and normal vector
plane_annotation_normal_vector(PartInst, FeaturePos, FeatureDir) :-

    owl_individual_of(PartInst, 'http://ias.cs.tum.edu/kb/knowrob.owl#FlatPhysicalSurface'),
    annotation_pose_list(PartInst, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_])
    FeaturePos = [X,Y,Z],

    annotation_plane_normal(PartInst, Direction),
    rdf_has(Direction, knowrob:vectorX, literal(type(xsd:'float',DirX))),
    rdf_has(Direction, knowrob:vectorY, literal(type(xsd:'float',DirY))),
    rdf_has(Direction, knowrob:vectorZ, literal(type(xsd:'float',DirZ))),
    FeatureDir = [DirX, DirY, DirZ].




% direction vectors for all edges of a plane annotation

% short edges
plane_annotation_side_vector(PartInst, SidePosList, SideDirList) :-

    % read direction of short side and long side of the part
    annotation_plane_longside(PartInst, LongSideInst),
    rdf_has(LongSideInst, knowrob:vectorX, literal(type(xsd:'float',LongSideX))),
    rdf_has(LongSideInst, knowrob:vectorY, literal(type(xsd:'float',LongSideY))),
    rdf_has(LongSideInst, knowrob:vectorZ, literal(type(xsd:'float',LongSideZ))),
    knowrob_coordinates:list_to_vector3d([LongSideX,LongSideY,LongSideZ], LongSideVec),

    annotation_plane_shortside(PartInst, ShortSideInst),
    rdf_has(ShortSideInst, knowrob:vectorX, literal(type(xsd:'float',ShortSideX))),
    rdf_has(ShortSideInst, knowrob:vectorY, literal(type(xsd:'float',ShortSideY))),
    rdf_has(ShortSideInst, knowrob:vectorZ, literal(type(xsd:'float',ShortSideZ))),
    knowrob_coordinates:list_to_vector3d([ShortSideX,ShortSideY,ShortSideZ], ShortSideVec),

    % read pose of object part and transform to global coordinates
    knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J),
    annotation_pose_list(J, PartPose),
    knowrob_coordinates:list_to_vector3d(PartPose, PartPoseVec),

    compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList).



% compute front edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [LongSideVec], SidePosVec),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(ShortSideVec, SideDirList)
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).

% compute rear edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [LongSideVec], SidePosVec),
    jpl_call(SidePosVec, negate, [], _),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(ShortSideVec, SideDirList)
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).


% compute right edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [ShortSideVec], SidePosVec),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(LongSideVec, SideDirList)
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).

% compute left edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [ShortSideVec], SidePosVec),
    jpl_call(SidePosVec, negate, [], _),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(LongSideVec, SideDirList)
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% global position stuff


generate_obj_parts(Obj, Parts) :-
  findall(Part, comp_physical_parts(Obj, Part), Parts).


% comp_physical_parts(knowrob:'Spatula_OdhJed', Parts).

% current_object_pose(knowrob:'Spatula_OdhJed', ObjPose).

% add_object(knowrob:'Spatula_OdhJed',_).




print_annotation_normal_vectors(Obj) :-

    current_object_pose(Obj, ObjPose),

    % get all normal vectors
    annotation_plane_normal(PartInst, NormalVec), 

    % read relative poses, and transform to global coordinates
    knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J), 
    annotation_pose_list(J, Pose), 
    pose_into_global_coord(ObjPose, Pose, PoseGl), 
    PoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_], 

    % create a cylinder instance for visualization in direction of the 
    % normal vector
    create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', 
                             [1,0,0,X,0,1,0,Y,0,0,1,Z,0,0,0,1],  
                             ['VisualPerception'], Cyl),  
    rdf_assert(Cyl, knowrob:longitudinalDirection, NormalVec), 
    add_object(Cyl, _), highlight_object(Cyl, _).






% create side vector for plane annotation

% short edges
plane_annotation_side_vector(Obj, SideVecPosList, ShortSideGlList) :-

    % read pose of object center
    current_object_pose(Obj, ObjPose),
    knowrob_coordinates:list_to_matrix4d(ObjPose, SpMat4),
    jpl_new('javax.vecmath.Matrix3d', [], ObjOrientation),
    jpl_call(SpMat4, getRotationScale, [ObjOrientation], _),


    % read direction of short side and long side of the part
    annotation_plane_longside(PartInst, LongSideInst),
    rdf_has(LongSideInst, knowrob:vectorX, literal(type(xsd:'float',LongSideX))),
    rdf_has(LongSideInst, knowrob:vectorY, literal(type(xsd:'float',LongSideY))),
    rdf_has(LongSideInst, knowrob:vectorZ, literal(type(xsd:'float',LongSideZ))),
    knowrob_coordinates:list_to_vector3d([LongSideX,LongSideY,LongSideZ], LongSideVec),

    % transform longside vector with object main orientation
    jpl_call(ObjOrientation, transform, [LongSideVec], _),
%     knowrob_coordinates:vector3d_to_list(LongSideVec, LongSideGlList),

    annotation_plane_shortside(PartInst, ShortSideInst),
    rdf_has(ShortSideInst, knowrob:vectorX, literal(type(xsd:'float',ShortSideX))),
    rdf_has(ShortSideInst, knowrob:vectorY, literal(type(xsd:'float',ShortSideY))),
    rdf_has(ShortSideInst, knowrob:vectorZ, literal(type(xsd:'float',ShortSideZ))),
    knowrob_coordinates:list_to_vector3d([ShortSideX,ShortSideY,ShortSideZ], ShortSideVec),

    % transform direction vector with object main orientation
    jpl_call(ObjOrientation, transform, [ShortSideVec], _),
    knowrob_coordinates:vector3d_to_list(ShortSideVec, ShortSideGlList),

    % read pose of object part and transform to global coordinates
    knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J),
    annotation_pose_list(J, PartPose),
    pose_into_global_coord(ObjPose, PartPose, PartPoseGl),
    PartPoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_],
    knowrob_coordinates:list_to_vector3d([X,Y,Z], PartPoseGlVec),



    % compute position at the center of the edge of the plane:
    % == PartPoseGlVec + 0.5 * LongSideVec
    jpl_new('javax.vecmath.Vector3d', [LongSideVec], SideCenterVec),
    jpl_call(SideCenterVec, scaleAdd, [0.5, PartPoseGlVec], _),



    instantiate_and_visualize_cylinder(SideCenterVec, ShortSideGlList).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Visualization

instantiate_and_visualize_cylinder(SideCenterVec, ShortSideGlList) :-
    % create cylinder at center of edge
    knowrob_coordinates:vector3d_to_list(SideCenterVec, SideVecPosList), 
    SideVecPosList=[PX,PY,PZ], 
    create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', 
                            [1,0,0,PX,0,1,0,PY,0,0,1,PZ,0,0,0,1],  
                            ['VisualPerception'], Cyl),


    ShortSideGlList = [VX, VY, VZ],
    rdf_instance_from_class(knowrob:'Vector', SideVecDir),
    rdf_assert(SideVecDir, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
    rdf_assert(SideVecDir, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
    rdf_assert(SideVecDir, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))),

    rdf_assert(Cyl, knowrob:longitudinalDirection, SideVecDir),
    add_object(Cyl, _).






