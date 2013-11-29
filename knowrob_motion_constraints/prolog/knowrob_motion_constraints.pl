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
      object_feature/4,
      motion_constraint/2,
%       motion_constraint/3,
      constraint_properties/7,
      plan_constraints_of_type/3,
      features_in_constraints/2,
      feature_properties/6
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
    object_feature(r, ?, ?, ?),
    plan_constraints_of_type(r,r,r),
    features_in_constraints(r,r),
    motion_constraint(r, r),
%     motion_constraint(r, r, r),
    constraint_properties(r, r, r, r, -, -, -),
    feature_properties(r, -, -, -, -, -).



:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(constr, 'http://ias.cs.tum.edu/kb/motion-constraints.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake_constr, 'http://ias.cs.tum.edu/kb/pancake-making-constr.owl#', [keep(true)]).



%% motion_constraint(+Motion, -C) is nondet.
%
% All constraints defined for the given motion class.
%
% @param Motion OWL identifier for a motion class, e.g. pancake_constr:'BothSpatulasApproach'
% @param Constr OWL identifier of the constraint
%
motion_constraint(Motion, Constr) :-
    class_properties(Motion, knowrob:constrainedBy, Constr).


%% motion_constraint(+Motion, +Tool, -C) is nondet.
%
% All constraints defined for the given motion class and the given Tool.
%
% @param Motion OWL identifier for a motion class, e.g. pancake_constr:'BothSpatulasApproach'
% @param Tool   OWL identifier for a tool, e.g. an instance of a spatula
% @param Constr OWL identifier of the constraint
%
%TODO: ADAPT THIS TO THE FINAL FEATURE REPRESENTATION
% 
% motion_constraint(Motion, Tool, Constr) :-
%     class_properties(Motion, knowrob:constrainedBy, Constr),
%     class_properties(Constr, constr:toolFeature, Tf),
%     once(owl_individual_of(ToolPart, Tf)),
%     owl_has(Tool, knowrob:properPhysicalParts, ToolPart).





%% constraint_properties(+Constr, -Type, -ToolFeature, -WorldFeature, -Weight, -Lower, -Upper, -MaxVel) is nondet.
%
% Convenience predicate: read all properties of a motion constraint with
% one predicate call
%
% @param Constr         OWL identifier of the constraint to be read
% @param Type           OWL class describing the type of constraint (e.g. distance, height)
% @param ToolFeature    OWL identifier of the tool feature (i.e. spatial part of the tool to be controlled)
% @param WorldFeature   OWL identifier of the world feature (i.e. spatial part of the world)
% @param ReferenceFrame Tf frame for the reference frame
% @param Lower          Lower limit for the controlled values
% @param Upper          Upper limit for the controlled values
%
constraint_properties(Constr, Type, ToolFeature, WorldFeature, ReferenceFrame, Lower, Upper) :-

    owl_subclass_of(Constr, Type),
    once(owl_direct_subclass_of(Type, constr:'MotionConstraint')),

    class_properties(Constr, constr:toolFeature, ToolFeatureClass),
    once(owl_individual_of(ToolFeature, ToolFeatureClass)),

    class_properties(Constr, constr:worldFeature, WorldFeatureClass),
    once(owl_individual_of(WorldFeature, WorldFeatureClass)),

    once(class_properties(Constr, constr:refFeature, literal(type(_, ReferenceFrame)));true),

    class_properties(Constr, constr:constrLowerLimit, literal(type(_, L))),
    term_to_atom(Lower, L),

    class_properties(Constr, constr:constrUpperLimit, literal(type(_, U))),
    term_to_atom(Upper, U),!.




%% feature_properties(Feature, Type, Label, TfFrame, Position, Direction, ContactDirection) is nondet.
%
% Convenience predicate: read all properties of a feature with one predicate call
%
% @param Feature           OWL identifier of the feature to be reads
% @param Type              OWL class describing the feature type (e.g. line, plane)
% @param Label             Natural-language label describing this feature
% @param TfFrame           Coordinate frame in which this feature is described
% @param Position          Position vector as list [x,y,z] w.r.t. TfFrame
% @param Direction         Direction vector as list [x,y,z] w.r.t. TfFrame
% @param ContactDirection  ContactDirection vector as list [x,y,z] w.r.t. TfFrame
%

% read properties of 'real' features (manually defined)
feature_properties(Feature, Type, Label, TfFrame, Position, Direction) :-

  rdf_has(Feature, rdf:type, Type),
  owl_subclass_of(Type, knowrob:'ToolFeature'),
  rdf_has(Feature, rdfs:label, literal(type(_,Label))),
  rdf_has(Feature, knowrob:tfFrame, literal(type(_,TfFrame))),

  owl_has(Feature, knowrob:position, Pos),
  owl_has(Pos, knowrob:vectorX, literal(type(_,Px))), term_to_atom(PX, Px),
  owl_has(Pos, knowrob:vectorY, literal(type(_,Py))), term_to_atom(PY, Py),
  owl_has(Pos, knowrob:vectorZ, literal(type(_,Pz))), term_to_atom(PZ, Pz),
  Position = [PX, PY, PZ],

  owl_has(Feature, knowrob:direction, Dir),
  owl_has(Dir, knowrob:vectorX, literal(type(_,Dx))), term_to_atom(DX, Dx),
  owl_has(Dir, knowrob:vectorY, literal(type(_,Dy))), term_to_atom(DY, Dy),
  owl_has(Dir, knowrob:vectorZ, literal(type(_,Dz))), term_to_atom(DZ, Dz),
  Direction = [DX, DY, DZ].


% read cylinders as line features
feature_properties(Feature, Type, Label, TfFrame, Position, Direction) :-

%   owl_individual_of(Feature, FeatureClassDef),

  rdf_has(Feature, rdf:type, T),
  owl_subclass_of(T, knowrob:'Cone'),
  Type = 'http://ias.cs.tum.edu/kb/knowrob.owl#LineFeature',

  (rdf_has(Feature, rdfs:label, literal(type(_,Label))),!; Label=''),
  (rdf_has(Feature, knowrob:tfFrame, literal(type(_,TfFrame))),!; TfFrame = 'map'),

  % todo: use relative pose instead?
  current_object_pose(Feature, [_,_,_,PX,_,_,_,PY,_,_,_,PZ,_,_,_,_]),
  Position = [PX, PY, PZ],

  rdf_triple(knowrob:longitudinalDirection, Feature, Dir),
  owl_has(Dir, knowrob:vectorX, literal(type(_,Dx))), term_to_atom(DX, Dx),
  owl_has(Dir, knowrob:vectorY, literal(type(_,Dy))), term_to_atom(DY, Dy),
  owl_has(Dir, knowrob:vectorZ, literal(type(_,Dz))), term_to_atom(DZ, Dz),
  Direction = [DX, DY, DZ].


% read planes as plane features
feature_properties(Feature, Type, Label, TfFrame, Position, Direction) :-

%   owl_individual_of(Feature, FeatureClassDef),

  rdf_has(Feature, rdf:type, T),
  owl_subclass_of(T, knowrob:'FlatPhysicalSurface'),
  Type = 'http://ias.cs.tum.edu/kb/knowrob.owl#PlaneFeature',

  (rdf_has(Feature, rdfs:label, literal(type(_,Label))),!; Label=''),
  (rdf_has(Feature, knowrob:tfFrame, literal(type(_,TfFrame))),!; TfFrame = 'map'),

  % todo: use relative pose instead?
  current_object_pose(Feature, [_,_,_,PX,_,_,_,PY,_,_,_,PZ,_,_,_,_]),
  Position = [PX, PY, PZ],

  rdf_triple(knowrob:normalDirection, Feature, Dir),
  owl_has(Dir, knowrob:vectorX, literal(type(_,Dx))), term_to_atom(DX, Dx),
  owl_has(Dir, knowrob:vectorY, literal(type(_,Dy))), term_to_atom(DY, Dy),
  owl_has(Dir, knowrob:vectorZ, literal(type(_,Dz))), term_to_atom(DZ, Dz),
  Direction = [DX, DY, DZ].



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% all distance constraints in the task
plan_constraints_of_type(Plan, Type, C) :-
   plan_subevents(Plan, Sub),
   member(Motion, Sub),
   class_properties(Motion, knowrob:constrainedBy, C),
   owl_subclass_of(C, Type).


% all object/feature specs in the task
features_in_constraints(Plan, O) :-
   plan_subevents(Plan, Sub),
   member(Motion, Sub),
   class_properties(Motion, knowrob:constrainedBy, C),
   (class_properties(C, constr:toolFeature, O);
    class_properties(C, constr:worldFeature, O)).

% all properties of that constraint
constraint_property(C, P, O) :-
   class_properties(C, P, O).






% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% local object poses

object_feature(ObjClass, FeatureType, FeaturePos, FeatureDir) :-

    ( (owl_individual_of(ObjInst, ObjClass),!);
      (rdf_instance_from_class(ObjClass, ObjInst))),

    rdf_triple(knowrob:properPhysicalParts, ObjInst, PartInst),

    (
     (sphere_annotation_position(PartInst, FeaturePos, FeatureDir),
       FeatureType = 2) ; % POINT
     (cone_annotation_dir_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 0) ; % LINE
     (plane_annotation_side_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 0) ; % LINE
     (plane_annotation_normal_vector(PartInst, FeaturePos, FeatureDir),
       FeatureType = 1) % PLANE
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
    annotation_pose_list(PartInst, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_]),
    FeaturePos = [X,Y,Z],

    annotation_cone_direction(PartInst, Direction),
    rdf_has(Direction, knowrob:vectorX, literal(type(xsd:'float',DirX))),
    rdf_has(Direction, knowrob:vectorY, literal(type(xsd:'float',DirY))),
    rdf_has(Direction, knowrob:vectorZ, literal(type(xsd:'float',DirZ))),
    FeatureDir = [DirX, DirY, DirZ].



% plane annotation: center point and normal vector
plane_annotation_normal_vector(PartInst, FeaturePos, FeatureDir) :-

    owl_individual_of(PartInst, 'http://ias.cs.tum.edu/kb/knowrob.owl#FlatPhysicalSurface'),
    annotation_pose_list(PartInst, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_]),
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
    annotation_pose_list(J, PartPose1),
    PartPose1 = [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_],
    PartPose = [X,Y,Z],
    knowrob_coordinates:list_to_vector3d(PartPose, PartPoseVec),

    compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList).



% compute front edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [LongSideVec], SidePosVec),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(ShortSideVec, SideDirList),
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).

% compute rear edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [LongSideVec], SidePosVec),
    jpl_call(SidePosVec, negate, [], _),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(ShortSideVec, SideDirList),
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).


% compute right edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [ShortSideVec], SidePosVec),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(LongSideVec, SideDirList),
    knowrob_coordinates:vector3d_to_list(SidePosVec, SidePosList).

% compute left edge
compute_edge_vector(PartPoseVec, LongSideVec, ShortSideVec, SidePosList, SideDirList) :-

    jpl_new('javax.vecmath.Vector3d', [ShortSideVec], SidePosVec),
    jpl_call(SidePosVec, negate, [], _),
    jpl_call(SidePosVec, scaleAdd, [0.5, PartPoseVec], _),

    knowrob_coordinates:vector3d_to_list(LongSideVec, SideDirList),
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





/*
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
*/


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % Visualization
%
% instantiate_and_visualize_cylinder(SideCenterVec, ShortSideGlList) :-
%     % create cylinder at center of edge
%     knowrob_coordinates:vector3d_to_list(SideCenterVec, SideVecPosList),
%     SideVecPosList=[PX,PY,PZ],
%     create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder',
%                             [1,0,0,PX,0,1,0,PY,0,0,1,PZ,0,0,0,1],
%                             ['VisualPerception'], Cyl),
%
%
%     ShortSideGlList = [VX, VY, VZ],
%     rdf_instance_from_class(knowrob:'Vector', SideVecDir),
%     rdf_assert(SideVecDir, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
%     rdf_assert(SideVecDir, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
%     rdf_assert(SideVecDir, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))),
%
%     rdf_assert(Cyl, knowrob:longitudinalDirection, SideVecDir),
%     add_object(Cyl, _).
%
%




