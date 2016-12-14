/** <module> robcog_mongo_queries

  Copyright (C) 2016 Andrei Haidu

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

  @author Andrei Haidu
  @license BSD
*/

:- module(robcog_mongo_queries,
    [
        connect_to_db/1,
        set_mongo_coll/1,


        actor_pose/3,
        actor_pose/4,
        view_actor_pose/6,
        view_actor_pose/7,

        view_mesh/4,
        view_mesh/5,
        view_bones_meshes/4,
        view_bones_meshes/5,

        actor_traj/6,
        view_actor_traj/8,
        view_actor_traj/9,

        bone_pose/5,
        view_bone_pose/7,
        view_bone_pose/8,

        bone_traj/7,
        view_bone_traj/9,
        view_bone_traj/10,

        bones_names/3,
        bones_poses/4,
        view_bones_poses/6,
        view_bones_poses/7,

        bones_trajs/6,
        view_bones_trajs/8,
        view_bones_trajs/9,

        u_marker_remove/1,
        u_marker_remove_all/0,

        add_rating/4
    ]).

:-  rdf_meta
    u_marker_remove(t),
    add_rating(r, r, +, +).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Database where to read the raw data from
connect_to_db(DBName) :-
    mongo_robcog_conn(MongoRobcog),
    jpl_call(MongoRobcog, 'SetDatabase', [DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Set the collection from which to query
% TODO make sure the DB is set
set_mongo_coll(Coll) :-
    mongo_robcog_conn(MongoRobcog),
    jpl_call(MongoRobcog, 'SetCollection', [Coll], @void).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the pose of the actor at the given timestamp
% Actor = 'LeftHand'
actor_pose(Actor, Ts, Pose) :-
    get_ep(EpInst),
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetActorPoseAt', [Actor, Ts], JavaArr),
    jpl_array_to_list(JavaArr, Pose).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the pose of the actor at the given timestamp
% Actor = 'LeftHand'
actor_pose(EpInst, Actor, Ts, Pose) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetActorPoseAt', [Actor, Ts], JavaArr),
    jpl_array_to_list(JavaArr, Pose).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the pose of the actor at the given timestamp
% Actor = 'LeftHand'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
view_actor_pose(EpInst, Actor, Ts, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorPoseAt',
        [Actor, Ts, MarkerType, Color, Scale], @void).
%%
view_actor_pose(EpInst, Actor, Ts, MarkerID, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorPoseAt',
        [Actor, Ts, MarkerID, MarkerType, Color, Scale], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the pose of the actor at the given timestamp
% Actor = 'LeftHand'
% MeshPath = 'path to the mesh'
% MarkerID = 'hand_marker_id1'
view_mesh(EpInst, Actor, Ts, MeshPath) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorMeshAt',
        [Actor, Ts, MeshPath], @void).
%%
view_mesh(EpInst, Actor, Ts, MarkerID, MeshPath) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorMeshAt',
        [Actor, Ts, MarkerID, MeshPath], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the mesh of the actor's bones at the given timestamp
% Actor = 'LeftHand'
% MeshPath = 'path to the mesh'
% MarkerID = 'hand_marker_id1'
view_bones_meshes(EpInst, Actor, Ts, MeshFolderPath) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesMeshesAt',
        [Actor, Ts, MeshFolderPath], @void).
%%
view_bones_meshes(EpInst, Actor, Ts, MarkerID, MeshFolderPath) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesMeshesAt',
        [Actor, Ts, MarkerID, MeshFolderPath], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the trajectory of actor at between the given timestamps
% Actor = 'LeftHand'
% DT = 0.01 (seconds)
actor_traj(EpInst, Actor, Start, End, DT, Traj) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetActorTraj', [Actor, Start, End, DT], JavaMultiArr),
    jpl_array_to_list(JavaMultiArr, JavaObjList),
    maplist(jpl_array_to_list, JavaObjList, Traj).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View trajectory of actor at between the given timestamps 
% Actor = 'LeftHand'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
% DT = 0.01 (seconds)
view_actor_traj(EpInst, Actor, Start, End, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorTraj',
        [Actor, Start, End, MarkerType, Color, Scale, DT], @void).
%%
view_actor_traj(EpInst, Actor, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewActorTraj',
        [Actor, Start, End, MarkerID, MarkerType, Color, Scale, DT], @void).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the pose of the bone at the given timestamp
% Actor = 'LeftHand'
% Bone = 'index_3_l'
bone_pose(EpInst, Actor, Bone, Ts, Pose) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetBonePoseAt', [Actor, Ts, Bone], JavaArr),
    jpl_array_to_list(JavaArr, Pose).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the pose of the bone at the given timestamp 
% Actor = 'LeftHand'
% Bone = 'index_3_l'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
view_bone_pose(EpInst, Actor, Bone, Ts, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonePose',
        [Actor, Bone, Ts, MarkerType, Color, Scale], @void).
%%
view_bone_pose(EpInst, Actor, Bone, Ts, MarkerID, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonePose',
        [Actor, Bone, Ts, MarkerID, MarkerType, Color, Scale], @void).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the trajectory of the bone at between the given timestamps
% Actor = 'LeftHand'
% Bone = 'index_3_l'
% DT = 0.01 (seconds)
bone_traj(EpInst, Actor, Bone, Start, End, DT, Traj) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetBoneTraj', [Actor, Bone, Start, End, DT], JavaMultiArr),
    jpl_array_to_list(JavaMultiArr, JavaObjList),
    maplist(jpl_array_to_list, JavaObjList, Traj).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View trajectory of the bone at between the given timestamps 
% Actor = 'LeftHand'
% Bone = 'index_3_l'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
% DT = 0.01 (seconds)
view_bone_traj(EpInst, Actor, Bone, Start, End, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBoneTraj', 
        [Actor, Bone, Start, End, MarkerType, Color, Scale, DT], @void).
%%
view_bone_traj(EpInst, Actor, Bone, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBoneTraj', 
        [Actor, Bone, Start, End, MarkerID, MarkerType, Color, Scale, DT], @void).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the names of the actor bones
% Actor = 'LeftHand'
bones_names(EpInst, Actor, Bones) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetBonesNames', [Actor], JavaArr),
    jpl_array_to_list(JavaArr, Bones).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the poses of the actor bones at the given timestamp
% Actor = 'LeftHand'
bones_poses(EpInst, Actor, Ts, Poses) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetBonesPoses', [Actor, Ts], JavaMultiArr),
    jpl_array_to_list(JavaMultiArr, JavaObjList),
    maplist(jpl_array_to_list, JavaObjList, Poses).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the poses of the actor bones at the given timestamp 
% Actor = 'LeftHand'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
view_bones_poses(EpInst, Actor, Ts, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesPoses',
        [Actor, Ts, MarkerType, Color, Scale], @void).
%%
view_bones_poses(EpInst, Actor, Ts, MarkerID, MarkerType, Color, Scale) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesPoses',
        [Actor, Ts, MarkerID, MarkerType, Color, Scale], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the trajectories of the actor bones at between the given timestamps
% Actor = 'LeftHand'
% DT = 0.01 (seconds)
bones_trajs(EpInst, Actor, Start, End, DT, Trajs) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'GetBonesTrajs', [Actor, Start, End, DT], JavaMultiArr),
    jpl_array_to_list(JavaMultiArr, JavaObjList),
    maplist(jpl_array_to_list, JavaObjList, JavaPoseObjs),
    maplist(maplist_arr_to_list, JavaPoseObjs, Trajs).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the poses of the actor bones between the given timestamps 
% Actor = 'LeftHand'
% MarkerID = 'hand_marker_id1'
% MarkerType = 'sphere', 'cube', 'point'
% Color = 'blue'
% Scale = 0.01 (meters)
% DT = 0.01 (seconds)
view_bones_trajs(EpInst, Actor, Start, End, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesTrajs',
        [Actor, Start, End, MarkerType, Color, Scale, DT], @void).
%%
view_bones_trajs(EpInst, Actor, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
    mongo_robcog_query(MongoQuery),
    get_mongo_coll_name(EpInst, CollName),
    set_mongo_coll(CollName),
    jpl_call(MongoQuery, 'ViewBonesTrajs',
        [Actor, Start, End, MarkerID, MarkerType, Color, Scale, DT], @void).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Remove all markers created with the knowrob_robcog package
u_marker_remove(all) :-
    mongo_robcog_query(MongoQuery),
    jpl_call(MongoQuery, 'RemoveAllMarkers', [], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Remove the marker witht he given ID
% MarkerID = 'coll_traj_id'
u_marker_remove(MarkerID) :-
    mongo_robcog_query(MongoQuery),
    jpl_call(MongoQuery, 'RemoveMarker', [MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Remove all markers created with the knowrob_robcog package
u_marker_remove_all :-
    mongo_robcog_query(MongoQuery),
    jpl_call(MongoQuery, 'RemoveAllMarkers', [], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Add rating to file
% check for rating instance
% get file path from the folder and file name
add_rating(EpInst, RatingType, Score, EpisodesPath) :-    
    rdf_has(EpInst, knowrob_u:'rating', RatingInst),
    rdf_has(RatingInst, rdf:type, knowrob_u:'Rating'),
    ep_folder(EpInst, FolderName),
    atom_concat(EpisodesPath, FolderName, FolderPath),
    rating_file(EpInst, FileName),
    atom_concat(FolderPath, FileName, FilePath),
    mongo_robcog_query(MongoQuery),
    jpl_call(MongoQuery, 'AddRating',
        [RatingInst, RatingType, Score, FilePath], @void).
