
/** <module> knowrob_robcog

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

:- module(robcog_test_queries,
    [
    u_export/1,
    grasp_bowl/4,
    grasp_spatula/4,
    u_load/0,
    u_furniture_test/0,
    u_test/0,
    u_test2/0,
    term/1
    ]).


u_export(File) :-
  open(File, write, Stream, [encoding('ascii')]),
  rdf_save_header(Stream, [write_xml_base(true)]),
  rdf_save_footer(Stream),
  close(Stream).

%%
grasp_bowl(ExpInst, EventInst, Start, End) :-	
  % get events which occurred in the episodes
  u_occurs(ExpInst, EventInst, Start, End),
  % check for grasping events
  event_type(EventInst, knowrob:'GraspingSomething'),	
  % check object acted on
  acted_on(EventInst, knowrob:'Bowl').

%%
grasp_spatula(ExpInst, EventInst, Start, End) :-	
  % get events which occurred in the episodes
  u_occurs(ExpInst, EventInst, Start, End),
  % check for grasping events
  event_type(EventInst, knowrob:'GraspingSomething'),	
  % check object acted on
  acted_on(EventInst, knowrob:'Spatula').


arr_to_list_maplist(Objs, Trajs) :-
	maplist(jpl_array_to_list, Objs, Trajs).

%%
u_load :-
  % load all episodes    
  u_load_episodes('/home/haidu/TempLogs/RobCoG/rcg_4/Episodes'),
  
  % load semantic map
  owl_parse('/home/haidu/TempLogs/RobCoG/rcg_4/SemanticMap.owl'),

  % connect to the raw data 
  connect_to_db('RobCoG').

%%
u_furniture_test :-
  ep_inst(EpInst),
  % get events which occurred in the episodes
  u_occurs(EpInst, EventInst, Start, End),
  % check for open furniture event
  event_type(EventInst, knowrob_u:'FurnitureStateClosed'), 
  % check object acted on
  acted_on(EventInst, knowrob:'FridgeDrawer').

%%
u_test2 :-
    ep_inst(EpInst),
    % get events which occurred in the episodes
    u_occurs(EpInst, EventInst, Start, End),
    writeln(Start), writeln(End),
    % check for grasping events
    event_type(EventInst, knowrob:'GraspingSomething'), 
    % check object acted on
    acted_on(EventInst, knowrob:'Bowl').

%%
u_test :-

    %% % get the instance of the current episode
    ep_inst(EpInst),

    % semantic map instance
    sem_map_inst(MapInst),

    % db collection names
    get_mongo_coll_name(EpInst, Coll),

    %% % get the hand instance
    %% rdf_has(HandInst, rdf:type, knowrob:'LeftHand'),

    %% % get name withoug the namespace
    %% iri_xml_namespace(HandInst, _, HandShortName),    

    u_inst_name(knowrob:'LeftHand', HandShortName),
    writeln(HandShortName),

    % get the bone names
    bones_names(EpInst, HandShortName, Bones),

    writeln(Bones),

    % get episode tag
    ep_tag(EpInst, EpTag),
    write('** Episode tag: '), write(EpTag), nl.

    %% obj_type(ObjInst,  knowrob:'Bowl'),
    %% rdf_has(ObjInst, rdf:type, ObjType).

    %% % get events which occurred in the episodes
    %% u_occurs(EpInst, GraspEventInst, GraspBowlStart, GraspBowlEnd),
    %% % check for grasping events
    %% rdf_has(GraspEventInst, rdf:type, knowrob:'GraspingSomething'),
    %% % check object acted on
    %% rdf_has(GraspEventInst, knowrob:'objectActedOn', ObjActedOnInst),
    %% % check object type
    %% obj_type(ObjActedOnInst, knowrob:'Bowl'),

    %% % get name withoug the namespace
    %% iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
    %% writeln(ObjShortName),



    %actor_traj(EpInst, 'LeftHand', GraspBowlStart, GraspBowlEnd, 0.1, ActTraj).
    %actor_traj(EpInst, 'LeftHand', 'timepoint_114.054411', 'timepoint_119.054411', 0.1, ActTraj).
    %view_actor_pose(EpInst, 'LeftHand', GraspBowlStart, 'cube', 'green', 0.1).
    %view_actor_traj(EpInst, 'LeftHand', GraspBowlStart, GraspBowlEnd, 'h_id1', 'point', 'red', 0.01, 0.01).
    %view_mesh(EpInst, ObjShortName, GraspBowlStart, 'pipatm_id7', 'package://sim/robcog/Bowl.dae').
    %view_mesh(EpInst, ObjShortName, GraspBowlStart, 'pipatm_id7', 'package://sim/robcog/LeftHand/pinky_1_l.dae').
    %view_bones_meshes(EpInst, 'LeftHand', GraspBowlStart, 'pipatm_id8', 'package://sim/robcog/LeftHand/').
    %view_bones_meshes(EpInst, 'RightHand', GraspBowlStart, 'pipatm_id7', 'package://sim/robcog/RightHand/').

    %bone_traj(EpInst, 'LeftHand', 'pinky_1_l', GraspBowlStart, GraspBowlEnd, 0.1, BoneTraj).
    %bone_traj(EpInst, 'LeftHand', 'pinky_1_l', 'timepoint_114.054411', 'timepoint_119.054411', 0.1, BoneTraj).
    %view_bone_traj(EpInst, 'LeftHand','pinky_1_l', GraspBowlStart, GraspBowlEnd, 'h_id15', 'point', 'green', 0.01, 0.01).

    %bones_trajs(EpInst, 'LeftHand', GraspBowlStart, GraspBowlEnd, 0.5, BonesTrajs).
    %bones_trajs(EpInst, 'LeftHand', 'timepoint_114.054411', 'timepoint_119.054411', 0.5, BonesTrajs).
    %view_bones_trajs(EpInst, 'RightHand', GraspBowlStart, GraspBowlEnd, 'h_id12', 'point', 'red', 0.01, 0.01).



    % marker_update(object('http://knowrob.org/kb/u_map.owl#USemMap_XkrJ')).

    % u_marker_remove_all.


term(type1) :-
  writeln('Type1').
  
term(type2) :-
  writeln('Type2').