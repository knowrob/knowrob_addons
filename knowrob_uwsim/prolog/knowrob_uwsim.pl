/** <module> knowrob_uwsim

  Copyright (C) 2013 by Asil Kaan Bozcuoglu, Moritz Tenorth, Daniel Beßler

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

@author Asil Kaan Bozcuoglu, Moritz Tenorth, Daniel Beßler
@license BSD
*/


:- module(knowrob_uwsim,
    [
	display_trajectory_of_redsharkcollfish/1,
	display_trajectory_of_redfish/1,
	display_redfish_shark/1,
    	display_fish_reaction/1,
	display_blue_shark_collision/1,
	display_red_green_yellow_collision/1,
	display_red_collision/1,
	display_green_collision/1,
	display_yellow_fish/1,
	display_shark_fishhome/1,
	display_fishhome_image/1,
	display_starfish_and_image/1,
	remove_all_fishesinhome/0,
        display_all_fishpos_panorama/1,
	remove_all_fishes/0,
 	display_pose_uw_crab/1,
	display_uwvehicle_traj/1,
	display_panorama_traj/0,
	display_picture_acq_points/0,
	display_firstthreepoints/0,
	display_secondthreepoints/0,
	display_thirdthreepoints/0,
	display_lastthreepoints/0,
	panaroma_picture_display/0,
	display_combinedimage/0,
	display_trajectory_of_all_fishes/1,
	display_trajectory_bluishgreen_fish/1,
	display_trajectory_of_greenfish/1,
	display_trajectory_of_yellowfish/1,
	display_trajectory_of_bluefish/1,
	display_trajectory_of_shark/1,
	clear_objects/0,
	display_image_acquistion_points/0,
	display_first_setpoints/0,
	display_second_setpoints/0,
	display_third_setpoints/0,
	display_forth_setpoints/0,
	display_first1st_subsetpoints/0,
	display_first2nd_subsetpoints/0,
	display_first3rd_subsetpoints/0,
	display_first4th_subsetpoints/0,
	display_second1st_subsetpoints/0,
	display_second2nd_subsetpoints/0,
	display_second3rd_subsetpoints/0,
	display_second4th_subsetpoints/0,
	display_third1st_subsetpoints/0,
	display_third2nd_subsetpoints/0,
	display_third3rd_subsetpoints/0,
	display_third4th_subsetpoints/0,
	display_forth1st_subsetpoints/0,
	display_forth2nd_subsetpoints/0,
	display_forth3rd_subsetpoints/0,
	clear_all_trajectories_objects/0,
	highlight_images_acq_points/0,
	display_couple_imagesacquired_obj/1,
	retrieve_positions_and_addimages/0,
	retrieve_positions_and_addimages_firstsetdata/0,
	retrieve_positions_and_addimages_secondsetdata/0,
	retrieve_positions_and_addimages_thirdsetdata/0,
	retrieve_positions_and_addimages_lastsetdata/0
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_meshes')).
:- use_module(library('srdl2')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_uwsim, 'http://knowrob.org/kb/knowrob_uwsim.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishone, 'http://knowrob.org/kb/Fish1.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishtwo, 'http://knowrob.org/kb/Fish2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishthree, 'http://knowrob.org/kb/Fish3.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishfour, 'http://knowrob.org/kb/Fish4.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishfive, 'http://knowrob.org/kb/Fish5.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishsix, 'http://knowrob.org/kb/Fish6.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishseven, 'http://knowrob.org/kb/Fish7.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fisheight, 'http://knowrob.org/kb/Fish8.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishnine, 'http://knowrob.org/kb/Fish9.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(sharkone, 'http://knowrob.org/kb/Shark1.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(uwvehicle, 'http://knowrob.org/kb/UWVehicle.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(fishswarm, 'http://knowrob.org/kb/Fishswarm.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(tortoise, 'http://knowrob.org/kb/Tortoise.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(uwvehicle2, 'http://knowrob.org/kb/UWVehicle2.owl#', [keep(true)]).




% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta
    display_trajectory_of_redsharkcollfish(r),
    display_redfish_shark(r),
    display_shark_fishhome(r),
    display_fishhome_image(r),
    display_pose_uw_crab(r),
    display_all_fishpos_panorama(r),
    display_blue_shark_collision(r),
    display_starfish_and_image(r),
    display_fish_reaction(r),
    display_red_green_yellow_collision(r),
    display_red_collision(r),
    display_green_collision(r),
    display_uwvehicle_traj(r),
    display_yellow_fish(r),
    display_trajectory_of_redfish(r),
    display_trajectory_of_all_fishes(r),
    display_trajectory_bluishgreen_fish(r),
    display_trajectory_of_greenfish(r),
    display_trajectory_of_yellowfish(r),
    display_trajectory_of_bluefish(r),
    display_trajectory_of_shark(r),
    display_couple_imagesacquired_obj(r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_starfish_and_image(Perceive):-
	rdf_has(Perceive,knowrob:'capturedImage',Image),rdf_has(Image,knowrob:'rosTopic',literal('/uwsim/starfish')),once(designator_publish_image(Perceive)),camera_pose([-40,30,-80],[0.5,0.5,0.0,0.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_trajectory_of_redfish(ActionID):-
	task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Fish2/base_link'), interval(S, E,dt(1.0))),marker_scale(trajectory('/Fish2/base_link'),[1.5,1.5,1.5]), marker_highlight(trajectory('/Fish2/base_link'),	
		[1.0,0.0,0.0,1.0]),
	marker_update(trajectory('/Fish2/base_link'), interval(S, E,dt(1.0))),marker_scale(trajectory('/Fish2/base_link'),[1.5,1.5,1.5]), marker_highlight(trajectory('/Fish2/base_link'),
		[1.0,0.0,0.0,1.0]).

display_redfish_shark(ActionID):-
	task_goal(T, ActionID),task_start(T, S), task_end(T, E),marker_update(agent(fishtwo:'fish2_robot'),E),marker_update(agent(sharkone:'shark1_robot'),E).

display_trajectory_of_redsharkcollfish(ActionID):-
	display_trajectory_of_redfish(ActionID),
	display_redfish_shark(ActionID),
	camera_pose([-180,-150,-80],[0,0.5,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_shark_fishhome(ActionID):-
	task_goal(B, ActionID), rdf_has(B, knowrob:'startTime', T2),marker_update(agent(sharkone:'shark1_robot'), T2), marker_update(agent(fishseven:'fish7_robot'), T2),marker_update(agent(fisheight:'fish8_robot'), T2),marker_update(agent(fishnine:'fish9_robot'), T2),camera_pose([-215,-45,-30],[0.5,0.5,0.0,0.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_blue_shark_collision(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E),
	marker_update(trajectory('/Fish5/base_link'), interval(S,E, dt(0.5))),marker_update(agent(fishfive:'fish5_robot'),E),marker_update(agent(sharkone:'shark1_robot'),S),
		marker_scale(trajectory('/Fish5/base_link'),[1.5,1.5,1.5]),
	marker_update(trajectory('/Fish5/base_link'), interval(S, E, dt(0.5))),marker_update(agent(fishfive:'fish5_robot'),E),marker_update(agent(sharkone:'shark1_robot'),S),
		marker_scale(trajectory('/Fish5/base_link'),[1.5,1.5,1.5]),
	camera_pose([-80.0,0.0,-40.0],[-0.8,0.3,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_red_collision(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E),
	marker_update(trajectory('/Fish2/base_link'), interval(S,E, dt(0.5))),marker_update(agent(fishtwo:'fish2_robot'),E),marker_color(trajectory('/Fish2/base_link'),[0.5,0.5,1.0,1.0]),marker_scale(trajectory('/Fish2/base_link'),[1.5,1.5,1.5]),
	marker_update(trajectory('/Fish2/base_link'), interval(S,E, dt(0.5))),marker_update(agent(fishtwo:'fish2_robot'),E),marker_color(trajectory('/Fish2/base_link'),[0.5,0.5,1.0,1.0]),marker_scale(trajectory('/Fish2/base_link'),[1.5,1.5,1.5]).

display_green_collision(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E),
	marker_update(trajectory('/Fish3/base_link'), interval(S,E, dt(0.5))),marker_update(agent(fishthree:'fish3_robot'),E),marker_color(trajectory('/Fish3/base_link'),[1.0,0.5,0.5,1.0]),marker_scale(trajectory('/Fish3/base_link'),[1.5,1.5,1.5]),
	marker_update(trajectory('/Fish3/base_link'), interval(S,E, dt(0.5))),marker_update(agent(fishthree:'fish3_robot'),E),marker_color(trajectory('/Fish3/base_link'),[1.0,0.5,0.5,1.0]),marker_scale(trajectory('/Fish3/base_link'),[1.5,1.5,1.5]).

display_yellow_fish(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E),marker_update(agent(fishfour:'fish4_robot'),E),camera_pose([-180,-150,-50],[0.0,0.5,0.5,0.0]).

display_fish_reaction(ActionID):-
	task_goal(B, ActionID), rdf_has(B, knowrob:'startTime', T2),marker_update(agent(sharkone:'shark1_robot'), T2), marker_update(agent(fishseven:'fish7_robot'), T2),marker_update(agent(fisheight:'fish8_robot'), T2),marker_update(agent(fishnine:'fish9_robot'), T2),camera_pose([-220,-150,-80],[0.0,0.5,0.5,0.0]).

display_red_green_yellow_collision(ActionID):-
	display_red_collision(ActionID),
	display_green_collision(ActionID),
	display_yellow_fish(ActionID).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_fishhome_image(ActionID):-
	task_goal(B, ActionID), rdf_has(B, knowrob:'startTime', T2),marker_update(agent(fishsix:'fish6_robot'), T2), marker_update(agent(fishseven:'fish7_robot'), T2),marker_update(agent(fisheight:'fish8_robot'), T2),marker_update(agent(fishnine:'fish9_robot'), T2),rdf_has(B,knowrob:'capturedImage',Image),once(designator_publish_image(B)),camera_pose([-235,-100,-110],[0.0,0.5,0.5,0.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
remove_all_fishesinhome:-
	marker_remove(agent(sharkone:'shark1_robot')),marker_remove(agent(fishsix:'fish6_robot')),marker_remove(agent(fishseven:'fish7_robot')),marker_remove(agent(fisheight:'fish8_robot')),marker_remove(agent(fishnine:'fish9_robot')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_all_fishpos_panorama(ActionID):-
	task_goal(A, ActionID), rdf_has(A, knowrob:'startTime', T), marker_update(agent(fishfive:'fish5_robot'), T),marker_update(agent(fishtwo:'fish2_robot'), T),marker_update(agent(fishthree:'fish3_robot'), T),marker_update(agent(fishfour:'fish4_robot'), T),marker_update(agent(sharkone:'shark1_robot'), T),marker_update(agent(fishone:'fish1_robot'), T),camera_pose([-100,-150,-30],[0,0.5,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
remove_all_fishes:-
	marker_remove(agent(sharkone:'shark1_robot')),marker_remove(agent(fishone:'fish1_robot')),marker_remove(agent(fishtwo:'fish2_robot')),marker_remove(agent(fishthree:'fish3_robot')),marker_remove(agent(fishfour:'fish4_robot')),marker_remove(agent(fishfive:'fish5_robot')).

display_pose_uw_crab(ActionID):-
	task_goal(A, ActionID), rdf_has(A, knowrob:'startTime', T), marker_update(agent(uwvehicle:'Nessie_sample_robot'), T),camera_pose([-50,-230,-100],[0,0.5,0.5,0]),rdf_has(Perceive,knowrob:'capturedImage',Image),rdf_has(Image,knowrob:'rosTopic',literal('/uwsim/crabImage')),once(designator_publish_image(Perceive)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_uwvehicle_traj(ActionID):-
	task(T), task_goal(T,ActionID),task_start(T, S), task_end(T, E),
	!,marker_update(trajectory('/UWVehicle/base_link'),interval(S, E, dt(1.0))),!,marker_scale(trajectory('/UWVehicle/base_link'),[2.5,2.5,2.5]),!,marker_update(trajectory('/UWVehicle/base_link'), interval(S, E, dt(1.0))),!,marker_scale(trajectory('/UWVehicle/base_link'),[2.5,2.5,2.5]),!,marker_update(agent(uwvehicle2:'Nessie_sample_robot2'), S),camera_pose([-100.0,-320.0,60.0],[-0.1,0.5,0.5,0]),
	!,marker_update(trajectory('/UWVehicle/base_link'),interval(S, E, dt(1.0))),!,marker_scale(trajectory('/UWVehicle/base_link'),[2.5,2.5,2.5]),!,marker_update(trajectory('/UWVehicle/base_link'), interval(S, E, dt(1.0))),!,marker_scale(trajectory('/UWVehicle/base_link'),[2.5,2.5,2.5]),!,marker_update(agent(uwvehicle2:'Nessie_sample_robot2'), S),camera_pose([-100.0,-320.0,60.0],[-0.1,0.5,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_panorama_traj :-
	display_picture_acq_points.

display_picture_acq_points :-
	display_firstthreepoints,
	display_secondthreepoints,
	display_thirdthreepoints,
	display_lastthreepoints.

display_firstthreepoints :-
	marker(trajectory('/UWVehicle/base_link'), M1, 'Marker1'),
	task(T1), task_goal(T1, 'PanoromicMotionRecording1'),task_start(T1, S1), task_end(T1, E1), marker_update(M1, interval(S1, E1, dt(2.0))),marker_scale(M1,[5.5,1.5,4.5]),marker_color(M1,[1.0,0.0,0.0,1.0]),marker_update(M1,interval(S1, E1, dt(2.0))),
	task(T1), task_goal(T1, 'PanoromicMotionRecording1'),task_start(T1, S1), task_end(T1, E1), marker_update(M1, interval(S1, E1, dt(2.0))),marker_scale(M1,[5.5,1.5,4.5]),marker_color(M1,[1.0,0.0,0.0,1.0]),marker_update(M1,interval(S1, E1, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M2, 'Marker2'),
	task(T2), task_goal(T2, 'PanoromicMotionRecording2'),task_start(T2, S2), task_end(T2, E2), marker_update(M2, interval(S2, E2, dt(2.0))),marker_scale(M2,[5.5,1.5,4.5]),marker_color(M2,[1.0,0.0,0.0,1.0]),marker_update(M2,interval(S2, E2, dt(2.0))),
	task(T2), task_goal(T2, 'PanoromicMotionRecording2'),task_start(T2, S2), task_end(T2, E2), marker_update(M2, interval(S2, E2, dt(2.0))),marker_scale(M2,[5.5,1.5,4.5]),marker_color(M2,[1.0,0.0,0.0,1.0]),marker_update(M2,interval(S2, E2, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M3, 'Marker3'),
	task(T3), task_goal(T3, 'PanoromicMotionRecording3'),task_start(T3, S3), task_end(T3, E3), marker_update(M3, interval(S3, E3, dt(2.0))),marker_scale(M3,[5.5,1.5,4.5]),marker_color(M3,[1.0,0.0,0.0,1.0]),marker_update(M3,interval(S3, E3, dt(2.0))),
	task(T3), task_goal(T3, 'PanoromicMotionRecording3'),task_start(T3, S3), task_end(T3, E3), marker_update(M3, interval(S3, E3, dt(2.0))),marker_scale(M3,[5.5,1.5,4.5]),marker_color(M3,[1.0,0.0,0.0,1.0]),marker_update(M3,interval(S3, E3, dt(2.0))).

display_secondthreepoints :-
	marker(trajectory('/UWVehicle/base_link'), M4, 'Marker4'),
	task(T4), task_goal(T4, 'PanoromicMotionRecording4'),task_start(T4, S4), task_end(T4, E4), marker_update(M4, interval(S4, E4, dt(2.0))),marker_scale(M4,[5.5,1.5,4.5]),marker_color(M4,[1.0,0.0,0.0,1.0]),marker_update(M4,interval(S4, E4, dt(2.0))),
	task(T4), task_goal(T4, 'PanoromicMotionRecording4'),task_start(T4, S4), task_end(T4, E4), marker_update(M4, interval(S4, E4, dt(2.0))),marker_scale(M4,[5.5,1.5,4.5]),marker_color(M4,[1.0,0.0,0.0,1.0]),marker_update(M4,interval(S4, E4, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M5, 'Marker5'),
	task(T5), task_goal(T5, 'PanoromicMotionRecording5'),task_start(T5, S5), task_end(T5, E5), marker_update(M5, interval(S5, E5, dt(2.0))),marker_scale(M5,[5.5,1.5,4.5]),marker_color(M5,[1.0,0.0,0.0,1.0]),marker_update(M5,interval(S5, E5, dt(2.0))),
	task(T5), task_goal(T5, 'PanoromicMotionRecording5'),task_start(T5, S5), task_end(T5, E5), marker_update(M5, interval(S5, E5, dt(2.0))),marker_scale(M5,[5.5,1.5,4.5]),marker_color(M5,[1.0,0.0,0.0,1.0]),marker_update(M5,interval(S5, E5, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M6, 'Marker6'),
	task(T6), task_goal(T6, 'PanoromicMotionRecording6'),task_start(T6, S6), task_end(T6, E6), marker_update(M6, interval(S6, E6, dt(2.0))),marker_scale(M6,[5.5,1.5,4.5]),marker_color(M6,[1.0,0.0,0.0,1.0]),marker_update(M6,interval(S6, E6, dt(2.0))),
	task(T6), task_goal(T6, 'PanoromicMotionRecording6'),task_start(T6, S6), task_end(T6, E6), marker_update(M6, interval(S6, E6, dt(2.0))),marker_scale(M6,[5.5,1.5,4.5]),marker_color(M6,[1.0,0.0,0.0,1.0]),marker_update(M6,interval(S6, E6, dt(2.0))).


display_thirdthreepoints :-
	marker(trajectory('/UWVehicle/base_link'), M7, 'Marker7'),
	task(T7), task_goal(T7, 'PanoromicMotionRecording7'),task_start(T7, S7), task_end(T7, E7), marker_update(M7, interval(S7, E7, dt(2.0))),marker_scale(M7,[5.5,1.5,4.5]),marker_color(M7,[1.0,0.0,0.0,1.0]),marker_update(M7,interval(S7, E7, dt(2.0))),
	task(T7), task_goal(T7, 'PanoromicMotionRecording7'),task_start(T7, S7), task_end(T7, E7), marker_update(M7, interval(S7, E7, dt(2.0))),marker_scale(M7,[5.5,1.5,4.5]),marker_color(M7,[1.0,0.0,0.0,1.0]),marker_update(M7,interval(S7, E7, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M8, 'Marker8'),
	task(T8), task_goal(T8, 'PanoromicMotionRecording8'),task_start(T8, S8), task_end(T8, E8), marker_update(M8, interval(S8, E8, dt(2.0))),marker_scale(M8,[5.5,1.5,4.5]),marker_color(M8,[1.0,0.0,0.0,1.0]),marker_update(M8,interval(S8, E8, dt(2.0))),
	task(T8), task_goal(T8, 'PanoromicMotionRecording8'),task_start(T8, S8), task_end(T8, E8), marker_update(M8, interval(S8, E8, dt(2.0))),marker_scale(M8,[5.5,1.5,4.5]),marker_color(M8,[1.0,0.0,0.0,1.0]),marker_update(M8,interval(S8, E8, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M9, 'Marker9'),
	task(T9), task_goal(T9, 'PanoromicMotionRecording9'),task_start(T9, S9), task_end(T9, E9), marker_update(M9, interval(S9, E9, dt(2.0))),marker_scale(M9,[5.5,1.5,4.5]),marker_color(M9,[1.0,0.0,0.0,1.0]),marker_update(M9,interval(S9, E9, dt(2.0))),
	task(T9), task_goal(T9, 'PanoromicMotionRecording9'),task_start(T9, S9), task_end(T9, E9), marker_update(M9, interval(S9, E9, dt(2.0))),marker_scale(M9,[5.5,1.5,4.5]),marker_color(M9,[1.0,0.0,0.0,1.0]),marker_update(M9,interval(S9, E9, dt(2.0))).

display_lastthreepoints :-
	marker(trajectory('/UWVehicle/base_link'), M10, 'Marker10'),
	task(T10), task_goal(T10, 'PanoromicMotionRecording10'),task_start(T10, S10), task_end(T10, E10), marker_update(M10, interval(S10, E10, dt(2.0))),marker_scale(M10,[5.5,1.5,4.5]),marker_color(M10,[1.0,0.0,0.0,1.0]),marker_update(M10,interval(S10, E10, dt(2.0))),
	task(T10), task_goal(T10, 'PanoromicMotionRecording10'),task_start(T10, S10), task_end(T10, E10), marker_update(M10, interval(S10, E10, dt(2.0))),marker_scale(M10,[5.5,1.5,4.5]),marker_color(M10,[1.0,0.0,0.0,1.0]),marker_update(M10,interval(S10, E10, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M11, 'Marker11'),
	task(T11), task_goal(T11, 'PanoromicMotionRecording11'),task_start(T11, S11), task_end(T11, E11), marker_update(M11, interval(S11, E11, dt(2.0))),marker_scale(M11,[5.5,1.5,4.5]),marker_color(M11,[1.0,0.0,0.0,1.0]),marker_update(M11,interval(S11, E11, dt(2.0))),
	task(T11), task_goal(T11, 'PanoromicMotionRecording11'),task_start(T11, S11), task_end(T11, E11), marker_update(M11, interval(S11, E11, dt(2.0))),marker_scale(M11,[5.5,1.5,4.5]),marker_color(M11,[1.0,0.0,0.0,1.0]),marker_update(M11,interval(S11, E11, dt(2.0))),
	marker(trajectory('/UWVehicle/base_link'), M12, 'Marker12'),
	task(T12), task_goal(T12, 'PanoromicMotionRecording12'),task_start(T12, S12), task_end(T12, E12), marker_update(M12, interval(S12, E12, dt(2.0))),marker_scale(M12,[5.5,1.5,4.5]),marker_color(M12,[1.0,0.0,0.0,1.0]),marker_update(M12,interval(S12, E12, dt(2.0))),
	task(T12), task_goal(T12, 'PanoromicMotionRecording12'),task_start(T12, S12), task_end(T12, E12), marker_update(M12, interval(S12, E12, dt(2.0))),marker_scale(M12,[5.5,1.5,4.5]),marker_color(M12,[1.0,0.0,0.0,1.0]),marker_update(M12,interval(S12, E12, dt(2.0))),
	camera_pose([-150,0,-80],[0.5,0.5,0.0,0.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
panaroma_picture_display :-
	retrieve_positions_and_addimages,
	display_combinedimage.

retrieve_positions_and_addimages :-
	retrieve_positions_and_addimages_firstsetdata,
	retrieve_positions_and_addimages_secondsetdata,
	retrieve_positions_and_addimages_thirdsetdata,
	retrieve_positions_and_addimages_lastsetdata.

retrieve_positions_and_addimages_firstsetdata :-
	task(T1),task_goal(T1,'PanoromicMotionRecording1'),task_start(T1,S1),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part',S1, Position1),
	task(T2),task_goal(T2,'PanoromicMotionRecording2'),task_start(T2,S2),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part',S2, Position2),
	task(T3),task_goal(T3,'PanoromicMotionRecording3'),task_start(T3,S3),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S3, Position3),
	!,marker(sprite_text('HUD2'), TimeHudNew1), marker_text(TimeHudNew1,'<img src=\"/static/images/Picture1.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew1,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew1),marker_translation(TimeHudNew1,Position1),marker_update(TimeHudNew1),marker_publish,
	!,marker(sprite_text('HUD3'), TimeHudNew2), marker_text(TimeHudNew2,'<img src=\"/static/images/Picture2.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew2,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew2),marker_translation(TimeHudNew2,Position2),marker_update(TimeHudNew2),marker_publish,
	!,marker(sprite_text('HUD4'), TimeHudNew3), marker_text(TimeHudNew3,'<img src=\"/static/images/Picture3.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew3,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew3),marker_translation(TimeHudNew3,Position3),marker_update(TimeHudNew3),marker_publish.

retrieve_positions_and_addimages_secondsetdata :-
	task(T4),task_goal(T4,'PanoromicMotionRecording4'),task_start(T4,S4),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S4, Position4),
	task(T5),task_goal(T5,'PanoromicMotionRecording5'),task_start(T5,S5),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S5, Position5),
	task(T6),task_goal(T6,'PanoromicMotionRecording6'),task_start(T6,S6),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S6, Position6),
!,marker(sprite_text('HUD5'), TimeHudNew4), marker_text(TimeHudNew4,'<img src=\"/static/images/Picture4.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew4,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew4),marker_translation(TimeHudNew4,Position4),marker_update(TimeHudNew4),marker_publish,
	!,marker(sprite_text('HUD6'), TimeHudNew5), marker_text(TimeHudNew5,'<img src=\"/static/images/Picture5.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew5,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew5),marker_translation(TimeHudNew5,Position5),marker_update(TimeHudNew5),marker_publish,
	!,marker(sprite_text('HUD7'), TimeHudNew6), marker_text(TimeHudNew6,'<img src=\"/static/images/Picture6.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew6,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew6),marker_translation(TimeHudNew6,Position6),marker_update(TimeHudNew6),marker_publish.

retrieve_positions_and_addimages_thirdsetdata :-
	task(T7),task_goal(T7,'PanoromicMotionRecording7'),task_start(T7,S7),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S7, Position7),
	task(T8),task_goal(T8,'PanoromicMotionRecording8'),task_start(T8,S8),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S8, Position8),
	task(T9),task_goal(T9,'PanoromicMotionRecording9'),task_start(T9,S9),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S9, Position9),
	!,marker(sprite_text('HUD8'), TimeHudNew7), marker_text(TimeHudNew7,'<img src=\"/static/images/Picture7.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew7,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew7),marker_translation(TimeHudNew7,Position7),marker_update(TimeHudNew7),marker_publish,
	!,marker(sprite_text('HUD9'), TimeHudNew8), marker_text(TimeHudNew8,'<img src=\"/static/images/Picture8.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew8,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew8),marker_translation(TimeHudNew8,Position8),marker_update(TimeHudNew8),marker_publish,
	!,marker(sprite_text('HUD10'), TimeHudNew9), marker_text(TimeHudNew9,'<img src=\"/static/images/Picture9.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew9,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew9),marker_translation(TimeHudNew9,Position9),marker_update(TimeHudNew9),marker_publish.

retrieve_positions_and_addimages_lastsetdata :-
	task(T10),task_goal(T10,'PanoromicMotionRecording10'),task_start(T10,S10),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S10, Position10),
	task(T11),task_goal(T11,'PanoromicMotionRecording11'),task_start(T11,S11),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S11, Position11),
	task(T12),task_goal(T12,'PanoromicMotionRecording12'),task_start(T12,S12),rdf_has(A, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal('UWVehicle/End_part')), mng_lookup_position('/map','UWVehicle/End_part', S12, Position12),
	!,marker(sprite_text('HUD11'), TimeHudNew10), marker_text(TimeHudNew10,'<img src=\"/static/images/Picture10.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew10,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew10),marker_translation(TimeHudNew10,Position10),marker_update(TimeHudNew10),marker_publish,
	!,marker(sprite_text('HUD12'), TimeHudNew11), marker_text(TimeHudNew11,'<img src=\"/static/images/Picture11.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew11,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew11),marker_translation(TimeHudNew11,Position11),marker_update(TimeHudNew11),marker_publish,
	!,marker(sprite_text('HUD13'), TimeHudNew12), marker_text(TimeHudNew12,'<img src=\"/static/images/Picture12.png\" style=\"width:1000px;height:1000px;\">'),marker_color(TimeHudNew12,[0.0,0.0,0.0,1.0]),marker_update(TimeHudNew12),marker_translation(TimeHudNew12,Position12),marker_update(TimeHudNew12),marker_publish,camera_pose([-60,-60,-80],[-0.5,0.5,0.5,0.0]).

display_combinedimage :-
	rdf_has(Perceive,knowrob:'capturedImage',Image),rdf_has(Image,knowrob:'rosTopic',literal('/uwsim/combinedPhotos')),once(designator_publish_image(Perceive)),camera_pose([-60,-60,-80],[-0.5,0.5,0.5,0.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_trajectory_bluishgreen_fish(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Fish1/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish1/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish1/base_link'),[0.0,1.0,0.0,1.0]),
	marker_update(trajectory('/Fish1/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish1/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish1/base_link'),[0.0,1.0,0.0,1.0]),
	marker_update(agent(fishone:'fish1_robot'),E).

display_trajectory_of_greenfish(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Fish3/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish3/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish3/base_link'),[1.0,0.0,1.0,1.0]),
	marker_update(trajectory('/Fish3/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish3/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish3/base_link'),[1.0,0.0,1.0,1.0]),
	marker_update(agent(fishthree:'fish3_robot'),E).

display_trajectory_of_yellowfish(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Fish4/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish4/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish4/base_link'),[1.0,0.5,0.5,1.0]),
	marker_update(trajectory('/Fish4/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish4/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish4/base_link'),[1.0,0.5,0.5,1.0]),
	marker_update(agent(fishfour:'fish4_robot'),E).

display_trajectory_of_bluefish(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Fish5/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish5/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish5/base_link'),[0.5,1.0,0.5,1.0]),
	marker_update(trajectory('/Fish5/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Fish5/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Fish5/base_link'),[0.5,1.0,0.5,1.0]),
	marker_update(agent(fishfive:'fish5_robot'),E).

display_trajectory_of_shark(ActionID):-
	task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), 
	marker_update(trajectory('/Shark1/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Shark1/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Shark1/base_link'),[0.0,1.0,1.0,1.0]),
	marker_update(trajectory('/Shark1/base_link'), interval(S, E, dt(1.0))),marker_scale(trajectory('/Shark1/base_link'),[1.5,1.5,1.5]),marker_color(trajectory('/Shark1/base_link'),[0.0,1.0,1.0,1.0]),
	marker_update(agent(sharkone:'shark1_robot'),E).

clear_objects:-
	marker_remove(trajectory('/UWVehicle/base_link')),marker_remove(agent(uwvehicle2:'Nessie_sample_robot2')),marker_remove(sprite_text('HUD2')),marker_remove(sprite_text('HUD3')),marker_remove(sprite_text('HUD4')),marker_remove(sprite_text('HUD5')),marker_remove(sprite_text('HUD6')),marker_remove(sprite_text('HUD7')),marker_remove(sprite_text('HUD8')),marker_remove(sprite_text('HUD9')),marker_remove(sprite_text('HUD10')),marker_remove(sprite_text('HUD11')),marker_remove(sprite_text('HUD12')),marker_remove(sprite_text('HUD13')).

display_trajectory_of_all_fishes(ActionID):-
	display_trajectory_bluishgreen_fish(ActionID),
	display_trajectory_of_redfish(ActionID),
	display_trajectory_of_greenfish(ActionID),	
	display_trajectory_of_yellowfish(ActionID),
	display_trajectory_of_bluefish(ActionID),
	display_trajectory_of_shark(ActionID),
	camera_pose([-100.0,-250.0,20.0],[-0.1,0.5,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

display_image_acquistion_points :-
	display_first_setpoints,
	display_second_setpoints,
	display_third_setpoints,	
	display_forth_setpoints.

display_first_setpoints :-
	display_first1st_subsetpoints,
	display_first2nd_subsetpoints,
	display_first3rd_subsetpoints,
	display_first4th_subsetpoints.

display_second_setpoints :-
	display_second1st_subsetpoints,
	display_second2nd_subsetpoints,
	display_second3rd_subsetpoints,
	display_second4th_subsetpoints.

display_third_setpoints :-
	display_third1st_subsetpoints,
	display_third2nd_subsetpoints,
	display_third3rd_subsetpoints,
	display_third4th_subsetpoints.

display_forth_setpoints :-
	display_forth1st_subsetpoints,
	display_forth2nd_subsetpoints,
	display_forth3rd_subsetpoints.

display_first1st_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T0, 'Marker0'),marker_update(T0, interval('1473082986','1473082987',dt(2.0))),marker_scale(T0,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T0, 'Marker0'),marker_color(T0,[1.0,0.0,1.0,1.0]),marker_update(T0, interval('1473082986','1473082987',dt(2.0))),marker_scale(T0,[4.5,4.5,4.5]),marker_color(T0,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T1, 'Marker1'),marker_update(T1, interval('1473082996','1473082997',dt(2.0))),marker_scale(T1,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T1, 'Marker1'),marker_color(T1,[1.0,0.0,1.0,1.0]),marker_update(T1, interval('1473082996','1473082997',dt(2.0))),marker_scale(T1,[4.5,4.5,4.5]),marker_color(T1,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T2, 'Marker2'),marker_update(T2, interval('1473083009','1473083010',dt(2.0))),marker_scale(T2,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T2, 'Marker2'),marker_color(T2,[1.0,0.0,1.0,1.0]),marker_update(T2, interval('1473083009','1473083010',dt(2.0))),marker_scale(T2,[4.5,4.5,4.5]),marker_color(T2,[1.0,0.0,1.0,1.0]).

display_first2nd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T3, 'Marker3'),marker_update(T3, interval('1473083018','1473083019',dt(2.0))),marker_scale(T3,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T3, 'Marker3'),marker_color(T3,[1.0,0.0,1.0,1.0]),marker_update(T3, interval('1473083018','1473083019',dt(2.0))),marker_scale(T3,[4.5,4.5,4.5]),marker_color(T3,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T4, 'Marker4'),marker_update(T4, interval('1473083037','1473083038',dt(2.0))),marker_scale(T4,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T4, 'Marker4'),marker_color(T4,[1.0,0.0,1.0,1.0]),marker_update(T4, interval('1473083037','1473083038',dt(2.0))),marker_scale(T4,[4.5,4.5,4.5]),marker_color(T4,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T5, 'Marker5'),marker_update(T5, interval('1473083045','1473083046',dt(2.0))),marker_scale(T5,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T5, 'Marker5'),marker_color(T5,[1.0,0.0,1.0,1.0]),marker_update(T5, interval('1473083045','1473083046',dt(2.0))),marker_scale(T5,[4.5,4.5,4.5]),marker_color(T5,[1.0,0.0,1.0,1.0]).

display_first3rd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T6, 'Marker6'),marker_update(T6, interval('1473083054','1473083055',dt(2.0))),marker_scale(T6,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T6, 'Marker6'),marker_color(T6,[1.0,0.0,1.0,1.0]),marker_update(T6, interval('1473083054','1473083055',dt(2.0))),marker_scale(T6,[4.5,4.5,4.5]),marker_color(T6,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T7, 'Marker7'),marker_update(T7, interval('1473083062','1473083063',dt(2.0))),marker_scale(T7,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T7, 'Marker7'),marker_color(T7,[1.0,0.0,1.0,1.0]),marker_update(T7, interval('1473083062','1473083063',dt(2.0))),marker_scale(T7,[4.5,4.5,4.5]),marker_color(T7,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T8, 'Marker8'),marker_update(T8, interval('1473083102','1473083103',dt(2.0))),marker_scale(T8,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T8, 'Marker8'),marker_color(T8,[1.0,0.0,1.0,1.0]),marker_update(T8, interval('1473083102','1473083103',dt(2.0))),marker_scale(T8,[4.5,4.5,4.5]),marker_color(T8,[1.0,0.0,1.0,1.0]).

display_first4th_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T9, 'Marker9'),marker_update(T9, interval('1473083114','1473083115',dt(2.0))),marker_scale(T9,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T9, 'Marker9'),marker_color(T9,[1.0,0.0,1.0,1.0]),marker_update(T9, interval('1473083114','1473083115',dt(2.0))),marker_scale(T9,[4.5,4.5,4.5]),marker_color(T9,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T10, 'Marker10'),marker_update(T10, interval('1473083125','1473083126',dt(2.0))),marker_scale(T10,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T10, 'Marker10'),marker_color(T10,[1.0,0.0,1.0,1.0]),marker_update(T10, interval('1473083125','1473083126',dt(2.0))),marker_scale(T10,[4.5,4.5,4.5]),marker_color(T10,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T11, 'Marker11'),marker_update(T11, interval('1473083156','1473083157',dt(2.0))),marker_scale(T11,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T11, 'Marker11'),marker_color(T11,[1.0,0.0,1.0,1.0]),marker_update(T11, interval('1473083156','1473083157',dt(2.0))),marker_scale(T11,[4.5,4.5,4.5]),marker_color(T11,[1.0,0.0,1.0,1.0]).

display_second1st_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T12, 'Marker12'),marker_update(T12, interval('1473083164','1473083165',dt(2.0))),marker_scale(T12,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T12, 'Marker12'),marker_color(T12,[1.0,0.0,1.0,1.0]),marker_update(T12, interval('1473083164','1473083165',dt(2.0))),marker_scale(T12,[4.5,4.5,4.5]),marker_color(T12,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T13, 'Marker13'),marker_update(T13, interval('1473083172','1473083173',dt(2.0))),marker_scale(T13,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T13, 'Marker13'),marker_color(T13,[1.0,0.0,1.0,1.0]),marker_update(T13, interval('1473083172','1473083173',dt(2.0))),marker_scale(T13,[4.5,4.5,4.5]),marker_color(T13,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T14, 'Marker14'),marker_update(T14, interval('1473083195','1473083196',dt(2.0))),marker_scale(T14,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T14, 'Marker14'),marker_color(T14,[1.0,0.0,1.0,1.0]),marker_update(T14, interval('1473083195','1473083196',dt(2.0))),marker_scale(T14,[4.5,4.5,4.5]),marker_color(T14,[1.0,0.0,1.0,1.0]).

display_second2nd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T15, 'Marker15'),marker_update(T15, interval('1473083203','1473083204',dt(2.0))),marker_scale(T15,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T15, 'Marker15'),marker_color(T15,[1.0,0.0,1.0,1.0]),marker_update(T15, interval('1473083203','1473083204',dt(2.0))),marker_scale(T15,[4.5,4.5,4.5]),marker_color(T15,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T16, 'Marker16'),marker_update(T16, interval('1473083216','1473083217',dt(2.0))),marker_scale(T16,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T16, 'Marker16'),marker_color(T16,[1.0,0.0,1.0,1.0]),marker_update(T16, interval('1473083216','1473083217',dt(2.0))),marker_scale(T16,[4.5,4.5,4.5]),marker_color(T16,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T17, 'Marker17'),marker_update(T17, interval('1473083231','1473083232',dt(2.0))),marker_scale(T17,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T17, 'Marker17'),marker_color(T17,[1.0,0.0,1.0,1.0]),marker_update(T17, interval('1473083231','1473083232',dt(2.0))),marker_scale(T17,[4.5,4.5,4.5]),marker_color(T17,[1.0,0.0,1.0,1.0]).

display_second3rd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T18, 'Marker18'),marker_update(T18, interval('1473083254','1473083255',dt(2.0))),marker_scale(T18,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T18, 'Marker18'),marker_color(T18,[1.0,0.0,1.0,1.0]),marker_update(T18, interval('1473083254','1473083255',dt(2.0))),marker_scale(T18,[4.5,4.5,4.5]),marker_color(T18,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T19, 'Marker19'),marker_update(T19, interval('1473083264','1473083265',dt(2.0))),marker_scale(T19,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T19, 'Marker19'),marker_color(T19,[1.0,0.0,1.0,1.0]),marker_update(T19, interval('1473083264','1473083265',dt(2.0))),marker_scale(T19,[4.5,4.5,4.5]),marker_color(T19,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T20, 'Marker20'),marker_update(T20, interval('1473083275','1473083276',dt(2.0))),marker_scale(T20,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T20, 'Marker20'),marker_color(T20,[1.0,0.0,1.0,1.0]),marker_update(T20, interval('1473083275','1473083276',dt(2.0))),marker_scale(T20,[4.5,4.5,4.5]),marker_color(T20,[1.0,0.0,1.0,1.0]).

display_second4th_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T21, 'Marker21'),marker_update(T21, interval('1473083283','1473083284',dt(2.0))),marker_scale(T21,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T21, 'Marker21'),marker_color(T21,[1.0,0.0,1.0,1.0]),marker_update(T21, interval('1473083283','1473083284',dt(2.0))),marker_scale(T21,[4.5,4.5,4.5]),marker_color(T21,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T22, 'Marker22'),marker_update(T22, interval('1473083300','1473083301',dt(2.0))),marker_scale(T22,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T22, 'Marker22'),marker_color(T22,[1.0,0.0,1.0,1.0]),marker_update(T22, interval('1473083300','1473083301',dt(2.0))),marker_scale(T22,[4.5,4.5,4.5]),marker_color(T22,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T23, 'Marker23'),marker_update(T23, interval('1473083308','1473083309',dt(2.0))),marker_scale(T23,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T23, 'Marker23'),marker_color(T23,[1.0,0.0,1.0,1.0]),marker_update(T23, interval('1473083308','1473083309',dt(2.0))),marker_scale(T23,[4.5,4.5,4.5]),marker_color(T23,[1.0,0.0,1.0,1.0]).

display_third1st_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T24, 'Marker24'),marker_update(T24, interval('1473083316','1473083317',dt(2.0))),marker_scale(T24,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T24, 'Marker24'),marker_color(T24,[1.0,0.0,1.0,1.0]),marker_update(T24, interval('1473083316','1473083317',dt(2.0))),marker_scale(T24,[4.5,4.5,4.5]),marker_color(T24,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T25, 'Marker25'),marker_update(T25, interval('1473083328','1473083329',dt(2.0))),marker_scale(T25,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T25, 'Marker25'),marker_color(T25,[1.0,0.0,1.0,1.0]),marker_update(T25, interval('1473083328','1473083329',dt(2.0))),marker_scale(T25,[4.5,4.5,4.5]),marker_color(T25,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T26, 'Marker26'),marker_update(T26, interval('1473083355','1473083356',dt(2.0))),marker_scale(T26,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T26, 'Marker26'),marker_color(T26,[1.0,0.0,1.0,1.0]),marker_update(T26, interval('1473083355','1473083356',dt(2.0))),marker_scale(T26,[4.5,4.5,4.5]),marker_color(T26,[1.0,0.0,1.0,1.0]).

display_third2nd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T27, 'Marker27'),marker_update(T27, interval('1473083366','1473083367',dt(2.0))),marker_scale(T27,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T27, 'Marker27'),marker_color(T27,[1.0,0.0,1.0,1.0]),marker_update(T27, interval('1473083366','1473083367',dt(2.0))),marker_scale(T27,[4.5,4.5,4.5]),marker_color(T27,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T28, 'Marker28'),marker_update(T28, interval('1473083376','1473083377',dt(2.0))),marker_scale(T28,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T28, 'Marker28'),marker_color(T28,[1.0,0.0,1.0,1.0]),marker_update(T28, interval('1473083376','1473083377',dt(2.0))),marker_scale(T28,[4.5,4.5,4.5]),marker_color(T28,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T29, 'Marker29'),marker_update(T29, interval('1473083389','1473083390',dt(2.0))),marker_scale(T29,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T29, 'Marker29'),marker_color(T29,[1.0,0.0,1.0,1.0]),marker_update(T29, interval('1473083389','1473083390',dt(2.0))),marker_scale(T29,[4.5,4.5,4.5]),marker_color(T29,[1.0,0.0,1.0,1.0]).

display_third3rd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T30, 'Marker30'),marker_update(T30, interval('1473083412','1473083413',dt(2.0))),marker_scale(T30,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T30, 'Marker30'),marker_color(T30,[1.0,0.0,1.0,1.0]),marker_update(T30, interval('1473083412','1473083413',dt(2.0))),marker_scale(T30,[4.5,4.5,4.5]),marker_color(T30,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T31, 'Marker31'),marker_update(T31, interval('1473083421','1473083422',dt(2.0))),marker_scale(T31,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T31, 'Marker31'),marker_color(T31,[1.0,0.0,1.0,1.0]),marker_update(T31, interval('1473083421','1473083422',dt(2.0))),marker_scale(T31,[4.5,4.5,4.5]),marker_color(T31,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T32, 'Marker32'),marker_update(T32, interval('1473083429','1473083430',dt(2.0))),marker_scale(T32,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T32, 'Marker32'),marker_color(T32,[1.0,0.0,1.0,1.0]),marker_update(T32, interval('1473083429','1473083430',dt(2.0))),marker_scale(T32,[4.5,4.5,4.5]),marker_color(T32,[1.0,0.0,1.0,1.0]).

display_third4th_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T33, 'Marker33'),marker_update(T33, interval('1473083438','1473083439',dt(2.0))),marker_scale(T33,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T33, 'Marker33'),marker_color(T33,[1.0,0.0,1.0,1.0]),marker_update(T33, interval('1473083438','1473083439',dt(2.0))),marker_scale(T33,[4.5,4.5,4.5]),marker_color(T33,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T34, 'Marker34'),marker_update(T34, interval('1473083465','1473083466',dt(2.0))),marker_scale(T34,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T34, 'Marker34'),marker_color(T34,[1.0,0.0,1.0,1.0]),marker_update(T34, interval('1473083465','1473083466',dt(2.0))),marker_scale(T34,[4.5,4.5,4.5]),marker_color(T34,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T35, 'Marker35'),marker_update(T35, interval('1473083478','1473083479',dt(2.0))),marker_scale(T35,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T35, 'Marker35'),marker_color(T35,[1.0,0.0,1.0,1.0]),marker_update(T35, interval('1473083478','1473083479',dt(2.0))),marker_scale(T35,[4.5,4.5,4.5]),marker_color(T35,[1.0,0.0,1.0,1.0]).

display_forth1st_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T36, 'Marker36'),marker_update(T36, interval('1473083490','1473083491',dt(2.0))),marker_scale(T36,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T36, 'Marker36'),marker_color(T36,[1.0,0.0,1.0,1.0]),marker_update(T36, interval('1473083490','1473083491',dt(2.0))),marker_scale(T36,[4.5,4.5,4.5]),marker_color(T36,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T37, 'Marker37'),marker_update(T37, interval('1473083500','1473083501',dt(2.0))),marker_scale(T37,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T37, 'Marker37'),marker_color(T37,[1.0,0.0,1.0,1.0]),marker_update(T37, interval('1473083500','1473083501',dt(2.0))),marker_scale(T37,[4.5,4.5,4.5]),marker_color(T37,[1.0,0.0,1.0,1.0]).

display_forth2nd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T38, 'Marker38'),marker_update(T38, interval('1473083546','1473083547',dt(2.0))),marker_scale(T38,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T38, 'Marker38'),marker_color(T38,[1.0,0.0,1.0,1.0]),marker_update(T38, interval('1473083546','1473083547',dt(2.0))),marker_scale(T38,[4.5,4.5,4.5]),marker_color(T38,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T39, 'Marker39'),marker_update(T39, interval('1473083554','1473083555',dt(2.0))),marker_scale(T39,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T39, 'Marker39'),marker_color(T39,[1.0,0.0,1.0,1.0]),marker_update(T39, interval('1473083554','1473083555',dt(2.0))),marker_scale(T39,[4.5,4.5,4.5]),marker_color(T39,[1.0,0.0,1.0,1.0]).

display_forth3rd_subsetpoints:-
	marker(trajectory('/UWVehicle/base_link'), T40, 'Marker40'),marker_update(T40, interval('1473083562','1473083563',dt(2.0))),marker_scale(T40,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T40, 'Marker40'),marker_color(T40,[1.0,0.0,1.0,1.0]),marker_update(T40, interval('1473083562','1473083563',dt(2.0))),marker_scale(T40,[4.5,4.5,4.5]),marker_color(T40,[1.0,0.0,1.0,1.0]),
	marker(trajectory('/UWVehicle/base_link'), T41, 'Marker41'),marker_update(T41, interval('1473083571','1473083572',dt(2.0))),marker_scale(T41,[4.5,4.5,4.5]),marker(trajectory('/UWVehicle/base_link'), T41, 'Marker41'),marker_color(T41,[1.0,0.0,1.0,1.0]),marker_update(T41, interval('1473083571','1473083572',dt(2.0))),marker_color(T41,[1.0,0.0,1.0,1.0]),camera_pose([-180,-320,40],[0,0.5,0.5,0]).


clear_all_trajectories_objects:-
	marker_remove(agent(sharkone:'shark1_robot')),marker_remove(agent(fishone:'fish1_robot')),marker_remove(agent(fishtwo:'fish2_robot')),marker_remove(agent(fishthree:'fish3_robot')),marker_remove(agent(fishfour:'fish4_robot')),marker_remove(agent(fishfive:'fish5_robot')),marker_remove(agent(sharkone:'shark1_robot')),marker_remove(trajectory('/Fish1/base_link')),marker_remove(trajectory('/Fish2/base_link')),marker_remove(trajectory('/Fish3/base_link')),marker_remove(trajectory('/Fish4/base_link')),marker_remove(trajectory('/Fish5/base_link')),marker_remove(trajectory('/Shark1/base_link')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

highlight_images_acq_points :-
	marker_highlight('Marker0',[1.0,1.0,0.0,1.0]),marker_highlight('Marker4',[1.0,1.0,0.0,1.0]),marker_highlight('Marker5',[1.0,1.0,0.0,1.0]),marker_highlight('Marker11',[1.0,1.0,0.0,1.0]),marker_highlight('Marker13',[1.0,1.0,0.0,1.0]),marker_highlight('Marker15',[1.0,1.0,0.0,1.0]),marker_highlight('Marker16',[1.0,1.0,0.0,1.0]),marker_highlight('Marker17',[1.0,1.0,0.0,1.0]),marker_highlight('Marker18',[1.0,1.0,0.0,1.0]),marker_highlight('Marker19',[1.0,1.0,0.0,1.0]),marker_highlight('Marker24',[1.0,1.0,0.0,1.0]),marker_highlight('Marker25',[1.0,1.0,0.0,1.0]),marker_highlight('Marker39',[1.0,1.0,0.0,1.0]),marker_highlight('Marker40',[1.0,1.0,0.0,1.0]),camera_pose([-180,-320,40],[0,0.5,0.5,0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display_couple_imagesacquired_obj(ActionID):-
task(T), task_goal(T, ActionID),task_start(T, S), task_end(T, E), marker_update(agent(fishone:'fish1_robot'),S),marker_update(agent(fishfive:'fish5_robot'),S),marker_update(agent(uwvehicle2:'Nessie_sample_robot2'),S),rdf_has(T,knowrob:'capturedImage',Image),once(designator_publish_image(T)),camera_pose([-80,-180,10],[0,0.5,0.5,0]).
