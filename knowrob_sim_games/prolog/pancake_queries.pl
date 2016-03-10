
/** <module> knowrob_sim_games

  Copyright (C) 2014-15 by Moritz Tenorth, Andrei Haidu

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

:- module(pancake_queries,
    [
    check_pancake_short/1,
    check_pancake_ext/1,
    check_pancake_raw/9,
    
    grasp_mondamin/4,
    particle_transl/4,
    liquid_only_on_oven/4,
    grasp_spatula/4,
    slide_under/4,

    loop_pancake_exp/0,
    loop_pouring_roundness/1,
    loop_flip/0,
    loop_pancake_raw/0,
    p_test/0
    ]).

%%
grasp_mondamin(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for grasping events
	event_type(EventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(EventInst, knowrob:'Mondamin').

%%
particle_transl(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for particle translation
	event_type(EventInst, knowrob_sim:'ParticleTranslation').
	
%%
liquid_only_on_oven(EpInst, EventInst, Start, End) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for touching situation
	event_type(EventInst, knowrob_sim:'TouchingSituation'),
	% check obj 1 is only in contact with obj 2
	only_in_contact(EventInst, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker').

%%
grasp_spatula(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for grasping events
	event_type(EventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(EventInst, knowrob:'Spatula').

%%
slide_under(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for touching situation
	event_type(EventInst, knowrob_sim:'TouchingSituation'),
	% check objects in contact
	in_contact(EventInst, knowrob:'Spatula', knowrob:'PancakeMaker').


%%
check_pancake_short(EpInst) :-
	% check if mondamin has been graspped
	grasp_mondamin(EpInst, GraspMondaminEventInst, _, _),	
	
	% check for particle translation
	particle_transl(EpInst, ParticleTranslEventInst, _, _),	
	
	% check that the particle transl happened during the container grasp
	comp_duringI(ParticleTranslEventInst, GraspMondaminEventInst),	
	
	% check that the liquid is only on the oven
	liquid_only_on_oven(EpInst, LiquidContactEventInst, _, LiquidContactEnd),
	
	% check that the container grasping overlaps the liquid contacts
	comp_overlapsI(GraspMondaminEventInst, LiquidContactEventInst),
		
	% check that the spatula has been grasped
	grasp_spatula(EpInst, GraspSpatulaEventInst, _, GraspSpatulaEnd),
		
	% check that the container grasp happened before the spatula grasp
	comp_beforeI(GraspMondaminEventInst, GraspSpatulaEventInst),	
	
	% check sliding under the pancake with the spatula
	slide_under(EpInst, SlideContactInst, _, _),
	
	% check that the sliding under happened during the tool grasp
	comp_duringI(SlideContactInst, GraspSpatulaEventInst),

	% check that the liquid is only on the oven after the flip
	liquid_only_on_oven(EpInst, LiquidContactEventInst2, _, _),

	% check that a liquid contact interuption happens during the tool grasping   
	comp_overlapsI(LiquidContactEventInst, GraspSpatulaEventInst),
	comp_overlapsI(GraspSpatulaEventInst, LiquidContactEventInst2),

	% run computable to check if the model has been flipped between the timestamps
	check_model_flip(EpInst, 'LiquidTangibleThing', LiquidContactEnd, GraspSpatulaEnd, Flip),
	Flip == @true.


%%
check_pancake_ext(EpInst) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspMondaminEventInst, _, _),
	% check for grasping events
	event_type(GraspMondaminEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspMondaminEventInst, knowrob:'Mondamin'),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, ParticleTranslEventInst, _, _),
	% check for particle translation
	event_type(ParticleTranslEventInst, knowrob_sim:'ParticleTranslation'),	
	% check that the particle transl happened during the container grasp
	comp_duringI(ParticleTranslEventInst, GraspMondaminEventInst),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, LiquidContactEventInst, _, LiquidContactEnd),
	% check for touching situation
	event_type(LiquidContactEventInst, knowrob_sim:'TouchingSituation'),
	% check that the liquid is only in contact with the oven
	only_in_contact(LiquidContactEventInst, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker'),
	% check that the container grasping overlaps the liquid contacts
	comp_overlapsI(GraspMondaminEventInst, LiquidContactEventInst),
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspSpatulaEventInst, _, GraspSpatulaEnd),
	% check for grasping events
	event_type(GraspSpatulaEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspSpatulaEventInst, knowrob:'Spatula'),		
	% check that the container grasp happened before the spatula grasp
	comp_beforeI(GraspMondaminEventInst, GraspSpatulaEventInst),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, SlideContactInst, _, _),
	% check for touching situation
	event_type(SlideContactInst, knowrob_sim:'TouchingSituation'),
	% check objects in contact
	in_contact(SlideContactInst, knowrob:'Spatula', knowrob:'PancakeMaker'),	
	% check that the sliding under happened during the tool grasp
	comp_duringI(SlideContactInst, GraspSpatulaEventInst),
	% get events which occurred in the experiments
	sg_occurs(EpInst, LiquidContactEventInst2, _, _),
	% check for touching situation
	event_type(LiquidContactEventInst2, knowrob_sim:'TouchingSituation'),
	% check that the liquid is only in contact with the oven
	only_in_contact(LiquidContactEventInst2, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker'),
	% check that a liquid contact interuption happens during the tool grasping   
	comp_overlapsI(LiquidContactEventInst, GraspSpatulaEventInst),
	comp_overlapsI(GraspSpatulaEventInst, LiquidContactEventInst2),
	% run computable to check if the model has been flipped between the timestamps
	check_model_flip(EpInst, 'LiquidTangibleThing', LiquidContactEnd, GraspSpatulaEnd, Flip),
	Flip == @true.


%%
check_pancake_raw(EpInst, ContainerGraspInst, ContainerGraspObjInst,
			ParticleTranslInst, LiquidContactInstBeforeFlip,
			ToolGraspInst, ToolGraspObjInst, FlipStart, FlipEnd) :-
	% get the instance of the experiment
	rdf_has(EpInst, rdf:type, knowrob:'RobotExperiment'),
	%write('** EpInst: '), write(EpInst), nl,
	%exp_tag(EpInst, ExpTag),
	%write('** Experiment tag: '), write(ExpTag), nl,
	
	
	%%% GRASP MONDAMIN
	% check that the first grasp is a subActions of the current experiment	
	rdf_has(EpInst, knowrob:'subAction', ContainerGraspInst),
		
	% check that the first grasp is of type knowrob:'GraspingSomething'
	rdf_has(ContainerGraspInst, rdf:type, knowrob:'GraspingSomething'),
	
	% get the first grasped object instance
	rdf_has(ContainerGraspInst, knowrob:'objectActedOn', ContainerGraspObjInst),
	
	% check that the first grasped object is of type mondamin (TODO Container)
	rdf_has(ContainerGraspObjInst, rdf:type, knowrob:'Mondamin'),
	
	
	
	%%% PARTICLE TRANSLATION
	% check that the particle-transl is a subActions of the current experiment
	rdf_has(EpInst, knowrob:'subAction', ParticleTranslInst),
	
	% check that it is of type knowrob_sim:'ParticleTranslation'
	rdf_has(ParticleTranslInst, rdf:type, knowrob_sim:'ParticleTranslation'),	
		
	% check that the particle transl happened during the container grasp
	comp_duringI(ParticleTranslInst, ContainerGraspInst),



	%%% LIQUID CONTACTS
	% check that the contact instace is a subAction of the current experiment
	rdf_has(EpInst, knowrob:'subAction', LiquidContactInstBeforeFlip),	
		
	% check that it is of type knowrob_sim:'TouchingSituation'
	rdf_has(LiquidContactInstBeforeFlip, rdf:type, knowrob_sim:'TouchingSituation'),
	
	% get the objects in contact
	rdf_has(LiquidContactInstBeforeFlip, knowrob_sim:'inContact', LiquidObj1Inst),
	rdf_has(LiquidContactInstBeforeFlip, knowrob_sim:'inContact', LiquidObj2Inst),

	% make sure the objects differ
	LiquidObj1Inst \== LiquidObj2Inst,

	% check for the right type of contacts    
	rdf_has(LiquidObj1Inst, rdf:type, knowrob:'LiquidTangibleThing'),
	rdf_has(LiquidObj2Inst, rdf:type, knowrob:'PancakeMaker'),

	% check that the container grasping overlaps the liquid contacts
	comp_overlapsI(ContainerGraspInst, LiquidContactInstBeforeFlip),



	%%% GRASP SPATULA
	% check that the second grasp is a subActions of the current experiment
	rdf_has(EpInst, knowrob:'subAction', ToolGraspInst),
	
	% check that the second grasp is of type knowrob:'GraspingSomething'
	rdf_has(ToolGraspInst, rdf:type, knowrob:'GraspingSomething'),
	
	% get the second grasped object instance
	rdf_has(ToolGraspInst, knowrob:'objectActedOn', ToolGraspObjInst),
	
	% check that the first grasped object is of type spatula (TODO Tool?)
	rdf_has(ToolGraspObjInst, rdf:type, knowrob:'Spatula'),
		
	% check that the container grasp happened before the spatula grasp
	comp_beforeI(ContainerGraspInst, ToolGraspInst),
	
	
	
	%%% SLIDE UNDER PANCAKE
	% check that the contact instace is a subAction of the current experiment
	rdf_has(EpInst, knowrob:'subAction', SlideContactInst),	
	
	% check that it is of type knowrob_sim:'TouchingSituation'
	rdf_has(SlideContactInst, rdf:type, knowrob_sim:'TouchingSituation'),
	
	% get the objects in contact 
	rdf_has(SlideContactInst, knowrob_sim:'inContact', SlideObj1Inst),
	rdf_has(SlideContactInst, knowrob_sim:'inContact', SlideObj2Inst),

	% make sure the objects differ
	SlideObj1Inst \== SlideObj2Inst,

	% check for the right type of contacts    
	rdf_has(SlideObj1Inst, rdf:type, knowrob:'Spatula'),
	rdf_has(SlideObj2Inst, rdf:type, knowrob:'PancakeMaker'),
	
	% check that the sliding under happened during the tool grasp
	comp_duringI(SlideContactInst, ToolGraspInst),
	
	
	
	%%% LIQUID CONTACT DISCONECTION
	% check that the contact instace is a subAction of the current experiment
	rdf_has(EpInst, knowrob:'subAction', LiquidContactInstAfterFlip),	
	
	% check that it is of type knowrob_sim:'TouchingSituation'
	rdf_has(LiquidContactInstAfterFlip, rdf:type, knowrob_sim:'TouchingSituation'),
	
	% get the objects in contact
	rdf_has(LiquidContactInstAfterFlip, knowrob_sim:'inContact', LiquidObj1Inst2),
	rdf_has(LiquidContactInstAfterFlip, knowrob_sim:'inContact', LiquidObj2Inst2),

	% make sure the objects differ
	LiquidObj1Inst2 \== LiquidObj2Inst2,

	% check for the right type of contacts    
	rdf_has(LiquidObj1Inst2, rdf:type, knowrob:'LiquidTangibleThing'),
	rdf_has(LiquidObj2Inst2, rdf:type, knowrob:'PancakeMaker'),

	% check that a liquid contact interuption happend during the tool grasping   
	comp_overlapsI(LiquidContactInstBeforeFlip, ToolGraspInst),
	comp_overlapsI(ToolGraspInst, LiquidContactInstAfterFlip),


	%%% CHECK FLIP
	% get the timestamps when the liquid is not in contact with the oven
	% the end of the first contact, and the beginning of the second
	rdf_has(LiquidContactInstBeforeFlip, knowrob:'endTime', FlipStart),    
	%rdf_has(LiquidContactInstAfterFlip, knowrob:'startTime', FlipEnd),
	% get the time of the release of the tool, so the pancake has time to settle,
	% otherwise the contact might be saved when the pancake is at 90deg with the oven
	rdf_has(ToolGraspInst, knowrob:'endTime', FlipEnd),

	% check if the model has been flipped between the timestamps
	check_model_flip(EpInst, 'LiquidTangibleThing', FlipStart, FlipEnd, Flip),
	Flip == @true
	.

%%
loop_pancake_exp :-
	
	% load all experiments
	sg_load_experiments('/home/haidu/sr_experimental_data/mixed_pancake'),
	
	% connect to the raw data 
	connect_to_db('SIM-oldpancake-db'),	
	
	% get the instance of the current experiment
	exp_inst(EpInst),
	
	% check if pancake succesfully created
	check_pancake_ext(EpInst),
	
	% get experiment tag
	exp_tag(EpInst, ExpTag),
	write('** In '), write(ExpTag), write(' the pancake was succesfully created'), nl.


%%
loop_pouring_roundness(Limit) :-
	% load all experiments
	sg_load_experiments('/home/haidu/sr_experimental_data/mixed_pancake'),
	
	% connect to the raw data 
	connect_to_db('SIM-oldpancake-db'),	
	
	% get the instance of the current experiment
	exp_inst(EpInst),
	
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspMondaminEventInst, _, GraspMondaminEnd),
	% check for grasping events
	event_type(GraspMondaminEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspMondaminEventInst, knowrob:'Mondamin'),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, ParticleTranslEventInst, TranslStart, TranslEnd),
	% check for particle translation
	event_type(ParticleTranslEventInst, knowrob_sim:'ParticleTranslation'),	
	% check that the particle transl happened during the container grasp
	comp_duringI(ParticleTranslEventInst, GraspMondaminEventInst),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, LiquidContactEventInst, _, _),
	% check for touching situation
	event_type(LiquidContactEventInst, knowrob_sim:'TouchingSituation'),
	% check that the liquid is only in contact with the oven
	only_in_contact(LiquidContactEventInst, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker'),
	% check that the container grasping overlaps the liquid contacts
	comp_overlapsI(GraspMondaminEventInst, LiquidContactEventInst),
	
	% computable for the roundness of the pancake at the given timepoint	
	get_pancake_roundness(EpInst, 'LiquidTangibleThing', GraspMondaminEnd, Roundness),
	% check if the roundness meets the limit
	Roundness > Limit,
	
	% save the positon of the links in the database
	get_links_positions(EpInst, 'LiquidTangibleThing', GraspMondaminEnd, 'roundness_pancake_pos'),
	
	% save the pouring trajectory
	get_collision_traj(EpInst, 'Mondamin', 'mondamin_link', 'mondamin_event_collision',
		 TranslStart, TranslEnd, 'roundness_pouring_traj'),
	
	exp_tag(EpInst, ExpTag),
	write('** In '), write(ExpTag), write(' the pouring was round enough! '),
	write(Roundness), nl.


%%
loop_flip :-
	% load all experiments
	sg_load_experiments('/home/haidu/sr_experimental_data/mixed_pancake'),
	
	% connect to the raw data 
	connect_to_db('SIM-oldpancake-db'),	
	
	% get the instance of the current experiment
	exp_inst(EpInst),
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspMondaminEventInst, _, _),
	% check for grasping events
	event_type(GraspMondaminEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspMondaminEventInst, knowrob:'Mondamin'),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, ParticleTranslEventInst, _, _),
	% check for particle translation
	event_type(ParticleTranslEventInst, knowrob_sim:'ParticleTranslation'),	
	% check that the particle transl happened during the container grasp
	comp_duringI(ParticleTranslEventInst, GraspMondaminEventInst),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, LiquidContactEventInst, _, LiquidContactEnd),
	% check for touching situation
	event_type(LiquidContactEventInst, knowrob_sim:'TouchingSituation'),
	% check that the liquid is only in contact with the oven
	only_in_contact(LiquidContactEventInst, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker'),
	% check that the container grasping overlaps the liquid contacts
	comp_overlapsI(GraspMondaminEventInst, LiquidContactEventInst),
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspSpatulaEventInst, _, GraspSpatulaEnd),
	% check for grasping events
	event_type(GraspSpatulaEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspSpatulaEventInst, knowrob:'Spatula'),		
	% check that the container grasp happened before the spatula grasp
	comp_beforeI(GraspMondaminEventInst, GraspSpatulaEventInst),	
	% get events which occurred in the experiments
	sg_occurs(EpInst, SlideContactInst, SlideStart, SlideEnd),
	% check for touching situation
	event_type(SlideContactInst, knowrob_sim:'TouchingSituation'),
	% check objects in contact
	in_contact(SlideContactInst, knowrob:'Spatula', knowrob:'PancakeMaker'),	
	% check that the sliding under happened during the tool grasp
	comp_duringI(SlideContactInst, GraspSpatulaEventInst),
	% get events which occurred in the experiments
	sg_occurs(EpInst, LiquidContactEventInst2, LiquidContactStart2, _),
	% check for touching situation
	event_type(LiquidContactEventInst2, knowrob_sim:'TouchingSituation'),
	% check that the liquid is only in contact with the oven
	only_in_contact(LiquidContactEventInst2, knowrob:'LiquidTangibleThing', knowrob:'PancakeMaker'),
	% check that a liquid contact interuption happens during the tool grasping   
	comp_overlapsI(LiquidContactEventInst, GraspSpatulaEventInst),
	comp_overlapsI(GraspSpatulaEventInst, LiquidContactEventInst2),
	% run computable to check if the model has been flipped between the timestamps
	check_model_flip(EpInst, 'LiquidTangibleThing', LiquidContactEnd, GraspSpatulaEnd, Flip),
	Flip == @true,
	
	% save flipping trajectory of the pancake
	get_model_traj(EpInst, 'LiquidTangibleThing', LiquidContactEnd, LiquidContactStart2, 'flip_traj'),	
	
	% save flipping trajectory of the pancake links
	get_links_trajs(EpInst, 'LiquidTangibleThing', LiquidContactEnd, LiquidContactStart2, 'flip_links_traj'),	
		
	% save the slide under pancake traj
	get_link_traj(EpInst, 'Spatula', 'spatula_head_link', SlideStart, SlideEnd, 'slide_traj'),
	
	exp_tag(EpInst, ExpTag),
	write('** In '), write(ExpTag), write(' the pancake was successfully flipped! ').
	

	
loop_pancake_raw :-
	
	sg_load_experiments('/home/haidu/sr_experimental_data/mixed_pancake'),
	
	connect_to_db('SIM-oldpancake-db'),	
	
	
	check_pancake_raw(EpInst, ContainerGraspInst, ContainerGraspObjInst,
				ParticleTranslInst, LiquidContactInstBeforeFlip,
				ToolGraspInst, ToolGraspObjInst, FlipStart, FlipEnd),
	
	exp_tag(EpInst, ExpTag),
	write('** EXIT Experiment tag: '), write(ExpTag), nl,
	write(ContainerGraspInst), nl,
	write(ContainerGraspObjInst), nl,
	write(ParticleTranslInst), nl,
	write(LiquidContactInstBeforeFlip), nl,
	write(ToolGraspInst), nl,
	write(ToolGraspObjInst), nl,
	write(FlipStart), nl,
	write(FlipEnd), nl.



p_test :-
	
	sg_load_experiments('/home/haidu/sr_experimental_data/mixed_pancake'),
	
	connect_to_db('SIM-oldpancake-db'),	
	
	exp_tag(EpInst, ExpTag),
	write('** Experiment tag: '), write(ExpTag), nl,

	%% sg_exp_timeline(EpInst, 'diag_id1', 'Diag title');

	% load the semantic map
	%owl_parse('package://knowrob_sim_games/owl/pizza_kitchen_table.owl'), 
	%marker_update(object('http://knowrob.org/kb/pizza_kitchen_table.owl#SimPizzaMap_gVb2')),
	
	% load the semantic map
	%% owl_parse('package://knowrob_sim_games/owl/kitchen_table.owl'), 
	%% marker_update(object('http://knowrob.org/kb/kitchen_table.owl#SimPancakeMap_gVb3')),

   %marker(mesh('http://knowrob.org/kb/labels.owl#pizza_AleMDa28D1Kmvc'), MeshMarkerObj),
   %marker_mesh_resource(MeshMarkerObj, 'package://kitchen/food-drinks/pizza-credentials/pizza.dae'),
   %% marker_pose(MeshMarkerObj, pose(0.0,0.0,1.0,0.0,0.0,0.0,1.0)),

	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspMondaminEventInst, GrStart, GrEnd),
	% check for grasping events
	event_type(GraspMondaminEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspMondaminEventInst, knowrob:'Mondamin'),

	write('Start:'), writeln(GrStart), write('End:'), writeln(GrEnd),
	sg_marker_remove_all,

	view_nested_mesh_at(EpInst, 'Hand', GrStart, 'package://sim/r_hit_hand/meshes/','meshID'),

	% view model traj
	%view_model_traj(EpInst, 'Spatula', GrStart, GrEnd, 'model_traj_id'),
	%view_model_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'model_traj_id2'),

	%view_model_traj(EpInst, 'Spatula', GrStart, GrEnd, 'model_traj_id', 'cube', 'blue', 0.01, 0.05),
	%view_model_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'model_traj_id2', 'sphere', 'green', 0.01, 0.05),

	% view link traj
	%view_link_traj(EpInst, 'Spatula', 'spatula_head_link', GrStart, GrEnd, 'link_traj_id'),
	%view_link_traj(EpInst, 'Mondamin', 'mondamin_link', GrStart, GrEnd, 'link_traj_id2'),

	%view_link_traj(EpInst, 'Spatula', 'spatula_head_link', GrStart, GrEnd, 'link_traj_id', 'point', 'blue', 0.01, 0.05),
	%view_link_traj(EpInst, 'Mondamin', 'mondamin_link', GrStart, GrEnd, 'link_traj_id2', 'cube', 'green', 0.01, 0.05),

	% view coll traj
	%view_collision_traj(EpInst, 'Mondamin', 'mondamin_link', 'mondamin_event_collision', GrStart, GrEnd, 'coll_traj_id'),
	%% view_collision_traj(EpInst, 'Mondamin', 'mondamin_link', 'mondamin_event_collision', GrStart, GrEnd, 'coll_traj_id', 'point', 'green', 0.1, 0.05),	

	% view links pos
	%view_links_positions(EpInst, 'LiquidTangibleThing', GrStart, 'liquid_links_pos_id'),
	%% view_links_positions(EpInst, 'LiquidTangibleThing', GrStart, 'liquid_links_pos_id2', 'sphere', 'red', 0.01),
	
	% view links trajs
	%view_links_trajs(EpInst, 'LiquidTangibleThing', GrStart, GrEnd, 'liquid_links_trajs_id'),
	%% view_links_trajs(EpInst, 'LiquidTangibleThing', GrStart, GrEnd, 'liquid_links_trajs_id2', 'cube', 'green', 0.01, 0.05),

	% view mesh
	%view_mesh_at(EpInst, 'Spatula', GrStart, 'package://sim/pancake/pancake_maker.dae','meshID'),

	% view mesh traj
	%% view_mesh_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'package://sim/pancake/mondamin_pancake_mix.dae', 'meshTrajID', 0.5),
	
	%marker(mesh('http://knowrob.org/kb/labels.owl#pizza_AleMDa28D1Kmvc'), MeshMarkerObj),
	%marker_mesh_resource(MeshMarkerObj, 'package://kitchen/food-drinks/pizza-credentials/pizza.dae'),

	%writeln('removing marker'),
	%sg_marker_remove('coll_traj_id'),
	%writeln('marker removed?'),

	
	writeln('End').
	
	