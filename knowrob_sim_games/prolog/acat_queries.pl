
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

:- module(acat_queries,
    [
    a1_test/0
    ]).

a1_test :-
	
	sg_load_experiments('/home/haidu/sr_experimental_data/a1'),
	
	connect_to_db('ACAT1-db'),
	
	% load the semantic map
	owl_parse('package://knowrob_sim_games/owl/desk.owl'),
	marker_update(object('http://knowrob.org/kb/desk.owl#SimDesk_gVa3')),

   %marker(mesh('http://knowrob.org/kb/labels.owl#pizza_AleMDa28D1Kmvc'), MeshMarkerObj),
   %marker_mesh_resource(MeshMarkerObj, 'package://kitchen/food-drinks/pizza-credentials/pizza.dae'),
   %% marker_pose(MeshMarkerObj, pose(0.0,0.0,1.0,0.0,0.0,0.0,1.0)),

   	exp_tag(EpInst, ExpTag),
	write('** Experiment tag: '), write(ExpTag), nl,

	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspRingEventInst, GrStart, GrEnd),
	% check for grasping events
	event_type(GraspRingEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspRingEventInst, knowrob:'PressureRing'),

	write('Start:'), writeln(GrStart), write('End:'), writeln(GrEnd),
	sg_marker_remove_all,

	% view model traj
	view_model_traj(EpInst, 'PressureRing', GrStart, GrEnd, 'model_traj_id', 0.05, 'blue', 0.01),
	%view_model_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'model_traj_id2'),

	%view_model_traj(EpInst, 'Spatula', GrStart, GrEnd, 'model_traj_id', 0.05, 'blue', 0.01),
	%view_model_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'model_traj_id2', 0.05, 'green', 0.01),

	% view link traj
	%view_link_traj(EpInst, 'Spatula', 'spatula_head_link', GrStart, GrEnd, 'link_traj_id'),
	%view_link_traj(EpInst, 'Mondamin', 'mondamin_link', GrStart, GrEnd, 'link_traj_id2'),

	%view_link_traj(EpInst, 'Spatula', 'spatula_head_link', GrStart, GrEnd, 'link_traj_id', 0.05, 'blue', 0.01),
	%view_link_traj(EpInst, 'Mondamin', 'mondamin_link', GrStart, GrEnd, 'link_traj_id2', 0.05, 'green', 0.01),

	% view link traj
	%view_collision_traj(EpInst, 'Mondamin', 'mondamin_link', 'mondamin_event_collision', GrStart, GrEnd, 'coll_traj_id'),
	%view_collision_traj(EpInst, 'Mondamin', 'mondamin_link', 'mondamin_event_collision', GrStart, GrEnd, 'coll_traj_id', 0.05, 'green', 0.01),	

	% view links pos
	%view_links_positions(EpInst, 'LiquidTangibleThing', GrStart, 'liquid_links_pos_id'),
	%% view_links_positions(EpInst, 'LiquidTangibleThing', GrStart, 'liquid_links_pos_id2', 'green', 0.01),
	
	% view links trajs
	%view_links_trajs(EpInst, 'LiquidTangibleThing', GrStart, GrEnd, 'liquid_links_trajs_id'),
	%view_links_trajs(EpInst, 'LiquidTangibleThing', GrStart, GrEnd, 'liquid_links_trajs_id2', 0.05, 'green', 0.01),

	% view mesh
	%view_mesh_at(EpInst, 'Spatula', GrStart, 'package://sim/pancake/pancake_maker.dae','meshID'),

	% view mesh traj
	%% view_mesh_traj(EpInst, 'Mondamin', GrStart, GrEnd, 'package://sim/pancake/mondamin_pancake_mix.dae', 'meshTrajID',0.5),

	
	writeln('End').
	
	