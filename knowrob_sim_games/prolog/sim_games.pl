
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

:- module(knowrob_sim_games,
    [
    	exp_inst/1,
    	sg_task_context/2,
    	sg_occurs/2,
    	sg_occurs/4,
    	sg_exp_timeline/3,
    	event_type/2,
    	acted_on/2,
    	particle_type/2,
    	in_contact/3,
    	only_in_contact/3,
    
    	sg_load_experiments/1,
    	connect_to_db/1,
    	index_db/1,
    	set_coll/1,
    	exp_tag/2,

    	
    	get_raw_coll_name/2,
    	get_model_traj/5,
    	view_model_traj/5,
    	view_model_traj/9,
    	view_mesh_at/5,
    	view_mesh_traj/7,
    	get_link_traj/6,
    	view_link_traj/6,
    	view_link_traj/10,
    	get_collision_traj/7,
    	view_collision_traj/7,
    	view_collision_traj/11,
    	get_links_positions/4,
    	view_links_positions/4,
    	view_links_positions/7,
    	get_links_trajs/5,
    	view_links_trajs/5,
    	view_links_trajs/9,
    	get_pancake_roundness/4,
    	check_model_flip/5,

    	sg_marker_remove/1,
    	sg_marker_remove_all/0
    ]).

:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_sim, 'http://knowrob.org/kb/knowrob_sim.owl#', [keep(true)]).


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    event_type(+, r),
    acted_on(+, r),
    particle_type(+, r),
    in_contact(+, r, r),
    only_in_contact(+, r, r).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


% get the instance of the experiment
exp_inst(EpInst) :-
	rdf_has(EpInst, rdf:type, knowrob:'RobotExperiment').


% get the class name of the event instance
sg_task_context(EventInstance, TaskContext) :-
   rdf_has(EventInstance, knowrob:'taskContext', literal(type(_, TaskContext))).


% get events which occured in the experiment
sg_occurs(EpInst, EventInst) :-
	rdf_has(EpInst, knowrob:'subAction', EventInst).


% get events which occured in the experiments with their times
sg_occurs(EpInst, EventInst, Start, End) :-
	rdf_has(EpInst, knowrob:'subAction', EventInst),
	rdf_has(EventInst, knowrob:'startTime', Start),    
	rdf_has(EventInst, knowrob:'endTime', End).

% create timeline diagram of the given experiment
sg_exp_timeline(EpInst, DiagramID, Title) :-
	findall(TC-(ST-ET),
		(sg_occurs(EpInst, EvInst, Start, End), sg_task_context(EvInst, TC), time_term(Start, ST), time_term(End, ET)), 
		Events),
	pairs_keys_values(Events, Contexts, Times), 
	pairs_keys_values(Times, StartTimes, EndTimes),
	add_timeline(DiagramID, Title, Contexts, StartTimes, EndTimes).

% get a given event type
event_type(EventInst, EventType) :-
	rdf_has(EventInst, rdf:type, EventType).
	
	
% check object acted on
acted_on(EventInst, ObjActedOnType) :-
	rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),	
	rdf_has(ObjActedOnInst, rdf:type, ObjActedOnType).


% check particle type
particle_type(EventInst, ParticleType) :-
	rdf_has(EventInst, knowrob:'particleType', Particle),	
	rdf_has(Particle, rdf:type, ParticleType).


% check objects in contact
in_contact(EventInst, O1Type, O2Type) :-
	% get the objects in contact
	rdf_has(EventInst, knowrob_sim:'inContact', O1Inst),
	rdf_has(EventInst, knowrob_sim:'inContact', O2Inst),    
	% make sure the objects are distinct
	O1Inst \== O2Inst,
	% check for the right type of contacts    
	rdf_has(O1Inst, rdf:type, O1Type),
	rdf_has(O2Inst, rdf:type, O2Type),
	% avoid NamedIndividual returns for the object types
	not(rdf_equal(O1Type, owl:'NamedIndividual')),
	not(rdf_equal(O2Type, owl:'NamedIndividual')).
	

% check that Obj 1 is only in contact with Obj 2
only_in_contact(EventInst, O1Type, O2Type) :-
	% get all the objects in contact with Obj 1
	findall(_, in_contact(EventInst, O1Type, _), AllContacts),
	% make sure the size of the contacts is one
	length(AllContacts, Length), Length == 1,
	% get the first(only) value from the list (the head)
	[Head|_] = AllContacts,
	% make sure the contact is with Obj 2 
	rdf_equal(Head, O2Type).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Create the mongo sim interface object
% (makes sure the interface is only created once)

% set flag
:- assert(mng_sim_interface(fail)).

% from knowrob_mongo.pl
mongo_sim_interface :-
    mongo_sim_interface(_).

% check flag, then init interface
mongo_sim_interface(MongoSim) :-
    mng_sim_interface(fail), 	
    jpl_new('org.knowrob.knowrob_sim_games.MongoSimGames', [], MongoSim),
    retract(mng_sim_interface(fail)),
    assert(mng_sim_interface(MongoSim)),!.

% if set, return object
mongo_sim_interface(MongoSim) :-
    mng_sim_interface(MongoSim).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Create the vis sim interface object
% (makes sure the interface is only created once)

% set flag
:- assert(vs_sim_interface(fail)).

% from knowrob_mongo.pl
vis_sim_interface :-
    vis_sim_interface(_).

% check flag, then init interface
vis_sim_interface(VisSim) :-
    vs_sim_interface(fail), 	
    jpl_new('org.knowrob.knowrob_sim_games.VisSimGames',
			[],	VisSim),
    retract(vs_sim_interface(fail)),
    assert(vs_sim_interface(VisSim)),!.

% if set, return object
vis_sim_interface(VisSim) :-
    vs_sim_interface(VisSim).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Loads two-folder-nested owl files from a given path
% TODO recusively load all owl files

sg_load_experiments(Path) :-
	% get all entries from the given path
	directory_files(Path, Entries),
	% iterate through all the 1st level entries
	forall(member(CurrEntry, Entries),(
			% check that entries are not special types '..' / '.'
			(CurrEntry \== '.', CurrEntry \== '..') ->
					% true branch
					(
						% add '/' to the entry name and add them to the path
						atom_concat('/', CurrEntry, CurrentFolder),
						atom_concat(Path, CurrentFolder, CurrFolderPath),
						% get all entries from the current folder
						directory_files(CurrFolderPath, SecondEntries),
						% iterate through all the 2nd level entries
						forall(member(CurrSecondEntry, SecondEntries),(									
									% check that entries are not special types '..' / '.' and is owl
									file_name_extension(_, Ext, CurrSecondEntry),
									%write('!!!! Ext of type: '),write(Ext),
									(CurrSecondEntry \== '.', CurrSecondEntry \== '..', Ext == 'owl') ->
										% true branch
										(
											% add '/' to the filename and add it to the path
											atom_concat('/', CurrSecondEntry, CurrOwl),
											atom_concat(CurrFolderPath, CurrOwl, OwlPath),																			
											% parse the current owl file
											owl_parse(OwlPath)
										);
										%false branch
										%write('Skip entry: '), write(CurrSecondEntry), nl
										true
								)
							)
					);
					% false branch
					%write('Skip entry: '), write(CurrEntry), nl
					true
			)
	). 


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Database where to read the raw data from
connect_to_db(DBName) :-
	mongo_sim_interface(MongoSim),
	jpl_call(MongoSim, 'SetDatabase', [DBName], @void). 
	
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Index database
index_db(DBName) :-
	mongo_sim_interface(MongoSim),
	jpl_call(MongoSim, 'IndexOnTimestamp', [DBName], @void). 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
% Set the collection from which to query
% TODO make sure the DB is set
set_coll(Coll) :-
	mongo_sim_interface(MongoSim),
	jpl_call(MongoSim, 'SetCollection', [Coll], @void).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the tag of the given experiment instance	
exp_tag(EpInst, ExpTag) :-
	rdf_has(EpInst, knowrob:'experiment', literal(type(_, ExpTag))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the name of the raw collection of the experiment instance
get_raw_coll_name(EpInst, Coll) :-
	exp_tag(EpInst, ExpTag),
	string_concat(ExpTag, '_raw', CollStr),
	atom_codes(Coll, CollStr).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the traj of the model at the given times
% Model = 'Spatula',
% TODO make sure the correct coll is set, check that model exists
get_model_traj(EpInst, Model, Start, End, DBName) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'WriteModelTrajectory', 
		[Start, End, Model, DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the model at the given times
% Model = 'Spatula', 
% TODO make sure the correct coll is set, check that model exists
view_model_traj(EpInst, Model, Start, End, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewModelTrajectory', 
		[Start, End, Model, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the model at the given times with DT deltaT as steps
% Model = 'Spatula', 
% DT = 0.01 (seconds)
% Scale = 0.01
% Color = 'blue'
% TODO make sure the correct coll is set, check that model exists
view_model_traj(EpInst, Model, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewModelTrajectory', 
		[Start, End, Model, MarkerID, MarkerType, Color, Scale, DT], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the mesh of the model at the given time
% Model = 'Spatula', 
% TODO make sure the correct coll is set, check that model exists
view_mesh_at(EpInst, Model, Ts, MeshPath, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewModelMeshAt', 
		[Ts, Model, MeshPath, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the model mesh at the given times
% Model = 'Spatula', 
% TODO make sure the correct coll is set, check that model exists
view_mesh_traj(EpInst, Model, Start, End, MeshPath, MarkerID, DT) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewModelMeshTrajectory', 
		[Start, End, Model, MeshPath, MarkerID, DT], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the traj of the link at the given times
% Model = 'Spatula', 
% Link = 'spatula_head_link', 	
get_link_traj(EpInst, Model, Link, Start, End, DBName) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'WriteLinkTrajectory', 
		[Start, End, Model, Link, DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the link at the given times
% Model = 'Spatula', 
% Link = 'spatula_head_link', 	
view_link_traj(EpInst, Model, Link, Start, End, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinkTrajectory', 
		[Start, End, Model, Link, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the link at the given times with DT deltaT as steps
% Model = 'Spatula', 
% Link = 'spatula_head_link', 	
view_link_traj(EpInst, Model, Link, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinkTrajectory', 
		[Start, End, Model, Link, MarkerID, MarkerType, Color, Scale, DT], @void).
	
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the traj of the collision at the given times
% Model = 'Spatula', 
% Link = 'spatula_head_link', 
% Collision = 'spatula_head_collision', 	
get_collision_traj(EpInst, Model, Link, Collision, Start, End, DBName) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'WriteCollisionTrajectory', 
		[Start, End, Model, Link, Collision, DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the collision at the given times
% Model = 'Spatula', 
% Link = 'spatula_head_link', 
% Collision = 'spatula_head_collision', 	
view_collision_traj(EpInst, Model, Link, Collision, Start, End, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewCollisionTrajectory', 
		[Start, End, Model, Link, Collision, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the traj of the collision at the given times with DT deltaT as steps
% Model = 'Spatula', 
% Link = 'spatula_head_link', 
% Collision = 'spatula_head_collision', 	
view_collision_traj(EpInst, Model, Link, Collision, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewCollisionTrajectory', 
		[Start, End, Model, Link, Collision, MarkerID, MarkerType, Color, Scale, DT], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the positions of all the links of the model
% Model = 'LiquidTangibleThing'
get_links_positions(EpInst, Model, Timestamp, DBName) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'WriteLinksPositionsAt', 
		[Timestamp, Model, DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the positions of all the links of the model
% Model = 'LiquidTangibleThing'
view_links_positions(EpInst, Model, Timestamp, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinksPositionsAt', 
		[Timestamp, Model, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the positions of all the links of the model
% Model = 'LiquidTangibleThing'
view_links_positions(EpInst, Model, Timestamp, MarkerID, MarkerType, Color, Scale) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinksPositionsAt', 
		[Timestamp, Model, MarkerID, MarkerType, Color, Scale], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the trajectories of all the links of the model
% Model = 'LiquidTangibleThing'
get_links_trajs(EpInst, Model, Start, End, DBName) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'WriteLinksTrajs', 
		[Start, End, Model, DBName], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the trajectories of all the links of the model
% Model = 'LiquidTangibleThing'
view_links_trajs(EpInst, Model, Start, End, MarkerID) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinksTrajs', 
		[Start, End, Model, MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% View the trajectories of all the links of the model with DT deltaT as steps
% Model = 'LiquidTangibleThing'
view_links_trajs(EpInst, Model, Start, End, MarkerID, MarkerType, Color, Scale, DT) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'ViewLinksTrajs', 
		[Start, End, Model, MarkerID, MarkerType, Color, Scale, DT], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the positions of the links
% Model = 'LiquidTangibleThing', 
get_pancake_roundness(EpInst, Model, Timestamp, Roundness) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'GetPancakeRoundness', 
		[Timestamp, Model], Roundness).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% check if the model has been flipped between the two timestamps
% Model = 'LiquidTangibleThing'
check_model_flip(EpInst, Model, Start, End, Flip) :-
	mongo_sim_interface(MongoSim),
	get_raw_coll_name(EpInst, CollName),
	set_coll(CollName),
	jpl_call(MongoSim, 'CheckModelFlip', 
		[Start, End, Model], Flip).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% remove the marker witht he given ID
% MarkerID = 'coll_traj_id'
sg_marker_remove(MarkerID) :-
  	mongo_sim_interface(MongoSim),
    jpl_call(MongoSim, 'RemoveMarker', [MarkerID], @void).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% remove all markers created in sg
sg_marker_remove_all :-
  	mongo_sim_interface(MongoSim),
    jpl_call(MongoSim, 'RemoveAllMarkers', [], @void).