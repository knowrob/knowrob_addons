/** <module> robcog_games

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


:- module(robcog_games,
    [
        ep_inst/1,
        u_inst_name/2,
        show_ep_sem_map/1,
        sem_map_inst/1,
        sem_map_inst/2,
        rating_score/3,
        u_task_context/2,
        u_occurs/2,
        u_occurs/4,
        u_ep_timeline/3,
        u_ep_timeline/4,
        event_type/2,
        obj_type/2,
        obj_mesh/2,
        acted_on/2,
        performed_by/2,

        particle_type/2,
        from_location/2,
        goal_location/2,
        in_contact/3,
        only_in_contact/3,

        u_load_episodes/1,
        ep_tag/2,
        ep_folder/2,
        rating_file/2,
        get_mongo_coll_name/2,

        u_split_pose/3

    ]).


% returns the namspace when outputting values
:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_u, 'http://knowrob.org/kb/knowrob_u.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(log_u, 'http://knowrob.org/kb/robcog_log.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(u_map, 'http://knowrob.org/kb/u_map.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    u_inst_name(r,r),
    show_ep_sem_map(+),
    u_ep_timeline(r,r,+,+),
    sem_map_inst(r, r),
    rating_score(r, r, r),
    event_type(+, r),
    obj_type(+, r),
    obj_mesh(+, r),
    acted_on(+, r),
    performed_by(+, r),
    particle_type(+, r),
    in_contact(+, r, r),
    only_in_contact(+, r, r).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% get the instance of the episode
ep_inst(EpInst) :-
    rdf_has(EpInst, rdf:type, knowrob:'UnrealExperiment').

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% get the unique short name of the class
% e.g. Class = knowrob:'LeftHand'
u_inst_name(Class, InstShortName) :-
    rdf_has(Inst, rdf:type, Class),
    iri_xml_namespace(Inst, _, InstShortName).

% view the semantic map of the episode
% the cut ('!') operator is needed due to the following deadlock error with marker updates:
% PrologException: error(permission_error(write, rdf_db, default),
% context(_96, 'Operation would deadlock'))
show_ep_sem_map(EpInst) :-
    sem_map_inst(EpInst, SemMap),!,
    marker_update(object(SemMap)).

% get semantic map instance
sem_map_inst(MapInst) :-
    rdf_has(MapInst, rdf:type, knowrob:'SemanticEnvironmentMap').

% TODO robcog not needed since we have one map
% get the semantic map instance of the episode
sem_map_inst(EpInst, MapInst) :-
    rdf_has(EpInst, knowrob_u:'semanticMap', MapInst),
    rdf_has(MapInst, rdf:type, knowrob:'SemanticEnvironmentMap').

% get the score of the rating
rating_score(EpInst, RatingType, RatingScore) :- 
    rdf_has(EpInst, knowrob_u:'rating', Ratings),
    rdf_has(RatingInst, knowrob_u:'ratingOf', Ratings),
    rdf_has(RatingInst, rdf:type, RatingType),
    not(rdf_equal(RatingType, owl:'NamedIndividual')),
    rdf_has(RatingInst, knowrob_u:'ratingScore', literal(type(_, RatingScore))).

% get the class name of the event instance
u_task_context(EventInst, TaskContext) :-
   rdf_has(EventInst, knowrob:'taskContext', literal(type(_, TaskContext))).

% get events which occured in the experiment
u_occurs(EpInst, EventInst) :-
    rdf_has(EpInst, knowrob:'subAction', EventInst).

% get events which occured in the episodes
u_occurs(EpInst, EventInst, Start, End) :-
    rdf_has(EpInst, knowrob:'subAction', EventInst),
    rdf_has(EventInst, knowrob:'startTime', Start),
    rdf_has(EventInst, knowrob:'endTime', End).

% create timeline diagram of the given experiment
u_ep_timeline(EpInst, DiagramID, Title) :-
    findall(TC-(ST-ET),
        (u_occurs(EpInst, EvInst, Start, End), u_task_context(EvInst, TC), time_term(Start, ST), time_term(End, ET)), 
        Events),
    pairs_keys_values(Events, Contexts, Times), 
    pairs_keys_values(Times, StartTimes, EndTimes),
    add_timeline(DiagramID, Title, Contexts, StartTimes, EndTimes).

% create timeline diagram of the given experiment with of the given class
% e.g Class = knowrob_u:'TouchingSituation'
u_ep_timeline(EpInst, Class, DiagramID, Title) :-
    findall(TC-(ST-ET),(
            u_occurs(EpInst, EvInst, Start, End),
            rdfs_instance_of(EvInst, Class),
            u_task_context(EvInst, TC),
            time_term(Start, ST), time_term(End, ET)), 
        Events),
    pairs_keys_values(Events, Contexts, Times), 
    pairs_keys_values(Times, StartTimes, EndTimes),
    add_timeline(DiagramID, Title, Contexts, StartTimes, EndTimes).

% get a given event type
event_type(EventInst, EventType) :-
    rdf_has(EventInst, rdf:type, EventType).

% get the type of the object instance
obj_type(ObjInst, ObjType) :-
    rdf_has(ObjInst, rdf:type, ObjType),
    not(rdf_equal(ObjType, owl:'NamedIndividual')).

% get the type of the object instance
obj_mesh(ObjInst, ObjMeshPath) :-
    rdf_has(ObjInst, knowrob:'pathToCadModel', literal(type(_, ObjMeshPath))).

% check object acted on
acted_on(EventInst, ObjActedOnType) :-
    rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
    rdf_has(ObjActedOnInst, rdf:type, ObjActedOnType).

% check performed by
performed_by(EventInst, PerformedByInst) :-
    rdf_has(EventInst, knowrob:'performedBy', PerformedByInst).

% check particle type
particle_type(EventInst, ParticleType) :-
    rdf_has(EventInst, knowrob:'particleType', Particle), 
    rdf_has(Particle, rdf:type, ParticleType).

% check from location
from_location(EventInst, ObjType) :-
    rdf_has(EventInst, knowrob:'fromLocation', Obj),   
    rdf_has(Obj, rdf:type, ObjType).

% check goal location
goal_location(EventInst, ObjType) :-
    rdf_has(EventInst, knowrob:'goalLocation', Obj),   
    rdf_has(Obj, rdf:type, ObjType).

% check objects in contact
in_contact(EventInst, O1Type, O2Type) :-
    % get the objects in contact
    rdf_has(EventInst, knowrob_u:'inContact', O1Inst),
    rdf_has(EventInst, knowrob_u:'inContact', O2Inst),    
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
% Loads two-folder-nested owl files from a given path
% TODO recusively load all owl files
u_load_episodes(Path) :-
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
% Get the tag of the given episode instance
ep_tag(EpInst, ExpTag) :-
    rdf_has(EpInst, knowrob:'experiment', literal(type(_, ExpTag))).

% get the path of the rating file
ep_folder(EpInst, FolderName) :-
    rdf_has(EpInst, knowrob:'experiment', literal(type(_, Value))),
    atom_concat(Value, '/', FolderName).

% get the path of the rating file
rating_file(EpInst, FileName) :-
    rdf_has(EpInst, knowrob:'experiment', literal(type(_, Value))),
    atom_concat(Value,'_EpisodeRating.owl', FileName).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Get the name of the raw collection of the episode instance
get_mongo_coll_name(EpInst, Coll) :-
    ep_tag(EpInst, ExpTag),
    string_concat('RawData_', ExpTag, CollStr),
    atom_codes(Coll, CollStr).

% =================================================================================
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Helper functions
maplist_arr_to_list(ObjsArr, List) :-
    maplist(jpl_array_to_list, ObjsArr, List).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
% Split pose list into position and quaternion
u_split_pose(Pose, Pos, Quat) :-
    [X,Y,Z|Quat] = Pose,
    Pos = [X,Y,Z].
