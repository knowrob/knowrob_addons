/** <module> knowrob_sim

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


:- module(knowrob_sim,
    [
        simact/2,
        simact/3,
        simact_contact/4,
        simact_contact/6,
        simact_contact_specific/3,
        simact_contact_specific/4,
        simlift/4,
        simlift_specific/3,
        simlift_liftonly/4,
        simflip_full/9,
        simflip_fliponly/9,
        supported_during/4,
        simact_start/3,
        simact_end/3,
        subact/3,
        subact_all/2,
        successful_simacts_for_goal/2,
        simflipping/8,
        simgrasped/4,
        add_count/1,
        experiment_file/1,
        load_sim_experiments/2,
        anyact/3,
        intersected_uid_event/6,
        sim_subsumes/4,
        sim_timeline_val/4,
        get_exp_times/3,
        interval_setdifference/6
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/comp_temporal')).
:- use_module(library('knowrob/mongo')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_sim, 'http://knowrob.org/kb/knowrob_sim.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    simact(r,r),
    simact(r,r,r),
    simact_contact(r,r,r,r),
    simact_contact(r,r,r,r,r,r),
    simact_contact_specific(r,r,r),
    simact_contact_specific(r,r,r,r),
    simlift(r,r,r,r),
    simlift_specific(r,r,r),
    simlift_liftonly(r,r,r,r),
    simflip_full(r,r,r,r,r,r,r,r,r),
    simflip_fliponly(r,r,r,r,r,r,r,r,r),
    supported_during(r,r,r,r),
    subact(r,r),
    subact_all(r,r),
    simact_start(r,r),
    simact_end(r,r),
    simflipping(r,r,r,r,r,r,r,r),
    simgrasped(r,r,r,r),
    experiment_file(r),
    load_sim_experiments(r,r),
    intersected_uid_event(r,r,r,r,r,r),
    successful_simacts_for_goal(+,-),
    anyact(r,r,r),
    sim_subsumes(r,r,r,r),
    sim_timeline_val(r,r,r,r),
    get_exp_times(r,r,r),
    successful_simacts_for_goal(+,-).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% List of available experiments

exp_list_new(['pf_10.owl','pf_13.owl','pf_16.owl','pf_19.owl','pf_2.owl','pf_5.owl','pf_8.owl','sim_exp2.owl','sim_exp5.owl','pf_11.owl','pf_14.owl','pf_17.owl','pf_1.owl','pf_3.owl','pf_6.owl','pf_9.owl','sim_exp3.owl','sim_exp6.owl','pf_12.owl','pf_15.owl','pf_18.owl','pf_20.owl','pf_4.owl','pf_7.owl','sim_exp1.owl','sim_exp4.owl']).
exp_list(['sim_exp1.owl','sim_exp2.owl','sim_exp4.owl','sim_exp6.owl']).

experiment_file(X):-
    exp_list(List),
    member(X,List).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Basic simact handling
% 

%% load_experiment(+ExpOwlPath, +ExpFiles) is nondet.
%
%  Loads the owl logfiles of a list of experiments, located on ExpOwlPath using
%  repeated calls to load_experiment in knowrob_cram
%
%  @param ExpOwlPath path where the logfiles are located
%  @param ExpFiles list of logfile names to be loaded
% 
load_sim_experiments(_, []).
load_sim_experiments(ExpOwlPath, [ExpFile|T]) :-
    atom_concat(ExpOwlPath, ExpFile, Path),
    (load_experiment(Path) -> load_sim_experiments(ExpOwlPath, T); load_sim_experiments(ExpOwlPath, T)).

%% get_exp_times(?Exp, ?StartTime, ?EndTime)
%   
%  Provides information on the global start and endtime of a certain experiment.
%
get_exp_times(Exp, Start, End):-
    rdf_has(MetaData, knowrob:'experiment', literal(type(_, Exp))), 
    rdf_has(MetaData, knowrob:'startTime',Start), 
    rdf_has(MetaData, knowrob:'endTime',End).

%% simact(?Task) is nondet.
%
%  Check if class of Task is a subclass of SimulationEvent, or looks for Tasks that are subclass of SimulationEvent
%  Keeps track which experiment a certain event belongs to by searching for experiment
%  and only returning events that are a subaction of that experiment
%
%  @param Task Identifier of given Task
% 
simact(Experiment, EventID) :-
    rdf_has(EventID, rdf:type, EventClass),
    rdf_reachable(EventClass, rdfs:subClassOf, knowrob_sim:'SimulationEvent'),
    rdf_has(MetaData, knowrob:'experiment', literal(type(_, Experiment))),
    rdf_has(MetaData, knowrob:'subAction', EventID). %make sure that the event is an subaction of (belongs to) a specific experiment

%%
% This is the same as simact but does not require eventclasses to be a subclass of
% simulationEvent (works for events that haven't been added to the ontology yet)
%
anyact(Experiment, EventID, EventClass) :-
    rdf_has(EventID, rdf:type, EventClass), 
    rdf_has(MetaData, knowrob:'experiment', literal(type(_, Experiment))), 
    rdf_has(MetaData, knowrob:'subAction', EventID).

%%
%Checks whether there are any events of type EventClass that are not ID1 or ID2 
%that happen between Start and End.
%Solely used to make scooping query shorter
intersected_uid_event(Exp, ID1, ID2, EventClass, Start, End) :-
    anyact(Exp, TmpID, EventClass),
    rdf_has(TmpID, knowrob:'startTime', TmpStart),
    rdf_has(TmpID, knowrob:'endTime', TmpEnd),
    ID1\=TmpID,
    ID2\=TmpID,
    sim_subsumes(Start, End, TmpStart, TmpEnd).

%% simact(?Event, ?EventClass) is nondet.
%
%  Finds EventIDs of EventClass that are subclass of SimulationEvent
%  For example: simact(E, knowrob_sim:'TouchingSituation').
%
%  @param Event Identifier of given Event
%  @param EventClass Identifier of given EventClass
% 
simact(Experiment, EventID, EventClass) :-
    rdf_has(EventID, rdf:type, EventClass),
    rdf_reachable(EventClass, rdfs:subClassOf, knowrob_sim:'SimulationEvent'),
    rdf_has(MetaData, knowrob:'experiment', literal(type(_, Experiment))),
    rdf_has(MetaData, knowrob:'subAction', EventID).

%% sim_class_individual(?ObjectClass, ?ObjectIndivid)
%
% Get the ObjectClass of an individual, or an individual of a certain class
sim_class_individual(ObjectClass, ObjectIndivid) :-
    rdf_has(ObjectIndivid, rdf:type, ObjectClass),
    rdf_reachable(ObjectClass, rdfs:subClassOf, owl:'Thing'). %Otherwise it will return owl:namedIndividual as a Class of any objectinstance as well

%%  simact_contact(?Event, ?EventClass, ?ObjectClass) 
%
%   Find a certain event involving a certain object type(s) or certain object(s)
%
%   Example calls:
%   > simact_contact(Exp, E, knowrob_sim:'Cup', O).
%   > simact_contact_specific(Exp, E, knowrob_sim:'Cup_object_hkm6glYmRQ0BWF').
%   > simact_contact(Exp, E, knowrob_sim:'Cup', knowrob_sim:'KitchenTable', O1, O2).
%   > simact_contact_specific(Exp, E, knowrob_sim:'Cup_object_hkm6glYmRQ0BWF', knowrob_sim:'KitchenTable_object_50SJX00eStoIfD').
simact_contact(Experiment, Event, ObjectClass, ObjectInstance) :-
    simact(Experiment, Event, knowrob_sim:'TouchingSituation'),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance),
    sim_class_individual(ObjectClass, ObjectInstance).
    %simact_start(Experiment, Event, StartTime).
    %writeln(StartTime).
 %% Find a certain event involving a certain object
simact_contact_specific(Experiment, Event, ObjectInstance) :-
    simact(Experiment, Event, knowrob_sim:'TouchingSituation'),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance).
%%  simact_contact(?Event, ?EventClass, +Object1Class, +Object2Class, -ObjectInstance1, -ObjectInstance2)
%   
%   Find a certain event involving certain object types
%
simact_contact(Experiment, Event, Object1Class, Object2Class, ObjectInstance1, ObjectInstance2) :-
    simact(Experiment, Event, knowrob_sim:'TouchingSituation'),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance1),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance2),
    sim_class_individual(Object1Class, ObjectInstance1),
    sim_class_individual(Object2Class, ObjectInstance2),
    ObjectInstance1\=ObjectInstance2.
%%  simact_contact(?Event, ?EventClass, +ObjectInstance1, +ObjectInstance2)
%   
%   Find a certain event involving certain objects
%
simact_contact_specific(Experiment, Event, ObjectInstance1, ObjectInstance2) :-
    simact(Experiment, Event, knowrob_sim:'TouchingSituation'),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance1),
    rdf_has(Event, knowrob_sim:'inContact', ObjectInstance2),
    ObjectInstance1\=ObjectInstance2.

%% Function returns a range for each event in the list during which that event is true
%
% Example call for plotting timeline: sim_timeline_val(Exp, Events, Times), add_diagram('id', 'Title', timeline, 'Time', 'Events', 300, 300, '12px', [[Events,Times]]).
sim_timeline_val(Expname, EventNamesList, StartTimeList, EndTimeList):-
    %Find all events from a single experiment
    rdf_has(_, knowrob:'experiment', literal(type(_, Expname))),
    %Get list of all the events that happen in the experiment for extracting timepoints
    findall(Type, simact(Expname,Type), EventList), 
    %Get list of event names that happened in the experment for putting in the timeline
    findall(EventName, (member(E,EventList),rdf_split_url(_,EventName,E)), EventNamesList),
    %Get list of all [start|end] times of the events in the list
    findall(StartTime, (member(Event, EventList), simact_start(Exp,Event,Start), time_term(Start, StartTime)), StartTimeList),
    findall(EndTime, (member(Event, EventList), simact_end(Exp,Event,End), time_term(End, EndTime)), EndTimeList).


%%  Auxilary function for enabling changing color on consecutive calls
%
%   Step should be something like 0x404000

%   Example use:
%   nb_setval(counter,0x0080ff), simlift_liftonly(Exp, knowrob:'Cup', Start, End), 
%   add_count(0x404000), nb_getval(counter,Val).
%   Val will be different after each backtrack. Can use this as color
%
%   WARNING: not sure what will happen when "maximum" is reached
%
add_count(Step) :- 
    nb_getval(counter, C), CNew is C + Step, nb_setval(counter, CNew), writeln(CNew).

%% ignore this predicate, doesn't work right now but is also not so important, meant to change color of successive calls
simact_count(T, Class) :-
    findall(Task, simact(Task,Class), T),
    length(T, Val),
    write('Counter:'), writeln(Val).

%% subact(?Event, ?SubEvent) is nondet.
%
%  Check if SubEvent is a child of Event
%  Can be used for exampel to  
%
%  @param Task Identifier of given Task
%  @param Subsimact Identifier of given Subsimact
% 
subact(Experiment, Event, SubEvent) :-
    rdf_has(Event, knowrob:'subAction', SubEvent),
    simact(Experiment, Event),
    simact(Experiment, SubEvent).

%% subsimact_all(?Task, ?Subsimact) is nondet.
%
%  Check if Task is an ancestor of Subsimact in the simact tree
%
%  @param Task Identifier of given Task
%  @param Subsimact Identifier of given Subsimact
% 
subact_all(Event, SubEvent) :-
    owl_has(Event, knowrob:subAction,  SubEvent).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Simact specific actions handling
% 

simgrasped(Experiment, EventID, ObjectClass, ObjectInstance) :-
    simact(Experiment, EventID, knowrob:'GraspingSomething'),
    rdf_has(EventID, knowrob:'objectActedOn', ObjectInstance), %for a lift to occur, the event in which the object participates must involve GraspingSomething (lift can only happen while the object is grasped)
    sim_class_individual(ObjectClass, ObjectInstance).

%% Find event interval during which a specific object type is lifted
%% 
%% Example call: 
%% > simlift(E, knowrob_sim:'Cup').
simlift(Experiment, EventID, ObjectClass, ObjectInstance) :-
    simact(Experiment, EventID, knowrob:'GraspingSomething'),
    rdf_has(EventID, knowrob:'objectActedOn', ObjectInstance), %for a lift to occur, the event in which the object participates must involve GraspingSomething (lift can only happen while the object is grasped)
    sim_class_individual(ObjectClass, ObjectInstance),
    not(supported_during(Experiment, EventID, _, ObjectInstance)). %check that this specific object is not in a contact relation with a supporting object for at least part of the interval (the interval will overlap at least with some contact intervals, because for example when you lift the mug, it will still be in contact with the table while the hand initiates contact). 

%% Find event interval during which a specific object is lifted
%% 
%% Example call: 
%% > simlift_specific(E, knowrob_sim:'Cup_object_hkm6glYmRQ0BWF').
simlift_specific(Experiment, EventID, ObjectInstance) :-
    simact(Experiment, EventID, knowrob:'GraspingSomething'),
    rdf_has(EventID, knowrob:'objectActedOn', ObjectInstance),
    not(supported_during(Experiment, EventID, _,ObjectInstance)).

%% Gives a new interval (start and endtimes) during which the specified object is lifted
%% Differs from simlift because simlift can only return existing event intervals, and some
%% overlap with supportedby intervals are inevitable, while simlift_liftonly "defines" a new
%% interval by looking for the difference between grasping intervals and all supportedby 
%% intervals.
%%
%% Example call:
%% > simlift_liftonly(Exp, knowrob_sim:'Cup', Start, End).
simlift_liftonly(Experiment, ObjectClass, Start, End) :-
    simact(Experiment, EventID, knowrob:'GraspingSomething'),
    rdf_has(EventID, knowrob:'objectActedOn', ObjectInstance), %for a lift to occur, the event in which the object participates must involve GraspingSomething (lift can only happen while the object is grasped)
    sim_class_individual(ObjectClass, ObjectInstance),
    simact_start(Experiment, EventID, TempStart),
    simact_end(Experiment, EventID, TempEnd),
    findall(EventID2, (simsupported(Experiment, EventID2, ObjectInstance), not(comp_temporallySubsumes(EventID2, EventID))), Candidates),
    %% writeln(Candidates),
    interval_setdifference(Experiment, TempStart, TempEnd, Candidates, Start, End).

%% Flipping: grasping start until object entirely on tool, start contact tool and target, start object on tool until object back on pancakemaker, start still grasping tool until tool put down.
simflipping(Experiment, ObjectO, ToolO, TargetO, GraspS, ObjectLiftS, PutbackS, PutbackE) :-
    % start of GraspSpatula
    simgrasped(Experiment, Event1ID, _,ToolO),
    simact_start(Experiment, Event1ID, GraspS),
    simact_end(Experiment, Event1ID, PutbackE), %this end is the very end of the flipping
    % Object and target should have a contact interval overlapping with the beginning of GraspSpatula
    simact_contact(Experiment, Event0ID, knowrob:'LiquidTangibleThing', _, ObjectO, TargetO),
    comp_overlapsI(Event0ID, Event1ID),
    ToolO\=ObjectO,
    ObjectO\=TargetO,
    ToolO\=TargetO,
    %% Tool is in contact with the Object but Object has not left Target yet
    %% simact_contact(Experiment, Event2ID, _, _, ToolO, ObjectO),
    %% simact_start(Experiment, Event2ID, ToolCTargetS),
    %% Tool is no longer in contact with the Target, but it is in contact with the Object. There is a small overlapping issue because the pancake leaves the pancakemaker a few miliseconds after the spatula does, so ObjectLiftS starts a bit before simact_end(Event0ID, End0) ends.
    simflip_fliponly(Experiment,knowrob:'LiquidTangibleThing',_,_,ObjectLiftS, PutbackS, ObjectO, ToolO, TargetO).

%% Gives a new interval, which is the union of contactPancake-Spatula and contactSpatula-Liquid.
%% These two events should be overlapping in order to be a full flipping interval 
%% Finds a flip given the class of the object to be flipped and the tool with which this is done
%% Note that maybe the most important thing, whether or not the object was turned, cannot be deducted from the owl file
%% 
%% Example call: 
%% > simflip_full(knowrob_sim:'LiquidTangibleThing', knowrob_sim:'Spatula', knowrob_sim:'PancakeMaker', Start, End, OObj, TObj, LObj).
simflip_full(Experiment, ObjectClass, ToolClass, LocationClass, Start, End, OObj, TObj, LObj) :-
    % get contactInterval spatula-pancakemaker
    simact_contact(Experiment, EventID, ToolClass, LocationClass, TObj, LObj),
    % get contactInterval spatula-liquid
    simact_contact(Experiment, EventID2, ObjectClass, ToolClass, OObj, TObj),
    % these two should overlap, with the spatula-pancakemaker coming first
    comp_overlapsI(EventID, EventID2),
    % select start and end as union
    simact_start(Experiment, EventID, Start),
    simact_end(Experiment, EventID2, End).

%% Gives a new interval, which is a subset of contactSpatula-Liquid. This is only the time during
%% which the liquid is in contact with the spatula and not in contact with the pancakemaker (Note: pancakemaker is not a object-supportingFurniture in the ontology so can't use supportedby here).
%% 
%% Example call:
%% > simflip_fliponly(knowrob_sim:'LiquidTangibleThing', knowrob_sim:'Spatula', knowrob_sim:'PancakeMaker', Start, End, OObj, TObj, LObj).
simflip_fliponly(Experiment, ObjectClass, ToolClass, LocationClass, Start, End, OObj, TObj, LObj) :-
    % get contactInterval spatula-pancakemaker
    simact_contact(Experiment, EventID, ToolClass, LocationClass, TObj, LObj),
    %don't get the owlNamedIndividual classes
    ToolClass\=LocationClass,
    simact_contact(Experiment, EventID2, ObjectClass, LocationClass, OObj, LObj),
    %% findall(EventID2, (simact_contact(Experiment, EventID2, ObjectClass, LocationClass, OObj, LObj), not(comp_temporallySubsumes(EventID2, EventID))), Candidates),
    LocationClass\=ObjectClass,
    %% [EventID2a|EventIDs] = Candidates,
    % select start and end as intersection
    comp_beforeI(EventID,EventID2),
    simact_end(Experiment,EventID, Start),
    simact_start(Experiment,EventID2, End).

%%  simsupported(?Experiment, ?EventID, ?ObjectInstance) is 
%   Body of the function for supported_during
%
%   Returns the EventID of the events in which ObjectInstance was supported by supporting Object-SupportingFurniture
%   Assumes that supportedby is a TouchingSituation.
%
simsupported(Experiment, EventID, ObjectInstance) :-
    simact(Experiment, EventID, knowrob_sim:'TouchingSituation'),
    rdf_has(EventID, knowrob_sim:'inContact', ObjectInstance),
    rdf_has(EventID, knowrob_sim:'inContact', ObjectInstance2),
    ObjectInstance \= ObjectInstance2,
    sim_class_individual(Obj2Class, ObjectInstance2),
    rdf_reachable(Obj2Class, rdfs:subClassOf, knowrob:'Object-SupportingFurniture').

test(Arr) :-
    jpl_new('org.knowrob.vis.MarkerVisualization', [], Canvas),
    jpl_list_to_array(['1','2','3','4'], Arr),
    jpl_call(Canvas, 'showAverageTrajectory', [bla, Arr, Arr, 1, 1], _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Interval handling
% 

%% 
%   Given a start/endtime value, looks at given list of time intervals and subtracts
%   these if they overlap with the given start/endtime interval. Returns start/endtime values
%   that do not include any of those intervals in the list.
%
%If Start equals End, it means that no difference set could be found 
%Bottom case; unify temporary start and end with the result
interval_setdifference(_, Start, End, [], Start, End).
%If the event is identical to the interval we started with, there cannot be any difference left, so the call will fail.
interval_setdifference(Experiment, Start, End, [EventID2|Tail], ResStart, ResEnd) :-
    simact_start(Experiment,EventID2, Start2),
    simact_end(Experiment,EventID2, End2),
    Start == Start2,
    End == End2,!, fail.
%If the head overlaps at the beginning
interval_setdifference(Experiment, Start, End, [EventID2|Tail], ResStart, ResEnd) :-
    simact_start(Experiment,EventID2, Start2),
    simact_end(Experiment,EventID2, End2),
    sim_timepoints_overlap(Start, End, Start2, End2),
    End \= End2,
    interval_setdifference(Experiment, End2, End, Tail, ResStart, ResEnd), !.
%If the head overlaps at the end
interval_setdifference(Experiment, Start, End, [EventID2|Tail], ResStart, ResEnd) :-
    simact_start(Experiment,EventID2, Start2),
    simact_end(Experiment,EventID2, End2),
    sim_timepoints_overlap_inv(Start, End, Start2, End2),
    Start \= Start2,
    interval_setdifference(Experiment, Start, Start2, Tail, ResStart, ResEnd), !.
%if there is no overlap, we don't care about the event in the current head and leave Start and End the way it was.
interval_setdifference(Experiment, Start, End, [_|Tail], ResStart, ResEnd) :-
    not((Start==ResStart, End==ResEnd)),
    interval_setdifference(Experiment, Start, End, Tail, ResStart, ResEnd), !.
    

%% Similar to the comp_overlapsI predicate but works with separate timepoints 
%% True if I2 overlaps with I1 at the beginning
%% Called by: interval_setdifference
sim_timepoints_overlap(Start1, End1, Start2, End2) :-
    time_term(Start1, SVal1),
    time_term(End1, EVal1),
    time_term(Start2, SVal2),
    time_term(End2, EVal2),
    SVal2 =< SVal1, %Start2 is before Start1
    EVal2 > SVal1, %End2 is after Start1
    EVal2 =< EVal1. %End2 ends before End1
%% True if I2 overlaps with I1 at the end
sim_timepoints_overlap_inv(Start1, End1, Start2, End2) :-
    sim_timepoints_overlap(Start2, End2, Start1, End1).

sim_subsumes(Start1,End1, Start2, End2):-
    time_term(Start1, SVal1),
    time_term(End1, EVal1),
    time_term(Start2, SVal2),
    time_term(End2, EVal2),
    SVal1 =< EVal1, %check this interval is ok (start comes before end)
    SVal2 =< EVal2, %check this interval is ok (start comes before end)
    SVal2 >= SVal1, %Start2 is after Start1
    EVal2 =< EVal1. %End2 is before End1

%%  supported_during(?Experiment, +EventID, ?EventID2, ?ObjectInstance)
%
%   helpfunction for simlift (and its variants)
%   returns true if Object in Event1 was supported by another object in Event2, and
%   Event1 is entirely contained in Event2 (e.g. Object was supported by something during Event1)
%   For example, if the Cup was grasp and during the entire grasping interval, there was also
%   contact with the kitchen table.
%
%   NOTE: nothing in this function checks that ObjectInstance is involved in EventID, 
%   This depends on the function calling supported_during
%
supported_during(Experiment, EventID, EventID2, ObjectInstance) :-
    simsupported(Experiment, EventID2, ObjectInstance),
    EventID\=EventID2,
    comp_temporallySubsumes(EventID2, EventID). 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Temporal stuff: start, end, duration of a simact
%

%% simact_start(?Task, ?Start) is nondet.
%
%  Check if Start is the start time of Task
%
%  @param Task Identifier of given Task
%  @param Start Identifier of given Start
% 
simact_start(Experiment, Event, Start) :-
    rdf_has(Event, knowrob:'startTime', Start),
    simact(Experiment, Event).


%% simact_end(?Task, ?End) is nondet.
%
%  Check if End is the end time of Task
%
%  @param Task Identifier of given Task
%  @param End Identifier of given End
% 
simact_end(Experiment, Event, End) :-
    rdf_has(Event, knowrob:'endTime', End),
    simact(Experiment,Event).

%% Note: To get the duration, just call comp_duration(+Task, -Duration) from the comp_temporal 
%% package. Make sure it's registered though (register_ros_package)


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Goals, success, failure
%

%% successful_simacts_for_goal(+Goal, -Tasks) is nondet.
%
% Finds all Tasks that successsfully achieved Goal, i.e. without failure.
% 
% @param Goal  Identifier of the goal to be searched for
% @param Tasks List of simacts that successfully accomplished the Goal 
% 
successful_simacts_for_goal(Goal, Tasks) :-
     findall(T, (simact_goal(T, Goal)), Ts),
     findall(FT, ((simact_goal(FT, Goal), rdf_has(FT, knowrob:'caughtFailure', _F))), FTs),
     subtract(Ts, FTs, Tasks).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Objects (and robot parts) and their locations
%

add_object_as_semantic_instance(Obj, Matrix, Time, ObjInstance) :-
    add_object_to_semantic_map(Obj, Matrix, Time, ObjInstance, 0.2, 0.2, 0.2).

add_robot_as_basic_semantic_instance(PoseList, Time, ObjInstance) :-
    add_object_to_semantic_map(Time, PoseList, Time, ObjInstance, 0.5, 0.2, 0.2).


add_object_to_semantic_map(Obj, PoseList, Time, ObjInstance, H, W, D) :-
    is_list(PoseList),
    create_pose(PoseList, Matrix),
    add_object_to_semantic_map(Obj, Matrix, Time, ObjInstance, H, W, D).

add_object_to_semantic_map(Obj, Matrix, Time, ObjInstance, H, W, D) :-
    atom(Matrix),
    rdf_split_url(_, ObjLocal, Obj),
    atom_concat('http://knowrob.org/kb/cram_log.owl#Object_', ObjLocal, ObjInstance),
    rdf_assert(ObjInstance, rdf:type, knowrob:'SpatialThing-Localized'),
    rdf_assert(ObjInstance,knowrob:'depthOfObject',literal(type(xsd:float, D))),
    rdf_assert(ObjInstance,knowrob:'widthOfObject',literal(type(xsd:float, W))),
    rdf_assert(ObjInstance,knowrob:'heightOfObject',literal(type(xsd:float, H))),
    rdf_assert(ObjInstance,knowrob:'describedInMap','http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap_PM580j'), % TODO: give map as parameter

    rdf_instance_from_class(knowrob:'SemanticMapPerception', Perception),
    rdf_assert(Perception, knowrob:'startTime', Time),
    rdf_assert(Perception, knowrob:'eventOccursAt', Matrix),

    set_object_perception(ObjInstance, Perception).
