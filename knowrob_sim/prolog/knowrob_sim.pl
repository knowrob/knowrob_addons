
/** <module> knowrob_sim

  Copyright (C) 2013 by Asil Kaan Bozcuoglu and Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Asil Kaan Bozcuoglu, Moritz Tenorth
@license GPL
*/


:- module(knowrob_sim,
    [
        simact/1,
        simact/2,
        simact_contact/3,
        simact_contact/4,
        simact_contact_specific/3,
        simact_contact_specific/4,
        simlift/2,
        simlift_specific/2,
        simlift_liftonly/4,
        supported_during/3,
        simact_start/2,
        simact_end/2,
        subact/2,
        subact_all/2,
        simact_outcome/2,
        successful_simacts_for_goal/2
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_sim, 'http://knowrob.org/kb/knowrob_sim.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate cram_holds(0, ?, ?).
:- discontiguous cram_holds/2.

:- meta_predicate occurs(0, ?, ?).
:- discontiguous occurs/2.

:- meta_predicate belief_at(0, ?, ?).
:- discontiguous belief_at/1.


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    simact(r),
    simact(r,r),
    simact_contact(r,r,r),
    simact_contact(r,r,r,r),
    simact_contact_specific(r,r,r),
    simact_contact_specific(r,r,r,r),
    simlift(r,r),
    simlift_specific(r,r),
    simlift_liftonly(r,r,r,r),
    supported_during(r,r,r),
    subact(r,r),
    subact_all(r,r),
    simact_start(r,r),
    simact_end(r,r),
    simact_outcome(r,r),
    successful_simacts_for_goal(+,-).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Basic simact handling
% 


%% simact(?Task) is nondet.
%
%  Check if class of Task is a subclass of SimulationEvent, or looks for Tasks that are subclass of SimulationEvent
%
%  @param Task Identifier of given Task
% 
simact(Event) :-
    rdf_has(Event, rdf:type, EventClass),
    rdf_reachable(EventClass, rdfs:subClassOf, knowrob_sim:'SimulationEvent').


%% simact_type(?Event, ?EventClass) is nondet.
%
%  Check if Task is an instance of Class or looks for Tasks of Class that are subclass of SimulationEvent
%  For example: simact_type(T, knowrob_sim:'TouchingSituation').
%
%  @param Event Identifier of given Event
%  @param EventClass Identifier of given EventClass
% 
simact(EventID, EventClass) :-
    rdf_has(EventID, rdf:type, EventClass),
    rdf_reachable(EventClass, rdfs:subClassOf, knowrob_sim:'SimulationEvent').


%% Find a certain event involving a certain object type
simact_contact(Event, EventClass, ObjectClass) :-
    simact(Event, EventClass),
    rdf_has(ObjectInstance, rdf:type, ObjectClass),
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance).

simact_contact_specific(Event, EventClass, ObjectInstance) :-
    simact(Event, EventClass),
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance).

%% Find a certain event involving only certain object types
simact_contact(Event, EventClass, Object1Class, Object2Class) :-
    simact(Event, EventClass),
    rdf_has(ObjectInstance1, rdf:type, Object1Class),
    rdf_has(ObjectInstance2, rdf:type, Object2Class),
    ObjectInstance1\=ObjectInstance2,
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance1),
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance2).

%% Find a certain event involving only certain objects
simact_contact_specific(Event, EventClass, ObjectInstance1, ObjectInstance2) :-
    simact(Event, EventClass),
    ObjectInstance1\=ObjectInstance2,
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance1),
    rdf_has(Event, knowrob:'objectInContact', ObjectInstance2).


%% ignore these predicates for now, tried to change color of successive calls, doesn't work right now but is also not so important
add_count :- 
    nb_getval(counter, C), CNew is C + 0.1, nb_setval(counter, CNew).

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
subact(Event, SubEvent) :-
    rdf_has(Event, knowrob:'subAction', SubEvent),
    simact(Event),
    simact(SubEvent).


%% subsimact_all(?Task, ?Subsimact) is nondet.
%
%  Check if Task is an ancestor of Subsimact in the simact tree
%
%  @param Task Identifier of given Task
%  @param Subsimact Identifier of given Subsimact
% 
subact_all(Event, SubEvent) :-
    owl_has(Event, knowrob:subAction,  SubEvent).

%% Find event interval during which a specific object type is lifted
%% Example: simlift(E, knowrob_sim:'TouchingSituation',knowrob:'Cup').
simlift(EventID, ObjectClass) :-
    simact(EventID, knowrob_sim:'TouchingSituation'),
    rdf_has(EventID, knowrob:'GraspingSomething', ObjectInstance), %for a lift to occur, the event in which the object participates must involve GraspingSomething (lift can only happen while the object is grasped)
    rdf_has(ObjectInstance, rdf:type, ObjectClass),
    not(supported_during(EventID, _, ObjectInstance)). %check that this specific object is not in a contact relation with a supporting object for at least part of the interval (the interval will overlap at least with some contact intervals, because for example when you lift the mug, it will still be in contact with the table while the hand initiates contact). 
    %TODO: could define a new interval that only gives the path from where the object leaves the supporting surface until it touches it again.

%% Find event interval during which a specific object is lifted
%% Example: simlift(E, knowrob_sim:'TouchingSituation',knowrob:'Cup').
simlift_specific(EventID, ObjectInstance) :-
    simact(EventID, knowrob_sim:'TouchingSituation'),
    rdf_has(EventID, knowrob:'GraspingSomething', ObjectInstance),
    not(supported_during(EventID, _,ObjectInstance)).

%% Gives a new interval (start and endtimes) during which the specified object is lifted
%% Differs from simlift because simlift can only return existing event intervals, and some
%% overlap with supportedby intervals are inevitable, while simlift_liftonly "defines" a new
%% interval by looking for the difference between grasping intervals and all supportedby 
%%intervals.
%%
%% TODO: I'm not sure whether it only returns one liftInterval now, or whether it only doesn't
%% backtrack into interval_setdifference, which was my intention because it shouldn't go back.
%% It should give multiple results if more than one EventID is found however, and I think the
%% cut prevents that too.
simlift_liftonly(EventID, ObjectClass, Start, End) :-
    simact(EventID, knowrob_sim:'TouchingSituation'),
    simact_start(EventID, TempStart),
    simact_end(EventID, TempEnd),
    rdf_has(EventID, knowrob:'GraspingSomething', ObjectInstance), %for a lift to occur, the event in which the object participates must involve GraspingSomething (lift can only happen while the object is grasped)
    rdf_has(ObjectInstance, rdf:type, ObjectClass),
    findall(EventID2, (simsupported(EventID2, ObjectInstance), not(comp_temporallySubsumes(EventID2, EventID))), Candidates),
    writeln(Candidates),
    interval_setdifference(TempStart, TempEnd, Candidates, Start, End),!.

%Bottom case; unify temporary start and end with the result
interval_setdifference(Start, End, [], Start, End).    
%If the head overlaps at the beginning
interval_setdifference(Start, End, [EventID2|Tail], ResStart, ResEnd) :-
    simact_start(EventID2, Start2),
    simact_end(EventID2, End2),
    sim_timepoints_overlap(Start, End, Start2, End2),
    interval_setdifference(End2, End, Tail, ResStart, ResEnd).
%If the head overlaps at the end
interval_setdifference(Start, End, [EventID2|Tail], ResStart, ResEnd) :-
    simact_start(EventID2, Start2),
    simact_end(EventID2, End2),
    sim_timepoints_overlap_inv(Start, End, Start2, End2),
    interval_setdifference(Start, Start2, Tail, ResStart, ResEnd).
%if there is no overlap, so we don't care about the current head
interval_setdifference(Start, End, [_|Tail], ResStart, ResEnd) :-
    interval_setdifference(Start, End, Tail, ResStart, ResEnd).

%% Similar to the comp_overlapsI predicate but works with separate timepoints 
%% True if I2 overlaps with I1 at the beginning
%% Called by: interval_setdifference
sim_timepoints_overlap(Start1, End1, Start2, End2) :-
    time_point_value(Start1, SVal1),
    time_point_value(End1, EVal1),
    time_point_value(Start2, SVal2),
    time_point_value(End2, EVal2),
    SVal2 < SVal1, %Start2 is before Start1
    EVal2 > SVal1, %End2 is after Start1
    EVal2 < EVal1. %End2 ends before End1
sim_timepoints_overlap_inv(Start1, End1, Start2, End2) :-
    sim_timepoints_overlap(Start2, End2, Start1, End1).

%% Returns the EventID of the events in which ObjectInstance was supported by supporting Object-SupportingFurniture
%% Assumes that supportedby is a TouchingSituation.
simsupported(EventID, ObjectInstance) :-
    simact(EventID, knowrob_sim:'TouchingSituation'),
    rdf_has(EventID, knowrob:'objectInContact', ObjectInstance),
    rdf_has(EventID, knowrob:'objectInContact', ObjectInstance2),
    rdf_has(ObjectInstance2, rdf:type, Obj2Class),
    rdf_reachable(Obj2Class, rdfs:subClassOf, knowrob:'Object-SupportingFurniture').

%% returns true if Object in Event1 was supported by another object in Event2, and
%% Event1 is entirely contained in Event2 (e.g. Object was supported by something during Event1)
supported_during(EventID, EventID2, ObjectInstance) :-
    simsupported(EventID2, ObjectInstance),
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
simact_start(Task, Start) :-
    rdf_has(Task, knowrob:'startTime', Start),
    simact(Task).


%% simact_end(?Task, ?End) is nondet.
%
%  Check if End is the end time of Task
%
%  @param Task Identifier of given Task
%  @param End Identifier of given End
% 
simact_end(Task, End) :-
    rdf_has(Task, knowrob:'endTime', End),
    simact(Task).

%% Note: To get the duration, just call comp_duration(+Task, -Duration) from the comp_temporal 
%% package. Make sure it's registered though (register_ros_package)


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Goals, success, failure
%

%% simact_goal(?Task, ?Goal) is nondet.
%
%  Check if Goal is the goal of Task
%
%  @param Task Identifier of given Task
%  @param Goal Identifier of given Goal
% 
simact_subaction(Subaction, Type) :-
    simact(Task),
    rdf_has(Task, knowrob:'simactContext', literal(type(_, Goal))).

%% simact_outcome(?Task, ?Obj) is nondet.
%
% Returns the result of the given simact.
%
% @param Task Identifier of given Task
% @param Obj Identifier of the result
% 
simact_outcome(Task, Obj) :-
    rdf_has(Task, rdf:type, knowrob:'UIMAPerception'),
    rdf_has(Task, knowrob:'perceptionResult', Obj);

    simact(Task),
    simact_failure(Obj, Task).

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