
/** <module> comp_spatial

  This module contains all computables that calculate qualitative spatial relations
  between objects to allow for spatial reasoning. In addition, there are computables
  to extract components of a matrix or position vector.


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


:- module(comp_execution_trace,
    [
        load_experiment/1,
     	task/1,
	task_class/2,
	subtask/2,
	subtask_all/2,
      	task_goal/2,
	task_start/2,
	task_end/2,
	cram_holds/2,
	returned_value/2,
	belief_at/2,
	occurs/2,
	duration_of_a_task/2,
	failure_class/2,
	failure_task/2,
	failure_attribute/3,
	show_image/1,
	image_of_percepted_scene/1,
	avg_task_duration/2,
	add_object_as_semantic_instance/4,
	arm_used_for_manipulation/2,
	add_robot_as_basic_semantic_instance/3,
	add_object_to_semantic_map/7,
	successful_instances_of_given_goal/2,
	publish_designator/1,
	get_designator/2
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://ias.cs.tum.edu/kb/knowrob_cram.owl#', [keep(true)]).

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
    load_experiment(+),
    task(r),
    task_class(r,r),
    subtask(r,r),
    subtask_all(r,r),
    task_goal(r,r),
    task_start(r,r),
    task_end(r,r),
    belief_at(r,+),
    occurs(r,+),
    duration_of_a_task(r,-),
    cram_holds(r,+),
    returned_value(r,r),
    failure_class(r,r),
    failure_task(r,r),
    failure_attribute(r,r,r),
    show_image(r),
    image_of_percepted_scene(r),
    avg_task_duration(r,-),
    add_object_as_semantic_instance(+,+,+,-),
    arm_used_for_manipulation(+,-),
    add_object_as_semantic_instance(+,+,-),
    add_object_to_semantic_map(+,+,+,-,+,+,+),
    successful_instances_of_given_goal(+,-),
    publish_designator(+),
    get_designator(+,-).


%% load_experiment(+Path) is nondet.
%
%  Loads the logfile of the corresponding CRAM plan execution. Also,
%  asserts a new DirectoryName instance for accessing perception images of plans
%  from the directory that the logfile resides.
%
%  @param Path file path of the logfile
load_experiment(Path) :-
    owl_parse(Path, false, false, true),
    atomic_list_concat([_Empty, _Var, _Roslog, Dir, _File],'/', Path),
    atomic_list_concat(['http://ias.cs.tum.edu/kb/knowrob.owl', Dir], '#', NameInstance),
    rdf_assert(NameInstance, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#DirectoryName').


%% task(?Task) is nondet.
%
%  Check if  class of Task is a subclass of CRAMEvent
%
%  @param Task Identifier of given Task
task(Task) :-
    rdf_has(Task, rdf:type, A),
    rdf_reachable(A, rdfs:subClassOf, knowrob:'CRAMEvent').


%% task_class(?Task, ?Class) is nondet.
%
%  Check if Task is an instance of Class
%
%  @param Task Identifier of given Task
%  @param Class Identifier of given Class
task_class(Task, Class) :-
    rdf_has(Task, rdf:type, Class),
    rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMEvent').


%% subtask(?Task, ?Subtask) is nondet.
%
%  Check if Subtask is a child task of Task in the task tree
%
%  @param Task Identifier of given Task
%  @param Subtask Identifier of given Subtask
subtask(Task, Subtask) :-
    task(Task),
    task(Subtask),
    rdf_has(Task, knowrob:'subAction', Subtask).


%% subtask_all(?Task, ?Subtask) is nondet.
%
%  Check if Task is an ancestor of Subtask in the task tree
%
%  @param Task Identifier of given Task
%  @param Subtask Identifier of given Subtask
subtask_all(Task, Subtask) :-
    subtask(Task, Subtask);

    nonvar(Task),
    subtask(Task, A),
    subtask_all(A, Subtask);


    nonvar(Subtask),
    subtask(A, Subtask),
    subtask_all(Task, A);


    var(Task),
    var(Subtask),
    subtask(Task, A),
    subtask_all(A, Subtask).


%% task_goal(?Task, ?Goal) is nondet.
%
%  Check if Goal is the goal of Task
%
%  @param Task Identifier of given Task
%  @param Goal Identifier of given Goal
task_goal(Task, Goal) :-
    task(Task),
    rdf_has(Task, knowrob:'taskContext', literal(type(_, Goal)));

    task(Task),
    rdf_has(Task, knowrob:'goalContext', literal(type(_, Goal))).


%% task_start(?Task, ?Start) is nondet.
%
%  Check if Start is the start time of Task
%
%  @param Task Identifier of given Task
%  @param Start Identifier of given Start
task_start(Task, Start) :-
    task(Task),
    rdf_has(Task, knowrob:'startTime', Start).


%% task_end(?Task, ?End) is nondet.
%
%  Check if End is the end time of Task
%
%  @param Task Identifier of given Task
%  @param End Identifier of given End
task_end(Task, End) :-
    task(Task),
    rdf_has(Task, knowrob:'endTime', End).


%% belief_at(?locPredObj, -locPredLoc, +Time) is nondet.
%
% Check what the belief of the robot for location of Object at given Time .
%
% @param Obj    Identifier of the Object
% @param Location Pose matrix identifier
% @param Time   TimePoint
belief_at(loc(Obj,Loc), Time) :-
    findall(
            BeliefTime,
            (
                task_class(Tsk, knowrob:'UIMAPerception'),
                task_end(Tsk, Bt),
                returned_value(Tsk, Obj),
                term_to_atom(Bt, BeliefTime)
            ),
            BeliefTimes
    ),

    jpl_new( '[Ljava.lang.String;', BeliefTimes, Bts),
    term_to_atom(Time, TConverted),
    perform_time_check2(Bts, TConverted, LastPerception),

    task_class(T, knowrob:'UIMAPerception'),
    task_end(T, LastPerception),
    returned_value(T, Obj),
    image_of_percepted_scene(T),
    image_of_percepted_scene(T), !,
    get_designator(Obj, Loc).


%% belief_at(?robotPredPart, -robotPredLoc, +Time) is nondet.
%
% Check what the belief of the robot for location of Robot part at given Time .
%
% @param Part    Identifier of the Part
% @param Location Pose matrix identifier
% @param Time   TimePoint
belief_at(robot(Part,Loc), Time) :-
    mng_lookup_transform('/map', Part, Time, Loc).


%% occurs(?objectPredObj, ?Time) is nondet.
%
% Check whether Object was perceived at given Time .
%
% @param Obj    Identifier of the Object
% @param Time   TimePoint
occurs(object_perceived(Obj),T) :-
    nonvar(Obj),
    nonvar(T),
    task_class(Task, knowrob:'UIMAPerception'),
    returned_value(Task, Obj),
    task_start(Task, T).


%% cram_holds(+taskPredObj, +taskPredStatus, -T) is nondet.
%
% Check whether the given task was being continued, done, failed or not yet started at the given time point.
%
% @param Task Identifier of given Task
% @param Status Returned status
% @param T   TimePoint
cram_holds(task_status(Task, Status), T):-
    nonvar(Task),
    task(Task),
    task_start(Task, Start),
    task_end(Task, End),
    perform_time_check(Start, T, Compare_Result1),
    perform_time_check(T, End, Compare_Result2),
    term_to_atom(Compare_Result1, c1),
    term_to_atom(Compare_Result2, c2),
    ((c1 is 1) -> (((c2 is 1) -> (Status = ['Continue']);(Status = ['Done'])));(((c2 is 1) -> (Status = ['Error']); (Status = ['NotStarted'])))).


%% cram_holds(+objectPlacedPredObj, +objectPlacedPredLoc, -T) is nondet.
%
% Check whether the given object was placed at given location at given time.
%
% @param Task Identifier of given Task
% @param Loc Identifier of given Location
% @param T   TimePoint
cram_holds(object_placed_at(Object, Loc), T):-
    perform_belief(Object, T, Actual_Loc),
    perform_time_check(Loc, Actual_Loc, Compare_Result),
    term_to_atom(Compare_Result, r),
    ((r is 0) -> (true);(false)).


%% returned_value(+Task, +Obj) is nondet.
%
% Returns the result of the given task.
%
% @param Task Identifier of given Task
% @param Obj Identifier of the result
returned_value(Task, Obj) :-
    rdf_has(Task, rdf:type, knowrob:'UIMAPerception'),
    rdf_has(Task, knowrob:'perceptionResult', Obj);

    task(Task),
    failure_task(Obj, Task).


%% failure_class(?Error, ?Class) is nondet.
%
%  Check if Error is an instance of Class
%
%  @param Error Identifier of given Error
%  @param Class Identifier of given Class
failure_class(Error, Class) :-
    rdf_has(Error, rdf:type, Class),
    rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMFailure').


%% failure_task(?Error, ?Task) is nondet.
%
%  Check if Error was occured in the context of Task
%
%  @param Error Identifier of given Error
%  @param Task Identifier of given Task
failure_task(Error, Task) :-
    task(Task),
    %failure_class(Error, Class),
    rdf_has(Task, knowrob:'eventFailure', Error).


%% failure_attribute(?Error, ?AttributeName, ?Value) is nondet.
%
%  Check if Error has the given attribute with the given value
%
%  @param Error Identifier of given Error
%  @param Task Identifier of given Task
failure_attribute(Error,AttributeName,Value) :-
    %failure_class(Error, Class),
    rdf_has(Error, AttributeName, Value).


%%% Utility functions %%%

% publish the image to Knowrob Web tool's topic
show_image(Path) :-
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),
    jpl_call(Client, 'publishImage', [Path], _R).

% Get the path of percepted image from the given perception task
image_of_percepted_scene(T) :-
    task(T),
    rdf_has(T, knowrob:'capturedImage', Img),
    rdf_has(Img, knowrob:'linkToImageFile', PathName),
    PathName = literal(type(_A, Path)),

    rdf_has(Directory, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#DirectoryName'),
    atomic_list_concat([_Prefix, Dir], '#', Directory),
    atomic_list_concat(['', 'var', 'roslog', Dir, Path], '/', CompletePath),
    show_image(CompletePath).

duration_of_a_task(T, Dur) :-
    task(T),
    rdf_triple(knowrob:duration, T, Dur).

avg_task_duration(ActionType, AvgDuration) :-

  findall(D, (owl_individual_of(A, ActionType),
              rdf_triple(knowrob:duration, A, D)), Ds),

  sumlist(Ds, Sum),
  length(Ds, Len),
  Len \= 0,
  AvgDuration is Sum/Len.

add_object_as_semantic_instance(Obj, Matrix, Time, ObjInstance) :-
    add_object_to_semantic_map(Obj, Matrix, Time, ObjInstance, 0.2, 0.2, 0.2).

add_robot_as_basic_semantic_instance(Matrix, Time, ObjInstance) :-
    add_object_to_semantic_map(Time, Matrix, Time, ObjInstance, 0.5, 0.2, 0.2).

add_object_to_semantic_map(Obj, Matrix, Time, ObjInstance, H, W, D) :-
    rdf_split_url(_, ObjLocal, Obj),
    atom_concat('http://ias.cs.tum.edu/kb/cram_log.owl#Object_', ObjLocal, ObjInstance),
    rdf_assert(ObjInstance, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#SpatialThing-Localized'),
    rdf_assert(ObjInstance,'http://ias.cs.tum.edu/kb/knowrob.owl#depthOfObject',literal(type(xsd:float, D))),
    rdf_assert(ObjInstance,'http://ias.cs.tum.edu/kb/knowrob.owl#widthOfObject',literal(type(xsd:float, W))),
    rdf_assert(ObjInstance,'http://ias.cs.tum.edu/kb/knowrob.owl#heightOfObject',literal(type(xsd:float, H))),
    rdf_assert(ObjInstance,'http://ias.cs.tum.edu/kb/knowrob.owl#describedInMap','http://ias.cs.tum.edu/kb/ias_semantic_map.owl#SemanticEnvironmentMap_PM580j'), % TODO: give map as parameter

    atom_concat('http://ias.cs.tum.edu/kb/cram_log.owl#SemanticMapPerception_', ObjLocal, SemanticMapInstance),
    rdf_assert(SemanticMapInstance, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#SemanticMapPerception'),
    rdf_assert(SemanticMapInstance, 'http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn', ObjInstance),
    rdf_assert(SemanticMapInstance, 'http://ias.cs.tum.edu/kb/knowrob.owl#eventOccursAt', Matrix),
    rdf_assert(SemanticMapInstance, 'http://ias.cs.tum.edu/kb/knowrob.owl#startTime', Time).

arm_used_for_manipulation(Task, Link) :-
    subtask_all(Task, Movement),
    task_class(Movement, knowrob:'ArmMovement'),
    rdf_has(Movement, knowrob:'voluntaryMovementDetails', Designator),

    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),
    jpl_call(Client, 'getArmLink', [Designator], Link).

successful_instances_of_given_goal(Goal, Tasks) :-	
     findall(T, (task_goal(T, Goal)), Ts),
     findall(FT, ((task_goal(FT, Goal), rdf_has(FT, knowrob:'caughtFailure', _F))), FTs),
     subtract(Ts, FTs, Tasks).	      

publish_designator(Task) :-
    subtask(Task, Subtask),
    rdf_has(Subtask, knowrob:'designator', D),
    rdf_has(D, knowrob:'successorDesignator', D1),
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),
    jpl_call(Client, 'publishDesignator', [D1], _R).

get_designator(Designator, Loc) :-
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),
    jpl_call(Client, 'getBeliefByDesignator', [Designator], Localization_Array),
    jpl_array_to_list(Localization_Array, LocList),
    create_pose(LocList, Loc).


