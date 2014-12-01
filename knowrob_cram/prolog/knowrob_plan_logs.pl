
/** <module> knowrob_plan_logs

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


:- module(knowrob_plan_logs,
    [
        load_experiment/1,
        belief_at/2,
        cram_holds/2,
        occurs/2,
        task/1,
        task_type/2,
        task_goal/2,
        task_start/2,
        task_end/2,
        task_action/2,
        subtask/2,
        subtask_all/2,
        task_outcome/2,
        task_failure/2,
        failure_type/2,
        failure_attribute/3,
        successful_tasks_for_goal/2,
        task_used_gripper/2,
        show_image/1,
        image_of_perceived_scene/1,
        add_object_to_semantic_map/7,
        add_object_as_semantic_instance/4,
        add_robot_as_basic_semantic_instance/3,
        get_designator/2,
        publish_designator/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
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
    load_experiment(+),
    task(r),
    task_type(r,r),
    subtask(r,r),
    subtask_all(r,r),
    task_goal(r,r),
    task_start(r,r),
    task_end(r,r),
    task_action(r,r),
    belief_at(?,r),
    occurs(+,r),
    cram_holds(r,+),
    task_outcome(r,r),
    failure_type(r,r),
    task_failure(r,r),
    failure_attribute(r,r,r),
    task_used_gripper(+,-),
    show_image(r),
    image_of_perceived_scene(r),
    add_object_as_semantic_instance(+,+,+,-),
    add_object_as_semantic_instance(+,+,-),
    add_object_to_semantic_map(+,+,+,-,+,+,+),
    successful_tasks_for_goal(+,-),
    publish_designator(+),
    get_designator(r,-).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% 
% Experiment management: loading files
%


%% load_experiment(+Path) is nondet.
%
%  Loads the logfile of the corresponding CRAM plan execution. Also,
%  asserts a new DirectoryName instance for accessing perception images of plans
%  from the directory that the logfile resides.
%
%  @param Path file path of the logfile
% 
load_experiment(Path) :-

    owl_parse(Path),

    atom_concat('/home/ros/', LocalPath, Path),
    file_directory_name(LocalPath, Dir),
    atomic_list_concat(['http://knowrob.org/kb/knowrob.owl', Dir], '#', NameInstance),
    rdf_assert(NameInstance, rdf:type, knowrob:'DirectoryName').




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Basic task hierarchy handling
% 


%% task(?Task) is nondet.
%
%  Check if  class of Task is a subclass of CRAMEvent
%
%  @param Task Identifier of given Task
% 
task(Task) :-
    rdf_has(Task, rdf:type, A),
    rdf_reachable(A, rdfs:subClassOf, knowrob:'CRAMEvent').


%% task_type(?Task, ?Class) is nondet.
%
%  Check if Task is an instance of Class
%
%  @param Task Identifier of given Task
%  @param Class Identifier of given Class
% 
task_type(Task, Class) :-
    rdf_has(Task, rdf:type, Class),
    rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMEvent').


%% subtask(?Task, ?Subtask) is nondet.
%
%  Check if Subtask is a child task of Task in the task tree
%
%  @param Task Identifier of given Task
%  @param Subtask Identifier of given Subtask
% 
subtask(Task, Subtask) :-
    rdf_has(Task, knowrob:'subAction', Subtask),
    task(Task),
    task(Subtask).


%% subtask_all(?Task, ?Subtask) is nondet.
%
%  Check if Task is an ancestor of Subtask in the task tree
%
%  @param Task Identifier of given Task
%  @param Subtask Identifier of given Subtask
% 
subtask_all(Task, Subtask) :-
    owl_has(Task, knowrob:subAction,  Subtask).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Temporal stuff: start, end, duration of a task
%


%% task_start(?Task, ?Start) is nondet.
%
%  Check if Start is the start time of Task
%
%  @param Task Identifier of given Task
%  @param Start Identifier of given Start
% 
task_start(Task, Start) :-
    rdf_has(Task, knowrob:'startTime', Start),
    task(Task).


%% task_end(?Task, ?End) is nondet.
%
%  Check if End is the end time of Task
%
%  @param Task Identifier of given Task
%  @param End Identifier of given End
% 
task_end(Task, End) :-
    rdf_has(Task, knowrob:'endTime', End),
    task(Task).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Task actions
%

%% task_action_exp(?Task, ?Query) is nondet.
%
% Returns the task of the given action query.
%
% @param Task Identifier of the result
% @param Query Identifier of given Action query
% @param DesiredType Identifier of the task type
% 

task_action_exp(Action, [perform, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMPerform').

task_action_exp(Action, [achieve, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMAchieve').

task_action_exp(Action, [peceive, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMPerceive').

task_action_exp(Action, [failure, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMFailure').

task_action_exp(Action, [maintain, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMMaintain').

task_action_exp(Action, [monitor, QueryPattern]) :-
    task_action_exp(Action, QueryPattern, knowrob:'CRAMMonitor').

task_action_exp(Task, Query, DesiredType) :-
    action_designator_exp(Action, Query, DesiredType),
    task_action(Task, Action, DesiredType).
    
%% task_action(?Task, ?Action) is nondet.
%
% Returns the task of the given action.
%
% @param Task Identifier of the result
% @param Action Identifier of given Action
% @param DesiredType Identifier of the task type
% 

task_action(Task, Action) :-
    task_action(Task, Action, knowrob:'CRAMAction').
    
task_action(Task, Action, DesiredType) :-
    subtask(Parent, Action),
    rdf_has(Parent, rdf:type, Type),
    ( rdf_reachable(Type, rdfs:subClassOf, DesiredType) ->
      true ; task_action(Task, Parent)
    ).

%% action_designator_exp(?Task, ?Query, ?DesiredType) is nondet.
%
% Returns the task of the given action query.
%
% @param Task Identifier of the result
% @param Query Identifier of given Action query
% @param DesiredType Identifier of the task type
% 

action_designator_exp(Action, [perform, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMPerform').

action_designator_exp(Action, [achieve, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMAchieve').

action_designator_exp(Action, [peceive, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMPerceive').

action_designator_exp(Action, [failure, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMFailure').

action_designator_exp(Action, [maintain, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMMaintain').

action_designator_exp(Action, [monitor, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern, knowrob:'CRAMMonitor').

action_designator_exp(Action, QueryPattern, DesiredType) :-
    % Query mongo for matching designators
    mng_desig_matches(D, QueryPattern),
    % Query for action that references given designator
    rdf_has(Action, knowrob:'designator', D),
    % Make sure this action is a subaction of an action with matching type
    task_action(_, Action, DesiredType).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Goals, success, failure
%

%% task_goal(?Task, ?Goal) is nondet.
%
%  Check if Goal is the goal of Task
%
%  @param Task Identifier of given Task
%  @param Goal Identifier of given Goal
% 
task_goal(Task, Goal) :-
    task(Task),
    rdf_has(Task, knowrob:'taskContext', literal(type(_, Goal)));

    task(Task),
    rdf_has(Task, knowrob:'goalContext', literal(type(_, Goal))).


%% task_failure(?Task, ?Failure) is nondet.
%
%  Check if Failure has occurred in the context of Task
%
%  @param Failure Identifier of given Failure
%  @param Task Identifier of given Task
% 
task_failure(Task, Failure) :-
    task(Task),
    rdf_has(Task, knowrob:'eventFailure', Failure).


%% failure_type(?Failure, ?Class) is nondet.
%
%  Check if Failure is an instance of Class
%
%  @param Failure Identifier of given Failure
%  @param Class Identifier of given Class
% 
failure_type(Failure, Class) :-
    rdf_has(Failure, rdf:type, Class),
    rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMFailure').


%% task_outcome(?Task, ?Obj) is nondet.
%
% Returns the result of the given task.
%
% @param Task Identifier of given Task
% @param Obj Identifier of the result
% 
task_outcome(Task, Obj) :-
    rdf_has(Task, rdf:type, knowrob:'UIMAPerception'),
    rdf_has(Task, knowrob:'perceptionResult', Obj);

    task(Task),
    task_failure(Obj, Task).

%% failure_attribute(?Failure, ?AttributeName, ?Value) is nondet.
%
%  Check if Failure has the given attribute with the given value
%
%  @param Failure Identifier of given Failure
%  @param Task Identifier of given Task
% 
failure_attribute(Failure,AttributeName,Value) :-
    rdf_has(Failure, AttributeName, Value).


%% successful_tasks_for_goal(+Goal, -Tasks) is nondet.
%
% Finds all Tasks that successsfully achieved Goal, i.e. without failure.
% 
% @param Goal  Identifier of the goal to be searched for
% @param Tasks List of tasks that successfully accomplished the Goal 
% 
successful_tasks_for_goal(Goal, Tasks) :-
     findall(T, (task_goal(T, Goal)), Ts),
     findall(FT, ((task_goal(FT, Goal), rdf_has(FT, knowrob:'caughtFailure', _F))), FTs),
     subtract(Ts, FTs, Tasks).



%% cram_holds(+taskPredObj, -taskPredStatus, +T) is nondet.
%
% Check whether the given task was being continued, done, failed or not yet started at the given time point.
%
% @param Task Identifier of given Task
% @param Status Returned status
% @param T   TimePoint
% 
cram_holds(task_status(Task, Status), T):-
    nonvar(Task),
    task(Task),
    task_start(Task, Start),
    task_end(Task, End),
    
    ((rdf_triple(knowrob:after, Start, T)) ->   % Start < T
      (
        ((rdf_triple(knowrob:after, T, End)) ->
        (Status = ['Continue']);                % Start < T < End
        (Status = ['Done']))                    % Start < End < T
      )
      ; (                                       % T < Start
        ((rdf_triple(knowrob:after, T, End)) -> % T < End
          (Status = ['NotStarted']);            % T < Start, T < End
          (Status = ['Error']))                 % T < Start, T > End
        )
    ).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Objects (and robot parts) and their locations
%



%% belief_at(?locPredObj, -locPredLoc, +Time) is nondet.
%
% Check what the belief of the robot for location of Object at given Time .
%
% @param Desig    Identifier of an object designator
% @param Location Pose matrix identifier
% @param Time     TimePoint
% 
belief_at(loc(Desig,Loc), _Time) :-

% MT: commented this -- would maybe be relevant for objects, but since we
%     directly give designators as values, this is redundant.
% 
%     findall(End-Tsk, (
%                 task_type(Tsk, knowrob:'UIMAPerception'),
%                 task_end(Tsk, EndTp),
%                 rdf_triple(knowrob:after, EndTp, Time), % only consider tasks that end before Time
%                 time_point_value(EndTp, End)
%             ), Tsks),
% 
%     keysort(Tsks, TsksSorted),
%     last(TsksSorted, _-T),

    task_outcome(T, Desig),

    (image_of_perceived_scene(T);true), !,
    get_designator(Desig, Loc).


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
    task_type(Task, knowrob:'UIMAPerception'),
    task_outcome(Task, Obj),
    task_start(Task, T).


%% task_used_gripper(+Task, -Link) is nondet.
%
%
% @param Task Instance of an Action for which the arm is to be determined
% @param Link Identifier of a tf link denoting the arm
% 
task_used_gripper(Task, Link) :-

    subtask_all(Task, Movement),
    task_type(Movement, knowrob:'ArmMovement'),
    rdf_has(Movement, knowrob:'voluntaryMovementDetails', Designator),

    jpl_new('org.knowrob.cram.LogdataPublisher', [], Client),
    jpl_list_to_array(['org.knowrob.cram.LogdataPublisher'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Client, Arr], _),
    jpl_call(Client, 'getArmLink', [Designator], Link).



     

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Designator stuff
% 

:- assert(log_pbl(fail)).
log_publisher(Pbl) :-
    log_pbl(fail),
    jpl_new('org.knowrob.cram.LogdataPublisher', [], Pbl),
    jpl_list_to_array(['org.knowrob.cram.LogdataPublisher'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Pbl, Arr], _),
    retract(log_pbl(fail)),
    assert(log_pbl(Pbl)),!.
log_publisher(Pbl) :-
    log_pbl(Pbl).


publish_designator(Task) :-
    subtask(Task, Subtask),
    rdf_has(Subtask, knowrob:'designator', D),
    rdf_has(D, knowrob:'successorDesignator', D1),
    log_publisher(Client),
    jpl_call(Client, 'publishDesignator', [D1], _R).

get_designator(Designator, Loc) :-
    log_publisher(Client),
    jpl_call(Client, 'getBeliefByDesignator', [Designator], Localization_Array),
    jpl_array_to_list(Localization_Array, LocList),
    create_pose(LocList, Loc).



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
    


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Image stuff
%

% publish the image to Knowrob Web tool's topic
show_image(Path) :-
    log_publisher(Client),
    jpl_call(Client, 'publishImage', [Path], _R).

% Get the path of percepted image from the given perception task
image_of_perceived_scene(T) :-
    task(T),
    rdf_has(T, knowrob:'capturedImage', Img),
    rdf_has(Img, knowrob:'linkToImageFile', PathName),
    PathName = literal(type(_A, Path)),

    rdf_has(Directory, rdf:type, knowrob:'DirectoryName'),
    atomic_list_concat([_Prefix, Dir], '#', Directory),
    atomic_list_concat([Dir, Path], '/', CompletePath),
    show_image(CompletePath).
