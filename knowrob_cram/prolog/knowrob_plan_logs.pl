
/** <module> knowrob_plan_logs

  Copyright (C) 2013-15 Moritz Tenorth, Asil Kaan Bozcuoglu, Daniel Beßler
  All rights reserved.

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

  @author Moritz Tenorth, Asil Kaan Bozcuoglu, Daniel Beßler
  @license BSD
*/

:- module(knowrob_plan_logs,
    [
        load_experiment/1,
        load_experiments/1,
        load_experiments/2,
        load_experiments/3,
        remember_at/2,
        event/3,
        event_before/3,
        event_after/3,
        event_name/2,
        event_class_name/2,
        event_class/2,
        experiment/1,
        experiment/2,
        experiment_map/2,
        experiment_map/3,
        task/1,
        task/2,
        task/3,
        task_type/2,
        task_type_name/2,
        task_goal/2,
        task_goal_inherited/2,
        task_start/2,
        task_end/2,
        task_duration/2,
        task_status/2,
        subtask/2,
        subtask_all/2,
        subtask_typed/3,
        task_outcome/2,
        task_failure/2,
        failure_type/2,
        failure_attribute/3,
        successful_tasks_for_goal/2,
        add_object_to_semantic_map/7,
        add_object_as_semantic_instance/4,
        add_robot_as_basic_semantic_instance/3
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_temporal')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

% :- meta_predicate remember_at(0, ?, ?).
:- discontiguous remember_at/1.


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    load_experiment(+),
    load_experiments(+),
    load_experiments(+,+),
    load_experiments(+,+,+),
    event(r,r,r),
    event_before(r,r,r),
    event_name(r,?),
    event_class_name(r,?),
    event_class(r,?),
    event_after(r,r,r),
    experiment(r),
    experiment(r,r),
    experiment_map(r,r),
    experiment_map(r,r,r),
    task(r),
    task(r,r),
    task(r,r,r),
    task_type(r,r),
    task_type_name(r,?),
    subtask(r,r),
    subtask_all(r,r),
    subtask_typed(r,r,r),
    task_goal(r,r),
    task_goal_inherited(r,r),
    task_start(r,r),
    task_end(r,r),
    task_duration(r,?),
    remember_at(?,r),
    task_outcome(r,r),
    failure_type(r,r),
    task_failure(r,r),
    failure_attribute(r,r,r),
    add_object_as_semantic_instance(+,+,+,-),
    add_object_as_semantic_instance(+,+,-),
    add_object_to_semantic_map(+,+,+,-,+,+,+),
    successful_tasks_for_goal(+,-).

% TODO: hack for review
default_map('http://knowrob.org/kb/saphari.owl#SemanticEnvironmentMap_FSf74Vd').
%default_map('http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap_PM580j').


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
  findall( E, rdf_has(E, rdf:type, knowrob:'RobotExperiment'), E0),
  owl_parse(Path),
  findall( E, rdf_has(E, rdf:type, knowrob:'RobotExperiment'), E1),
  member( Experiment, E1 ),
  
  ( not( member( Experiment, E0 ) ),
    % TODO(daniel): Use environment var instead
    atom_concat('/episodes/', LocalPath, Path),
    file_directory_name(LocalPath, Dir),
    atomic_list_concat(['http://knowrob.org/kb/knowrob.owl', Dir], '#', NameInstance),
    rdf_assert(NameInstance, rdf:type, knowrob:'RobotExperimentDirectory'),
    rdf_assert(NameInstance, knowrob:experiment, Experiment)
  ) ; true.

%% load_experiments(+Path) is nondet.
%
%  Loads the logfiles of the corresponding CRAM plan executions. Also,
%  asserts new DirectoryName instances for accessing perception images of plans.
%  It is assumed that Path contains sub directories for each experiment
%  run where a log file 'cram_log.owl' is located in each of the sub directories.
%
%  @param Path Parent directory for experiment logs.
% 
load_experiments(Path) :-
  load_experiments(Path, 'log.owl').

%% load_experiments(+Path, +ExpFileName) is nondet.
%
%  Loads the logfiles of the corresponding CRAM plan executions. Also,
%  asserts new DirectoryName instances for accessing perception images of plans.
%  It is assumed that Path contains sub directories for each experiment
%  run where a log file ExpFileName is located in each of the sub directories.
%
%  @param Path Parent directory for experiment logs.
%  @param ExpFileName Name of the log file.
% 
load_experiments(Path, ExpFileName) :-
  directory_files(Path, SubDirs),
  load_experiments(Path, SubDirs, ExpFileName).

%% load_experiments(+Path, +SubDirs, +ExpFileName) is nondet.
%
%  Loads the logfiles of the corresponding CRAM plan executions. Also,
%  asserts new DirectoryName instances for accessing perception images of plans.
%
%  @param Path Parent directory for experiment logs.
%  @param SubDirs List of subdirectory names.
%  @param ExpFileName Name of the log file.
% 
load_experiments(Path, SubDirs, ExpFileName) :-
  forall( member(SubDir, SubDirs), ((
    atom_concat(Path, SubDir, ExpDir),
    atom_concat(ExpDir, '/', ExpDirSlash),
    atom_concat(ExpDirSlash, ExpFileName, ExpFile),
    exists_file(ExpFile),
    load_experiment(ExpFile)
  ) ; true )).

%% experiment(?Experiment) is nondet.
%
% Yields all experiments
%
%  @param Experiment Experiment identifier
% 
experiment(Experiment) :-
  rdfs_individual_of(Experiment, knowrob:'RobotExperiment').

%% experiment(?Experiment, +Timepoint) is nondet.
%
% Yields experiments which were active during Timepoint
%
%  @param Experiment Experiment identifier
%  @param Time       Time when experiment was performed
% 
experiment(Experiment, Timepoint) :-
  occurs(Experiment, Timepoint, knowrob:'RobotExperiment').

%% experiment_map(?Experiment, ?Map, +Time) is nondet.
%
% Check the semantic map instance that corresponds to an experiment
%
%  @param Experiment Experiment identifier
%  @param Map        Semantic map identifier
%  @param Time       Time when experiment was performed
% 
experiment_map(Experiment, Map, Time) :-
  experiment(Experiment, Time),
  experiment_map(Experiment, Map), !.

experiment_map(_, Map, _) :-
  default_map(Map).

%% experiment_map(+Experiment, ?Map) is nondet.
%
% Check the semantic map instance that corresponds to an experiment
%
%  @param Experiment Experiment identifier
%  @param Map        Semantic map identifier
% 
experiment_map(Experiment, Map) :-
  rdf_has(Experiment, knowrob:'performedInMap', Map).

experiment_map(_, Map) :-
  % Fallback to lab kitchen for backwards compatibiliy
  default_map(Map).
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Event handling
% 

event(EventClass, EventInstance, Timepoint) :-
  rdfs_individual_of(EventInstance, EventClass),
  rdf_has(EventInstance, knowrob:'startTime', T0),
  rdf_has(EventInstance, knowrob:'endTime', T1),
  time_between(Timepoint, T0, T1).

event_before(EventClass, EventInstance, Timepoint) :-
  rdfs_individual_of(EventInstance, EventClass),
  rdf_has(EventInstance, knowrob:'endTime', T0),
  time_earlier_then(T0, Timepoint).

event_after(EventClass, EventInstance, Timepoint) :-
  rdfs_individual_of(EventInstance, EventClass),
  rdf_has(EventInstance, knowrob:'startTime', T0),
  time_later_then(T0, Timepoint).

event_name(EventInstance, EventName) :-
  rdf_split_url(_, EventName, EventInstance).

event_class_name(EventInstance, EventClass) :-
  rdf_has(EventInstance, rdf:'type', ClassUri),
  rdf_split_url(_, EventClass, ClassUri),
  not( EventClass = 'NamedIndividual' ).

event_class(EventInstance, ClassUri) :-
  rdf_has(EventInstance, rdf:'type', ClassUri),
  rdf_split_url(_, EventClass, ClassUri),
  not( EventClass = 'NamedIndividual' ).

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

%% task(?Task, ?Timepoint) is nondet.
%
%  Check if  class of Task is a subclass of CRAMEvent
%  and that the task is active at given time.
%
%  @param Task Identifier of given Task
%  @param Timepoint Identifier of given timepoint
% 
task(Task, Timepoint) :-
    task(Task),
    interval(Task, [Start,End]),
    time_between(Timepoint, Start, End).

%% task(?Task, ?Timepoint, ?SuperClass) is nondet.
%
%  Check if  class of Task is a subclass of CRAMEvent,
%  that the task is active at given time
%  and that the class of Task is a specialization of given super class.
%
%  @param Task Identifier of given Task
%  @param Timepoint Identifier of given timepoint
%  @param SuperClass Identifier of given super class
% 
task(Task, Timepoint, SuperClass) :-
    task(Task, Timepoint),
    task_type(Task, TaskClass),
    rdf_reachable(TaskClass, rdfs:subClassOf, SuperClass).

%% task_type(?Task, ?Class) is nondet.
%
%  Check if Task is an instance of Class
%
%  @param Task Identifier of given Task
%  @param Class Identifier of given Class
% 
task_type(Task, Class) :-
    rdf_has(Task, rdf:type, Class),
    rdf_reachable(Class, rdfs:subClassOf, knowrob:'Event').
    
task_type_name(Task, ClassName) :-
    task_type(Task, Class),
    rdf_split_url(_, ClassName, Class).


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

%% subtask_typed(?Task, ?Subtask, &Type) is nondet.
%
%  Check if there is a parent task with given type.
%
%  @param Task Identifier of given Task
%  @param Subtask Identifier of given Subtask
%  @param Type Identifier of given subtask type
% 

subtask_typed(Task, Subtask, perform) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMPerform').

subtask_typed(Task, Subtask, achieve) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMAchieve').

subtask_typed(Task, Subtask, perceive) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMPerceive').

subtask_typed(Task, Subtask, failure) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMFailure').

subtask_typed(Task, Subtask, maintain) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMMaintain').

subtask_typed(Task, Subtask, monitor) :-
    subtask_typed(Task, Subtask, 'http://knowrob.org/kb/knowrob.owl#CRAMMonitor').
    
subtask_typed(Task, Subtask, Type) :-
    rdf_has(X, knowrob:'subAction', Subtask),
    (  rdf_has(X, rdf:type, Type)
    -> Task = X
    ;  subtask_typed(Task, X, Type)
    ).

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
  rdfs_individual_of(Task, knowrob:'Event'),
  interval_start(Task, Start).


%% task_end(?Task, ?End) is nondet.
%
%  Check if End is the end time of Task
%
%  @param Task Identifier of given Task
%  @param End Identifier of given End
% 
task_end(Task, End) :-
  rdfs_individual_of(Task, knowrob:'Event'),
  interval_end(Task, End).


%% task_duration(?Task, ?Duration) is nondet.
%
%  Compute duration of given task
%
%  @param Task Identifier of given Task
%  @param Duration Duration value
% 
task_duration(Task, Duration) :-
  interval(Task, [ST,ET]),
  Duration is (ET-ST).

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
    rdf_has(Task, knowrob:'taskContext', literal(type(_, Goal))).

task_goal(Task, Goal) :-
    task(Task),
    rdf_has(Task, knowrob:'goalContext', literal(type(_, Goal))).

%% task_goal_inherited(?Action, ?Goal) is nondet.
%
%  Find goal and task of given action.
%
%  @param Action Task individual.
%  @param Goal Identifier of given Goal
% 
task_goal_inherited(Action, Goal) :-
    (  rdf_has(Action, knowrob:'goalContext', Goal)
    -> true
    ;  rdf_has(Parent, knowrob:'subAction', Action),
       task_goal_inherited(Parent, Goal)
    ).

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

%% task_status(+Task, -Status)) is nondet.
%
% Check whether the given task was being continued, done, failed or not
% yet started at the current time point.
%
% @param Task Identifier of given Task
% @param Status Returned status
% 
task_status(Task, Status) :-
    get_timepoint(T),
    knowrob_temporal:holds(task_status(Task, Status), T).

%% holds(task_status(+Task, -Status), +T) is nondet.
%
% Check whether the given task was being continued, done, failed or not
% yet started at the given time point.
%
% @param Task Identifier of given Task
% @param Status Returned status
% @param T   TimePoint
% 
% TODO: re-enable
%knowrob_temporal:holds(task_status(Task, Status), T):-
%    nonvar(Task),
%    task(Task),
%    task_start(Task, Start),
%    task_end(Task, End),
%    
%    ((rdf_triple(knowrob:after, Start, T)) ->   % Start < T
%      (
%        ((rdf_triple(knowrob:after, T, End)) ->
%        (Status = ['Continue']);                % Start < T < End
%        (Status = ['Done']))                    % Start < End < T
%      )
%      ; (                                       % T < Start
%        ((rdf_triple(knowrob:after, T, End)) -> % T < End
%          (Status = ['NotStarted']);            % T < Start, T < End
%          (Status = ['Error']))                 % T < Start, T > End
%        )
%    ).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Objects (and robot parts) and their locations
%


%% remember_at(loc(+Desig,-Loc), ?Time) is nondet.
%
% Check what the belief of the robot for location of Object at given Time .
%
% @param Desig    Identifier of an object designator
% @param Location Pose matrix identifier
% @param Time     TimePoint
% 
remember_at(loc(Desig,Loc), _Time) :-
  mng_designator_location(Desig, Loc).

%% remember_at(robot(+Part,-Loc), +Time) is nondet.
%
% Check what the belief of the robot for location of Robot part at given Time .
%
% @param Part  Identifier of the Part
% @param Loc   Pose matrix identifier
% @param Time  TimePoint
% 
remember_at(robot(Part,Loc), Time) :-
  mng_lookup_transform('/map', Part, Time, Loc).

add_object_as_semantic_instance(Designator, Matrix, Time, ObjInstance) :-
  experiment_map(_Experiment, Map, Time), !,
  add_object_as_semantic_instance(Designator, Matrix, Time, Map, ObjInstance).

add_object_as_semantic_instance(Designator, Matrix, Time, Map, ObjInstance) :-
  ( number(Time)
  -> create_timepoint(Time, TimePoint)
  ;  TimePoint = Time ),
  % TODO: use fluents instead!
  designator_assert(ObjInstance, Designator, Map), !,
  designator_add_perception(ObjInstance, Designator, Matrix, TimePoint).

add_object_as_semantic_instance(Designator, Matrix, Time, Map, ObjInstance) :-
  add_object_to_semantic_map(Designator, Matrix, Time, Map, ObjInstance, 0.2, 0.2, 0.2).


add_robot_as_basic_semantic_instance(Matrix, Time, ObjInstance) :-
  % FIXME(daniel): Seems not a good idea to use time as identifier for robot here!
  ( number(Time)
  -> create_timepoint(Time, TimePoint)
  ;  TimePoint = Time ),
  add_object_to_semantic_map(TimePoint, Matrix, TimePoint, ObjInstance, 0.5, 0.5, 0.5).


add_object_to_semantic_map(Designator, Matrix, Time, ObjInstance, H, W, D) :-
  experiment_map(_Experiment, Map, Time), !,
  add_object_to_semantic_map(Designator, Matrix, Time, Map, ObjInstance, H, W, D).

add_object_to_semantic_map(Designator, Matrix, Time, Map, ObjInstance, H, W, D) :-
  once((
    designator_object(Designator, ObjInstance),
    rdf_assert(ObjInstance, rdf:type, knowrob:'SpatialThing-Localized'),
    rdf_assert(ObjInstance, knowrob:'describedInMap', Map),
    % TODO: use fluents instead!
    object_assert_dimensions(ObjInstance, H, W, D),
    designator_add_perception(ObjInstance, Designator, Matrix, Time)
  )).
