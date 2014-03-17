:- module(comp_execution_trace,
    [
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
	javarun_designator/2,
	javarun_time_check/3,
	javarun_time_check2/3,
	javarun_location_check/3,
	javarun_perception_time_instances/2,
	javarun_perception_object_instances/2,
	javarun_loc_change/3,
	failure_class/2,
	failure_task/2,
	failure_attribute/3
    ]).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://ias.cs.tum.edu/kb/knowrob_cram.owl#', [keep(true)]).

% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate cram_holds(0, ?, ?).
:- discontiguous cram_holds/2.

:- meta_predicate occurs(0, ?, ?).
:- discontiguous occurs/2.

:- meta_predicate belief_at(0, ?, ?).
:- discontiguous belief_at/2.


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    task(r),
    task_class(r,r),
    subtask(r,r),
    subtask_all(r,r),
    task_goal(r,r),
    task_start(r,r),
    task_end(r,r),
    belief_at(r,r),
    belief_at(r,r),
    occurs(r,r),
    cram_holds(r,r),
    returned_value(r,r),
    javarun_designator(r,r),
    javarun_time_check(r,r,r),
    javarun_time_check2(r,r,r),
    javarun_location_check(r,r,r),
    javarun_perception_time_instance(r,r),
    javarun_perception_object_instance(r,r),
    javarun_loc_change(r,r,r),
    failure_class(r,r),
    failure_task(r,r),
    failure_attribute(r,r,r).



task(Task) :-
	rdf_has(Task, rdf:type, A),
	rdf_reachable(A, rdfs:subClassOf, knowrob:'CRAMEvent').

task_class(Task, Class) :-
	rdf_has(Task, rdf:type, Class),
	rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMEvent').

subtask(Task, Subtask) :-
	task(Task),
	task(Subtask),
	rdf_has(Task, knowrob:'subAction', Subtask).

subtask_all(Task, Subtask) :-
	subtask(Task, Subtask);

	nonvar(Task),
	task(Task),
	task(Subtask),
	subtask(Task, A),
	subtask_all(A, Subtask);


	nonvar(Subtask),
	task(Task),
	task(Subtask),
	subtask(A, Subtask),
	subtask_all(Task, A);


	var(Task),
	var(Subtask),
	task(Task),
	task(Subtask),
	subtask(Task, A),
	subtask_all(A, Subtask).

task_goal(Task, Goal) :-
	task(Task),
	rdf_has(Task, knowrob:'taskContext', literal(type(_, Goal))).

task_start(Task, Start) :-
	task(Task),
	rdf_has(Task, knowrob:'startTime', Start).

task_end(Task, End) :-
	task(Task),
	rdf_has(Task, knowrob:'endTime', End).

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
		javarun_time_check2(Bts, TConverted, LastPerception),

		task_class(T, knowrob:'UIMAPerception'), 
		task_end(T, LastPerception), 
		returned_value(T, Obj),
 
		rdf_has(Obj, knowrob:'designator',Designator), 
		javarun_designator(Designator, Loc).

belief_at(robot(Part,Loc), Time) :-
		findall(
        		EndTime,
        		(   
			   task_class(Tsk, Part), 
			   task_end(Tsk, Et),
			   term_to_atom(Et, EndTime)     
        		),
        		EndTimes
    		),

		jpl_new( '[Ljava.lang.String;', EndTimes, Ets),
		term_to_atom(Time, TConverted),
		javarun_time_check2(Ets, TConverted, LastLocation),

		task_class(T, Part), 
		task_end(T, LastLocation), 
		subtask(ParentTask, T),
		rdf_has(ParentTask, knowrob:'goalPose',Designator), 
		javarun_designator(Designator, Loc).


%it is not possible to extract that kind of information from current logs
occurs(loc_change(Obj),T) :-
	nonvar(Obj),
	nonvar(T),
	task_class(Task, knowrob:'UIMAPerception'),
	returned_value(Task, Obj),
        task_start(Task, T),
	rdf_has(Task, knowrob:'objectActedOn', Obj), 
	rdf_has(Obj, knowrob:'designator',Designator),
	javarun_loc_change(Obj, Designator, T).

occurs(object_perceived(Obj),T) :-
	nonvar(Obj),
	nonvar(T),
	task_class(Task, knowrob:'UIMAPerception'),
	returned_value(Task, Obj),
	task_start(Task, T).

cram_holds(task_status(Task, Status), T):-
	nonvar(Task),
	task(Task),
	task_start(Task, Start),
	task_end(Task, End),
	javarun_time_check(Start, T, Compare_Result1),
	javarun_time_check(T, End, Compare_Result2),
	term_to_atom(Compare_Result1, c1),
	term_to_atom(Compare_Result2, c2),
	((c1 is 1) -> (((c2 is 1) -> (Status = ['Continue']);(Status = ['Done'])));(((c2 is 1) -> (Status = ['Error']); (Status = ['NotStarted'])))).

cram_holds(object_visible(Object, Status), T):-
	nonvar(Object),
	nonvar(T),
	javarun_belief(Object, T, Loc),
	rdf_triple(comp_spatial:'m01', Loc, Result),
	term_to_atom(Result, r),
	((r is -1) -> (Status = [true]);(Status = [false])).

	%nonvar(Object),
	%var(T),
	%javarun_perception_time_instances(Object, T),
	%Status = [true];

	%var(Object),
	%nonvar(T),
	%javarun_perception_object_instances(T, Object),
	%Status = [true].

cram_holds(object_placed_at(Object, Loc), T):-
	javarun_belief(Object, T, Actual_Loc),
	javarun_time_check(Loc, Actual_Loc, Compare_Result),
	term_to_atom(Compare_Result, r),
	((r is 0) -> (true);(false)).

returned_value(Task, Obj) :-
	rdf_has(Task, rdf:type, knowrob:'UIMAPerception'),
	rdf_has(Task, knowrob:'objectActedOn', Obj);

	task(Task),
	failure_task(Obj, Task).

javarun_designator(Designator, Loc) :-
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    % Designator = literal(type(A,D)),

    jpl_call(Client, 'getBeliefByDesignator', [Designator], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),

    [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33] = LocList,

    atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D').

javarun_loc_change(Obj, Designator, Time) :-
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'checkLocationChange', [Obj, Designator, Time], Result),

    jpl_array_to_list(Result, ResultList),

    [Compare_Result] = ResultList,
    ((Compare_Result is -1) -> (false);((rdf_has(Compare_Result, rdf:type, knowrob:'HumanScaleObject')) -> (true);(false))).


javarun_time_check(Time1, Time2, Compare_Result) :-

    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'timeComparison', [Time1, Time2], Compare_Result).

javarun_time_check2(TimeList, Time2, Compare_Result) :-

    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'timeComparison2', [TimeList, Time2], Compare_Result).


javarun_location_check(L1, L2, Compare_Result) :-

    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'locationComparison', [L1, L2], Compare_Result).

javarun_perception_time_instances(Object, TimeList) :-

    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'getPerceptionTimeStamps', [Object], Times),

    jpl_array_to_list(Times, TimeList).

javarun_perception_object_instances(Time, ObjectList) :-
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    jpl_call(Client, 'getPerceptionObjects', [Time], Objects),

    jpl_array_to_list(Objects, ObjectList).

failure_class(Error, Class) :-
	rdf_has(Error, rdf:type, Class),
	rdf_reachable(Class, rdfs:subClassOf, knowrob:'CRAMFailure').

failure_task(Error, Task) :-
	task(Task),
	%failure_class(Error, Class),
	rdf_has(Task, knowrob:'eventFailure', Error).

failure_attribute(Error,AttributeName,Value) :-
	%failure_class(Error, Class),
	rdf_has(Error, AttributeName, Value).
