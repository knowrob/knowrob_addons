:- module(comp_self_info,
    [
     	task/1,
	subtask/2,
	subtask_all/2,
      	task_goal/2,
	task_start/2,
	task_end/2,
	holds/2,
	returned_value/2,
	computable_belief/3,
      	computable_truth/3,
	computable_time_check/3,
	computable_location_check/3,
	belief_at/2,
	truth_at/2
	%task_status done
	%belief_at done
	%occur
	%holds_at _ during _tt(throughout) different predicates or single
	
    ]).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://ias.cs.tum.edu/kb/knowrob_cram.owl#', [keep(true)]).

task(Task) :-	
	rdf_has(Task, rdf:type, A),
	rdf_individual_of(A, modexecutiontrace:'CRAMAction').

subtask(Task, Subtask) :-
	task(Task),
	task(Subtask),
	rdf_has(Task, knowrob:'subAction', Subtask).

subtask_all(Task, Subtask) :-
	subtask(Task, Subtask);	

	task(Task),
	task(Subtask),
	subtask(Task, A),
	subtask_all(A, Subtask).

task_goal(Task, Goal) :-
	task(Task),
	rdf_has(Task, rdf:type, Goal);

	% task(Task),
	% rdf_has(Task, rdf:type, Goal),
	% Goal = modexecutiontrace:'AchieveGoalAction';

	task(Task),
	rdf_has(Task, rdf:type, Goal),
	rdf_has(Goal, rdfs:subClassOf, modexecutiontrace:'AchieveGoalAction');

	task(Task),
	rdf_has(Task, rdf:type, Goal),
	rdf_has(Goal, rdfs:subClassOf, B),
	rdf_has(B, rdfs:subClassOf, modexecutiontrace:'AchieveGoalAction').
	
task_start(Task, Start) :-
	task(Task),
	rdf_has(Task, knowrob:'startTime', Start).

task_end(Task, End) :-
	task(Task),
	rdf_has(Task, knowrob:'endTime', End).

holds(task_status(Task, Status), T):-
	task(Task),
	task_start(Task, Start),
	task_end(Task, End),
	computable_time_check(Start, T, Compare_Result1),
	computable_time_check(T, End, Compare_Result2)
	term_to_atom(Compare_Result1, c1),	
	term_to_atom(Compare_Result2, c2),
	((c1 is 1) -> (((c2 is 1) -> (Status = [Continue]);(Status = [Done])));(((c2 is 1) -> (Status = [Error]); (Status = [NotStarted])))).

holds(object_visible(Object, Status), T):-
	computable_belief(Object, T, Loc),
	rdf_triple(comp_spatial:'m01', Loc, Result),
	term_to_atom(Result, r),
	((r is -1) -> (Status = [true]);(Status = [false])).	

holds(object_placed_at(Object, Loc), T):-
	computable_belief(Object, T, Actual_Loc),
	computable_time_check(Loc, Actual_Loc, Compare_Result),
	term_to_atom(Compare_Result, r),
	((r is 0) -> (true);(false)).

returned_value(Task, Obj) :-
	rdf_has(Task, rdf:type, knowrob:'VisualPerception'),
	rdf_has(Task, knowrob:'detectedObject', Obj).

belief_at(Loc, Time) :-
	computable_belief(first(Loc), Time, second(Loc)).

truth_at(Loc, Time) :-
	computable_belief(first(Loc), Time, second(Loc)).

computable_belief(Object, Time, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

  

    jpl_call(Client, 'getBelief', [], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),


    [M00, M01, M02, M10, M11, M12, M20, M21, M22] = LocList,

    atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M10,'_',M11,'_',M12, '_',M20,'_',M21,'_',M22], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix2D').

computable_truth(Object, Time, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    term_to_atom(Time, t),

    term_to_atom(Object, o),

    jpl_call(Client, 'getReal', [o, t], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),


    [M00, M01, M02, M10, M11, M12, M20, M21, M22] = LocList,

    atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M10,'_',M11,'_',M12, '_',M20,'_',M21,'_',M22], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D').

computable_time_check(Time1, Time2, Compare_Result) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    term_to_atom(Time1, t1),

    term_to_atom(Time2, t2),

    jpl_call(Client, 'timeComparison', [t1, t2], Result),

    jpl_array_to_list(Result, ResultList),

    [Compare_Result] = ResultList.

computable_location_check(L1, L2, Compare_Result) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    term_to_atom(L1, l1),

    term_to_atom(L2, l2),

    jpl_call(Client, 'timeComparison', [l1, l2], Result),

    jpl_array_to_list(Result, ResultList),

    [Compare_Result] = ResultList.
