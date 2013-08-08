:- module(comp_self_info,
    [
     	task/1,
	subtask/2,
	subtask_all/2,
      	task_goal/2,
	task_start/2,
	task_end/2,
	perceived_object/2,
	computable_belief/3,
      	computable_truth/3,
	computable_observed/3
	
    ]).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://ias.cs.tum.edu/kb/knowrob_cram.owl#', [keep(true)]).

task(Task) :-
	rdf_has(Task, rdf:type, modexecutiontrace:'CRAMAction');

	rdf_has(Task, rdf:type, knowrob:'VisualPerception');

	rdf_has(Task, rdf:type, knowrob:'PerformOnProcessModule');

	rdf_has(Task, rdf:type, knowrob:'Perform');

	rdf_has(Task, rdf:type, knowrob:'Monitor');

	rdf_has(Task, rdf:type, knowrob:'Perceive');

	rdf_has(Task, rdf:type, knowrob:'ResolveActionDesignator');

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdfs:subClassOf, modexecutiontrace:'CRAMAction');

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdfs:subClassOf, B),
	rdf_has(B, rdfs:subClassOf, modexecutiontrace:'CRAMAction');
	
	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdfs:subClassOf, B),
	rdf_has(B, rdfs:subClassOf, C),
	rdf_has(C, rdfs:subClassOf, modexecutiontrace:'CRAMAction').

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

perceived_object(Task, Obj) :-
	rdf_has(Task, rdf:type, knowrob:'VisualPerception'),
	rdf_has(Task, knowrob:'detectedObject', Obj).

belief_at(Loc, Time) :-
	computable_belief(first(Loc), Time, second(Loc)).

truth_at(Loc, Time) :-
	computable_belief(first(Loc), Time, second(Loc)).

perceive_at(Loc, Time) :-
	computable_belief(first(Loc), Time, second(Loc)).

computable_belief(Object, Time, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

  

    jpl_call(Client, 'getBelief', [], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),


    [M00, M01, M02] = LocList,

    atomic_list_concat(['rotMat2D_',M00,'_',M01,'_',M02], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix2D').

computable_truth(Object, Time, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    term_to_atom(Time, t),

    term_to_atom(Object, o),

    jpl_call(Client, 'getReal', [o, t], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),


    [M00, M01, M02] = LocList,

    atomic_list_concat(['rotMat2D_',M00,'_',M01,'_',M02], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix2D').

computable_observed(Object, Time, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_execution_trace.ROSClient_low_level', ['my_low_level'], Client),

    term_to_atom(Time, t),

    term_to_atom(Object, o),

    jpl_call(Client, 'getBelief', [o, t], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),


    [M00, M01, M02] = LocList,

    atomic_list_concat(['rotMat2D_',M00,'_',M01,'_',M02], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix2D').
