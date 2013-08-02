:- module(comp_self_info,
    [
     	task/1,
	subtask/2,
	subtask_all/2,
      	task_goal/2,
	task_start_time/2,
	task_end_time/2,
	perceived_object/2
	
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

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdf:subClassOf, modexecutiontrace:'CRAMAction');

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdf:subClassOf, B),
	rdf_has(B, rdf:subClassOf, modexecutiontrace:'CRAMAction');
	
	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdf:subClassOf, B),
	rdf_has(B, rdf:subClassOf, C),
	rdf_has(C, rdf:subClassOf, modexecutiontrace:'CRAMAction').

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
	rdf_has(Task, rdf:type, Goal),
	Goal = modexecutiontrace:'AchieveGoalAction';

	task(Task),
	rdf_has(Task, rdf:type, Goal),
	rdf_has(Goal, rdf:subClassOf, modexecutiontrace:'AchieveGoalAction');

	task(Task),
	rdf_has(Task, rdf:type, Goal),
	rdf_has(Goal, rdf:subClassOf, B),
	rdf_has(B, rdf:subClassOf, modexecutiontrace:'AchieveGoalAction').
	
task_start(Task, Start) :-
	task(Task),
	rdf_has(Task, knowrob:'startTime', End).

task_end(Task, End) :-
	task(Task),
	rdf_has(Task, knowrob:'endTime', End).

perceived_object(Tsk, Obj) :-
	rdf_has(Task, rdf:type, knowrob:'Perceiving'),
	rdf_has(Task, modexecutiontrace:'ObjectOfInterest', Obj).
