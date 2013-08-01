:- module(comp_self_info,
    [
     	task/1,
      	task_goal/2,
	perceived_object/2
    ]).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(modexecutiontrace, 'http://ias.cs.tum.edu/kb/modexecutiontrace.owl#', [keep(true)]).

task(Task) :-
	rdf_has(Task, rdf:type, knowrob:'Event');

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdf:subClassOf, knowrob:'Event');

	rdf_has(Task, rdf:type, A),
	rdf_has(A, rdf:subClassOf, B),
	rdf_has(B, rdf:subClassOf, knowrob:'Event').

task_goal(Task, Goal) :-
	rdf_has(Task, modexecutiontrace:'TaskGoal', Goal).

perceived_object(Tsk, Obj) :-
	rdf_has(Task, modexecutiontrace:'TaskGoal', Goal),
	rdf_has(Goal, rdf:type, knowrob:'Perceiving'),
	rdf_has(Task, modexecutiontrace:'ObjectOfInterest', Obj);

	rdf_has(Task, rdf:type, knowrob:'Perceiving'),
	rdf_has(Task, modexecutiontrace:'ObjectOfInterest', Obj).
