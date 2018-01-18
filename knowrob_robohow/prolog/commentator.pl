
:- module(commentator,
    [
        comment/2,
        comment/3,
        comment_interval/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/comp_temporal')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/srdl2')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(forth_human, 'http://knowrob.org/kb/forth_human.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(boxy2, 'http://knowrob.org/kb/BoxyWithRoller.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pr2, 'http://knowrob.org/kb/PR2.owl#', [keep(true)]).


task_with_context_ends_before(Context, EventInstance, Timepoint) :-
  rdf_has(EventInstance, knowrob:'taskContext', literal(type(_,Context))),
  rdf_has(EventInstance, knowrob:'endTime', Instant),
  time_earlier_then(Instant, Timepoint).

task_with_context_begins_before(Context, EventInstance, Timepoint) :-
  rdf_has(EventInstance, knowrob:'taskContext', literal(type(_,Context))),
  rdf_has(EventInstance, knowrob:'startTime', Instant),
  time_earlier_then(Instant, Timepoint).


comment_history(Id) :-
  current_predicate(v_comment,_),
  v_commment(Id).


comment(What, Where) :-
  current_time(Now),
  comment(What, Where, Now).

% if perception failed...
comment(What, none, When) :-
  not(rdf_has(perception, _, _)),
  not(rdf_has(perception_failed, _, _)),
  task_with_context_ends_before('UIMA-PERCEIVE', Perc, When),
  % TODO: ask jan, howto distinguish between perception fail and success
  rdf_has(Perc, knowrob:'taskSuccess', literal(type(_,false))),
  %task_with_context_ends_before('REPLACEABLE-FUNCTION-NAVIGATE', EventInstance, When),
  What='If no objects were found Raphael changes the perspective by moving to another location.',
  atom_number(When_, When), rdf_assert(perception_failed, What, When_),
  writeln('v_commment(perception_failed)'), !.

% if perception succeeded...
comment(What, _, When) :-
  not(rdf_has(perception, _, _)),
  task_with_context_ends_before('UIMA-PERCEIVE', Perc, When),
  rdf_has(Perc, knowrob:'taskSuccess', literal(type(_,true))),
  
  rdf_has(Perc, knowrob:'objectActedOn', PercObj),
  rdf_has(PercObj, knowrob:'designator', Desig),
  mng_designator(Desig, DesignatorJ),
  jpl_call(DesignatorJ, 'get', ['ROBOSHERLOCK-CLASS'], Type),
  camelcase(TypeUnderscore, Type),
  atomic_list_concat(L, '_', TypeUnderscore),
  atomic_list_concat(L, ' ', Name),
  
  atomic_list_concat([
    'Now Raphael has found a ', Name, '. ',
    'The next step is to grasp it.'], '', What),
  
  atom_number(When_, When), rdf_assert(perception, What, When_), !.

% search for grasping position
comment(What, none, When) :-
  not(rdf_has(carry, _, _)),
  not(rdf_has(grasp_position, _, _)),
  rdf_has(perception, _, _),
  
  not(task_with_context_begins_before('REPLACEABLE-FUNCTION-PLACE-OBJECT', _, When)),
  task_with_context_ends_before('REPLACEABLE-FUNCTION-NAVIGATE', Nav, When),
  task_with_context_ends_before('UIMA-PERCEIVE', Perc, When),
  rdf_has(Perc, knowrob:'taskSuccess', literal(type(_,true))),
  interval_before(Perc, Nav),
  rdf_has(Perc, knowrob:'endTime', Instant),
  
  time_term(Instant, Instant_v),
  Threshold is Instant_v + 8.0,
  When > Threshold,
  
  atomic_list_concat([
    'A suitable position is required in order to perform the grasp action.'], '', What),
  
  atom_number(When_, When), rdf_assert(grasp_position, What, When_), !.

% if object grasped
comment(What, none, When) :-
  not(rdf_has(carry, _, _)),
  rdf_has(perception, _, _),
  task_with_context_begins_before('REPLACEABLE-FUNCTION-PLACE-OBJECT', _, When),
  What='The object is now carried to the dinner table where it is placed as part of a dinner setting.',
  atom_number(When_, When), rdf_assert(carry, What, When_), !.

% TODO: comment on putdown action?

% goodbye
comment(What, none, When) :-
  not(rdf_has(goodbye, _, _)),
  rdf_has(carry, _, _),
  task_with_context_ends_before('REPLACEABLE-FUNCTION-PLACE-OBJECT', Place, When),
  task_with_context_ends_before('REPLACEABLE-FUNCTION-PICK-OBJECT', Pick, When),
  interval(Pick, [Begin,_]),
  interval(Place, [_,End]),
  Duration is End-Begin,
  round(Duration, DurationRound),
  
  findall( Action, (
    rdfs_individual_of(Action, knowrob:'Action'),
    interval_during(Action, [Begin,End])
  ), Actions),
  length(Actions, NumActions),
  
  findall( Action, (
    rdfs_individual_of(Action, knowrob:'Action'),
    rdf_has(Action, knowrob:'taskSuccess', literal(type(_,false))),
    interval_during(Action, [Begin,End])
  ), FailedActions),
  length(FailedActions, NumFailedActions),
  
  atomic_list_concat([
    'The first pick and place task was performed within ', DurationRound, ' seconds. ',
    'The plan contained ', NumActions, ' actions from which ', NumFailedActions, ' failed.'], '', What),
  
  atom_number(When_, When), rdf_assert(goodbye, What, When_), !.

comment_interval([Begin, End], DT, Comments) :-
  ( Begin < End -> (
  NextBegin is Begin + DT,
  (  comment(What, _, Begin)
  -> (comment_interval([NextBegin, End], DT, Tail), Comments = [[What,Begin]|Tail])
  ;   comment_interval([NextBegin, End], DT, Comments)
  ))
  ; Comments = [] ).

