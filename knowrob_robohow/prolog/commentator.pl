
:- module(commentator,
    [
        comment/2,
        comment/3,
        comment_action/2,
        action_position/2
    ]).

:-  rdf_meta
    comment_action(r,-).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('srdl2')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(forth_human, 'http://knowrob.org/kb/forth_human.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(boxy2, 'http://knowrob.org/kb/BoxyWithRoller.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pr2, 'http://knowrob.org/kb/PR2.owl#', [keep(true)]).

% TODO: should be in another module
action_during(Action, Now) :-
  rdfs_individual_of(Action, knowrob:'Action'),
  interval(Action, ActionInterval),
  interval_during(Now, ActionInterval).

comment(What, Where) :-
  current_time(Now),
  comment(What, Where, Now).

comment(What, Where, When) :-
  action_during(Action, When),
  comment_action(Action, What),
  action_position(Action, Where).

comment_action(Action, Comment) :-
  rdfs_individual_of(Action, TypeIri),
  owl_subclass_of(TypeIri, knowrob:'Action'),
  rdf_split_url(_, Type, TypeIri),
  atomic_list_concat([
    'The robot performs a',
    Type,
    'action'], ' ', Comment).
  
% TODO: implement
action_position(Action, [0.1,-0.2,1.05]).

