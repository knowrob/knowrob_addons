/*
  Copyright (C) 2019 Daniel Beßler
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
*/

:- module(ogp_agenda,
    [
      ogp_agenda_create/3,
      ogp_agenda_pop/3,
      ogp_agenda_push/2,
      ogp_agenda_update/3,
      ogp_agenda_specialize_task/2,
      ogp_agenda_isEmpty/1
    ]).
/** <module> Agenda of characterization steps in the OGP method.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('ogp_task')).

:- rdf_meta ogp_agenda_create(r,r,-).

ogp_agenda_create(OGP,Entity,(OGP,_,TaskDicts)) :-
  ogp_steps_for_entity(OGP,Entity,0,TaskDicts).

ogp_steps_for_entity(OGP,Entity,Depth,TaskDicts) :-
  findall(TaskDict,(
    ogp_traverse(OGP,Entity,Depth,[O,O_depth],[]),
    print_message(debug(ogp), format('Add entity: ~w.', [O])),
    ogp_entity_step(OGP,O,O_depth,TaskDict)
  ), TaskDicts).

%%
% 
ogp_traverse(_,O0,_,_,Visited) :-
  member(O0,Visited), !,
  fail.
ogp_traverse(_,O0,D0,[O0,D0],_).
ogp_traverse(OGP,O0,D0,[OX,DX],Visited) :-
  %%
  rdf(O0,P,O1), atom(P),
  rdfs_individual_of(P,owl:'ObjectProperty'),
  % ignore "unfocussed" triples
  ogp_task_hasFocus(_{ogp: OGP, subject: O1, predicate: P}),
  %%
  D1 is D0 + 1,
  ogp_traverse(OGP,O1,D1,[OX,DX],[O0|Visited]).

%%
% 
ogp_entity_step(OGP,Entity,Depth,TaskDict) :-
  owl_unsatisfied_restriction(Entity,Restr),
  ogp_task_create(OGP,Entity,Restr,Depth,TaskDict),
  ogp_task_triple(TaskDict,S,P,_),
  ( ogp_task_hasFocus(TaskDict) -> 
    print_message(debug(ogp), format('Task added: (~w,~w).', [S,P]));(
    print_message(debug(ogp), format('Task ignored: (~w,~w).', [S,P])),
    fail
  )).

%%
% 
ogp_agenda_isEmpty((_,_,[])).

%%
% 
ogp_agenda_pop((OGP,S0,T0)->(OGP,SX,TX),Decisions,Popped) :-
  predsort(compare_task_dicts((OGP,S0,T0)), T0, [Top|T1]),
  ogp_task_subject(Top,S1),
  %%
  ogp_task_triple(Top,S,P,D),
  %%
  ( ogp_task_isObsolete(Top,Decisions) -> (
    %% debugging
    print_message(debug(ogp), format('Task obsolete: (~w,~w,~w).', [S,P,D])),
    %%
    ogp_agenda_pop((OGP,S1,T1)->(OGP,SX,TX),Decisions,Popped) );
  ( TX=T1, SX=S1, Popped=Top )).

%%
% 
ogp_agenda_push(A0->A0,[]) :- !.

ogp_agenda_push(A0->AX,[X|Xs]) :-
  ogp_agenda_push(A0->A1, X),
  ogp_agenda_push(A1->AX, Xs).

ogp_agenda_push((OGP,S,Xs)->(OGP,S,[X|Xs]),X) :-
  ogp_task_triple(X,_,_,_),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % Task ordering
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

compare_task_dicts(Agenda,Delta, Item1, Item2) :-
  get_dict(ogp,Item1,OGP), get_dict(ogp,Item2,OGP),
  ogp:ogp_quantification_criteria(OGP, Criteria),
  compare_task_dicts(Agenda, Delta, Criteria, Item1, Item2).

compare_task_dicts(_, '>', [], _, _) :- !. % no selection criterium left -> random selection
compare_task_dicts(Agenda, Delta, [Criterium|Rest], Item1, Item2) :-
  rdf_has(Criterium, dul:describes, C),
  ogp_task_quantify(Agenda, Item1, C, Val1),
  ogp_task_quantify(Agenda, Item2, C, Val2),
  (  Val1 is Val2
  -> compare_task_dicts(Agenda, Delta, Rest, Item1, Item2)
  ; ( % "smaller" elements (higher priority) first
     Val1 > Val2 -> Delta='<' ;  Delta='>'
  )).

%%
%
ogp_task_quantify(_,TaskDict,Quantifier,V) :-
  rdfs_individual_of(Quantifier, knowrob_planning:'DepthFirst'),!,
  get_dict(depth,TaskDict,V).

ogp_task_quantify(_,TaskDict,Quantifier,V) :-
  rdfs_individual_of(Quantifier, knowrob_planning:'BreadthFirst'),!,
  get_dict(depth,TaskDict,D),
  V is -D.

ogp_task_quantify(_,TaskDict,Quantifier,V) :-
  rdfs_individual_of(Quantifier, knowrob_planning:'InhibitionSelection'),!,
  get_dict(inhibition,TaskDict,InhibitionValue),
  V is -InhibitionValue.

ogp_task_quantify((_,LastS,_),TaskDict,Quantifier,V) :-
  rdfs_individual_of(Quantifier, knowrob_planning:'ContinuitySelection'),!,
  ( get_dict(subject,TaskDict,LastS) -> V is 1 ; V is 0 ).

ogp_task_quantify(_,TaskDict,Pattern,V) :-
  rdfs_individual_of(Pattern, knowrob_planning:'PatternSelection'),!,
  ( ogp_task_matches_pattern(TaskDict,Pattern) -> V is 1 ; V is 0 ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%
% 
ogp_agenda_specialize_task(Agenda,TaskDict0->TaskDict1) :-
  dict_pairs(TaskDict0,_,TaskPairs0),
  dict_pairs(TaskDict1,_,TaskPairs0),
  ogp_task_triple(TaskDict0,_,P0,Domain0),
  ogp_agenda_siblings(Agenda,TaskDict0,Siblings),
  %%
  ogp_agenda_specialize_property(Siblings,P0->P1),
  ogp_agenda_specialize_domain(Siblings,Domain0->Domain1),
  %%
  b_set_dict(predicate,TaskDict1,P1),
  b_set_dict(domain,TaskDict1,Domain1).

ogp_agenda_specialize_property(Siblings,P0->P1) :-
  findall(P, (P=P0 ; (
    member(X,Siblings),
    ogp_task_property(X,P)
  )), Ps),
  list_to_set(Ps,Ps_set),
  owl_most_specific_predicate(Ps_set,P1).

ogp_agenda_specialize_domain(_,Domain0->Domain0) :-
  rdfs_individual_of(Domain0,owl:'NamedIndividual'),!.

ogp_agenda_specialize_domain(Siblings,Domain0->Domain1) :-
  rdfs_individual_of(Domain0,owl:'Class'),!,
  findall(D, (D=Domain0 ; (
    member(X,Siblings),
    ogp_task_domain(X,D)
  )), Domains),
  list_to_set(Domains,Domains_set),
  owl_most_specific(Domains_set,Domain1),
  owl_subclass_of(Domain1,Domain0),!.

ogp_agenda_siblings((_,_,Tasks),TaskDict,Siblings) :-
  ogp_task_type(TaskDict,Type),
  ogp_task_triple(TaskDict,S,P0,Domain0),
  findall(Sibling, (
    member(Sibling,Tasks),
    ogp_task_type(Sibling,Type),
    ogp_task_triple(Sibling,S,P1,Domain1),
    once((
      rdfs_individual_of(Domain1,owl:'Class'),
      owl_subproperty_of(P1,P0),
      owl_subclass_of(Domain1,Domain0)
    ))),
    Siblings).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % Expand the agenda after a decision has been taken
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_agenda_update(t,+,r).

%%
% 
ogp_agenda_update(A0->A1, TaskDict, Entity) :-
  ogp_task_integration(TaskDict),!,
  ogp_task_domain(TaskDict,Domain),
  get_dict(ogp,TaskDict,OGP),
  get_dict(depth,TaskDict,Depth0),
  Depth1 is Depth0 + 1,
  %%
  findall(SubTaskDict,(
    ogp_task_create(OGP,Entity,Domain,Depth1,SubTaskDict);
    ogp_entity_step(OGP,Entity,Depth1,SubTaskDict)
  ), SubTaskDicts),
  %%
  ogp_agenda_push(A0->A1,SubTaskDicts).

ogp_agenda_update(A0->A1, TaskDict, Class) :-
  ogp_task_classification(TaskDict),!,
  ogp_task_subject(TaskDict,Subject),
  get_dict(ogp,TaskDict,OGP),
  get_dict(depth,TaskDict,Depth),
  %%
  findall(SubTaskDict,
    ogp_task_create(OGP,Subject,Class,Depth,SubTaskDict),
    SubTaskDicts),
  %%
  ogp_agenda_push(A0->A1,SubTaskDicts).

ogp_agenda_update(A0->A0, TaskDict, _) :-
  ogp_task_nullification(TaskDict),!.
ogp_agenda_update(A0->A0, TaskDict, _) :-
  ogp_task_quantification(TaskDict),!.
