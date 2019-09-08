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

:- module(ogp_task,
    [
      ogp_task_create/5,
      %%
      ogp_task_hasFocus/1,
      ogp_task_isObsolete/2,
      ogp_task_matches_pattern/2,
      %%
      ogp_task_type/2,
      ogp_task_integration/1,
      ogp_task_decomposition/1,
      ogp_task_nullification/1,
      ogp_task_classification/1,
      ogp_task_quantification/1,
      %%
      ogp_task_subject/2,
      ogp_task_property/2,
      ogp_task_domain/2,
      ogp_task_reason/3,
      ogp_task_triple/4,
      %%
      ogp_task_inhibit/1,
      ogp_task_decrease_card/1,
      %%
      ogp_task_initialize/3,
      ogp_task_selection/3
    ]).
/** <module> A task item on the agenda of the OGP procedure.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('owl_planning')).

:- rdf_meta ogp_task_type(+,r),
      ogp_task_subject(+,r),
      ogp_task_property(+,r),
      ogp_task_domain(+,r),
      ogp_task_reason(+,r,r),
      ogp_task_triple(+,r,r,r).

%% ogp_task_type(+TaskDict,?Type)
%
% Maps an agenda item to its type (i.e., one of integrate, decompose, classify, ...)
%
ogp_task_type(TaskDict,TaskType) :-
  get_dict(type, TaskDict, integrate),!,
  rdf_equal(TaskType, knowrob_planning:'Integration').

ogp_task_type(TaskDict,TaskType) :-
  get_dict(type, TaskDict, decompose),!,
  rdf_equal(TaskType, knowrob_planning:'Decomposition').

ogp_task_type(TaskDict,TaskType) :-
  get_dict(type, TaskDict, detach),!,
  rdf_equal(TaskType, knowrob_planning:'Nullification').

ogp_task_type(TaskDict,TaskType) :-
  get_dict(type, TaskDict, classify),!,
  rdf_equal(TaskType, knowrob_planning:'Classification').

ogp_task_type(TaskDict,TaskType) :-
  get_dict(type, TaskDict, quantify),!,
  rdf_equal(TaskType, knowrob_planning:'Quantification').

ogp_task_integration(TaskDict)    :- ogp_task_type(TaskDict,knowrob_planning:'Integration').
ogp_task_decomposition(TaskDict)  :- ogp_task_type(TaskDict,knowrob_planning:'Decomposition').
ogp_task_nullification(TaskDict)  :- ogp_task_type(TaskDict,knowrob_planning:'Nullification').
ogp_task_classification(TaskDict) :- ogp_task_type(TaskDict,knowrob_planning:'Classification').
ogp_task_quantification(TaskDict) :- ogp_task_type(TaskDict,knowrob_planning:'Quantification').

%% ogp_task_subject(+TaskDict,?S)
%
% Maps an agenda item to its subject (i.e., the object for which
% new specifications need to be asserted).
%
ogp_task_subject(TaskDict,S) :- get_dict(subject,TaskDict,S),!.

%% ogp_task_property(+TaskDict,?P)
%
% Maps an agenda item to its predicate (i.e., the predicate which
% needs to be specified for the items subject).
%
ogp_task_property(TaskDict,P) :- get_dict(predicate,TaskDict,P),!.

%% ogp_task_domain(+TaskDict,?Domain)
%
% Maps an agenda item to its domain (i.e., the class description
% of values to be specified for the items subject along the items predicate).
%
ogp_task_domain(TaskDict,Domain) :- get_dict(domain,TaskDict,Domain),!.

%% ogp_task_reason(+TaskDict,?Cause,?CauseRestr)
%
% Maps an agenda item to its reason (i.e., the unsattisfied restriction
% that caused this item to be generated).
%
ogp_task_reason(TaskDict,Cause,CauseRestr) :-
  get_dict(cause,TaskDict,[Cause,CauseRestr]),!.

%%
ogp_task_triple(TaskDict,S,P,Domain) :-
  ogp_task_subject(TaskDict,S),
  ( ogp_task_property(TaskDict,P);
    P=_
  ),
  ogp_task_domain(TaskDict,Domain),!.

%%
ogp_task_inhibit(TaskDict) :-
  get_dict(inhibition,TaskDict,X0),
  X1 is X0+1,
  b_set_dict(inhibition,TaskDict,X1).

%%
ogp_task_decrease_card(TaskDict) :-
  get_dict(cardinality,TaskDict,X0),
  X1 is X0-1,
  b_set_dict(cardinality,TaskDict,X1).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % Creating tasks from unsattisfied restrictions
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_task_create(r,r,r,+,-),
            ogp_task_matches_pattern(+,r).

%%
ogp_task_create(OGP,Cause0,Restr0,Depth0,TaskDict) :-
  rdfs_individual_of(Restr0,owl:'Class'),
  owl_satisfies_restriction_up_to(Cause0,Restr0,UpTo),
  %% auto decompose
  ( ogp_decomposed_restriction(OGP,UpTo,Cause1,Restr1)
  *-> (
    Depth1 is Depth0 + 1,
    ogp_task_create(OGP,Cause1,Restr1,Depth1,TaskDict)
  ) ; (
    ogp_task_create(UpTo,TaskDict0),
    TaskDict = _{ogp: OGP,
                 cause: [Cause0,Restr0],
                 depth: Depth0,
                 inhibition: 0}.put(TaskDict0)
  )).

ogp_task_create(specify(S,P,Domain,Card),_{
  type:      integrate,
  subject:   S,
  predicate: P,
  domain:    Domain,
  cardinality: Card}) :-
  rdfs_individual_of(P,owl:'ObjectProperty'),!.

ogp_task_create(specify(S,P,Domain,Card),_{
  type:      quantify,
  subject:   S,
  predicate: P,
  domain:    Domain,
  cardinality: Card}).

ogp_task_create(detach(S,P,Domain,Card),_{
  type:      detach,
  subject:   S,
  predicate: P,
  domain:    Domain,
  cardinality: Card}).

ogp_task_create(classify(S,Cls),_{
  type:      classify,
  subject:   S,
  predicate: 'http://www.w3.org/2000/01/rdf-schema#type',
  domain:    Cls,
  cardinality: 1}).

ogp_decomposed_restriction(OGP,specify(S,P,Domain,Card),O,Restr) :-
  ogp_is_decomposable(OGP,P),
  rdfs_individual_of(Domain,owl:'Class'),!,
  % decompose *Card* times
  between(1,Card,_),
  decompose(S,P,Domain,O),
  ( Restr=Domain;
    owl_unsatisfied_restriction(O,Restr)
  ).

ogp_is_decomposable(OGP,P0) :-
  rdf_has(OGP,knowrob_planning:hasDecomposableProperties,Collection),
  rdf_has(Collection,dul:hasMember,ReifiedRelation),
  owl_reified_relation(P1,ReifiedRelation),
  owl_subproperty_of(P0,P1), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

ogp_task_hasFocus(TaskDict) :-
  get_dict(ogp, TaskDict, OGP),
  rdf_has(OGP,knowrob_planning:hasTaskWhitelist,Collection),
  rdf_has(Collection,dul:hasMember,Pattern),
  ogp_task_matches_pattern(TaskDict,Pattern),!.

ogp_task_matches_pattern(TaskDict,Pattern) :-
  ogp_task_matches_type(TaskDict,Pattern),
  ogp_task_matches_subject(TaskDict,Pattern),
  ogp_task_matches_predicate(TaskDict,Pattern),
  ogp_task_matches_domain(TaskDict,Pattern).

ogp_task_matches_type(TaskDict,Pattern) :-
  ( ogp_pattern_type(Pattern,T0) *-> (
    ogp_task_type(TaskDict,T1),
    owl_subclass_of(T1,T0) )
  ; true ), !.

ogp_task_matches_subject(TaskDict,Pattern) :-
  ( ogp_pattern_subject(Pattern,S_class) *-> (
    ogp_task_subject(TaskDict,S),
    owl_individual_of(S,S_class) )
  ; true ), !.

ogp_task_matches_predicate(TaskDict,Pattern) :-
  ( ogp_pattern_property(Pattern,P0) *-> (
    ogp_task_property(TaskDict,P1),
    owl_subproperty_of(P1,P0) )
  ; true ), !.

ogp_task_matches_domain(TaskDict,Pattern) :-
  ( ogp_pattern_domain(Pattern,C0) *-> (
    ogp_task_property(TaskDict,C1),
    % FIXME: individual domains
    owl_subclass_of(C1,C0) )
  ; true ), !.

ogp_pattern_type(Pattern,C) :-
  rdf_has(Pattern,knowrob_planning:hasTypePattern,ReifiedClass),
  owl_reified_class(C,ReifiedClass). 
ogp_pattern_subject(Pattern,C) :-
  rdf_has(Pattern,knowrob_planning:hasSubjectPattern,ReifiedClass),
  owl_reified_class(C,ReifiedClass). 
ogp_pattern_property(Pattern,P) :-
  rdf_has(Pattern,knowrob_planning:hasPredicatePattern,ReifiedRelation),
  owl_reified_relation(P,ReifiedRelation).
ogp_pattern_domain(Pattern,C) :-
  rdf_has(Pattern,knowrob_planning:hasDomainPattern,ReifiedClass),
  owl_reified_class(C,ReifiedClass).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % agenda task validity
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_task_isObsolete(+,t).

ogp_task_isObsolete(Top,Decisions) :-
  ogp_task_classification(Top),!,
  ogp_task_triple(Top,S,_,Domain),
  member([TaskDict,Selection],Decisions),
  ogp_task_classification(TaskDict),
  ogp_task_subject(TaskDict,S),
  owl_subclass_of(Selection,Domain),!.

ogp_task_isObsolete(Top,Decisions) :-
  ogp_task_nullification(Top),!,
  ogp_task_triple(Top,S,P,Domain),
  get_dict(cardinality,Top,Card0),
  findall(X, (
    member([TaskDict,X],Decisions),
    ogp_task_nullification(TaskDict),
    ogp_task_triple(TaskDict,S,P,_),
    ( rdfs_individual_of(Domain,owl:'Class') ->
      once(owl_individual_of(X,Domain)) ;
      X=Domain )
    ), Xs),
  length(Xs,Card1),
  Card0 =< Card1.

ogp_task_isObsolete(Top,Decisions) :-
  ogp_task_integration(Top),!,
  ogp_task_triple(Top,S,P,Domain),
  get_dict(cardinality,Top,Card0),
  findall(X, (
    member([TaskDict,X],Decisions),
    ogp_task_integration(TaskDict),
    ogp_task_triple(TaskDict,S,P,_),
    once(owl_specializable(X,Domain))),
    Xs),
  length(Xs,Card1),
  Card0 =< Card1.

ogp_task_isObsolete(Top,_Decisions) :-
  ogp_task_quantification(Top),!,
  fail. % TODO support quantification

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % OGP task input/output
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_task_initialize(+,-,?),
            ogp_task_selection(r,+,r),
            ogp_task_initialize_(+,r,r,r,-).

ogp_task_initialize(TaskDict,InputDict,IndividualTask) :-
  ogp_task_classification(TaskDict),!,
  rdf_equal(IndividualTask,knowrob_planning:'Classification_OGP'),
  rdf_equal(knowrob_planning:'Classification_OGP_Subject', S_key),
  rdf_equal(knowrob_planning:'Classification_OGP_Constraint', C_key),
  ogp_task_triple(TaskDict,S_val,_,Domain),
  owl_reified_class(Domain,C_val),
  dict_pairs(InputDict, _, [S_key-S_val,C_key-C_val]).

ogp_task_initialize(TaskDict,InputDict,IndividualTask) :-
  ogp_task_integration(TaskDict),!,
  rdf_equal(IndividualTask,knowrob_planning:'Integration_OGP'),
  ogp_task_initialize_(TaskDict,
    knowrob_planning:'Integration_OGP_Subject',
    knowrob_planning:'Integration_OGP_Predicate',
    knowrob_planning:'Integration_OGP_Constraint', InputDict).

ogp_task_initialize(TaskDict,InputDict,IndividualTask) :-
  ogp_task_nullification(TaskDict),!,
  rdf_equal(IndividualTask,knowrob_planning:'Nullification_OGP'),
  ogp_task_initialize_(TaskDict,
    knowrob_planning:'Nullification_OGP_Subject',
    knowrob_planning:'Nullification_OGP_Predicate',
    knowrob_planning:'Nullification_OGP_Constraint', InputDict).

ogp_task_initialize(TaskDict,InputDict,IndividualTask) :-
  ogp_task_nullification(TaskDict),!,
  rdf_equal(IndividualTask,knowrob_planning:'Quantification_OGP'),
  ogp_task_initialize_(TaskDict,
    knowrob_planning:'Quantification_OGP_Subject',
    knowrob_planning:'Quantification_OGP_Predicate',
    knowrob_planning:'Quantification_OGP_Constraint', InputDict).

ogp_task_initialize_(TaskDict,S_key,P_key,C_key,InputDict) :-
  ogp_task_triple(TaskDict,S_val,P,Domain),
  owl_reified_relation(P,P_val),
  ( rdfs_individual_of(Domain,owl:'Class') ->
    owl_reified_class(Domain,C_val);
    C_val=Domain ),
  dict_pairs(InputDict, _, [
    -(S_key,S_val),
    -(P_key,P_val),
    -(C_key,C_val)
  ]).
  
ogp_task_selection(IndividualTask,OutputDict,Selection) :-
  rdf_equal(IndividualTask,knowrob_planning:'Classification_OGP'),!,
  get_dict(
    'http://knowrob.org/kb/knowrob_planning.owl#Classification_OGP_Outcome',
    OutputDict,ReifiedClass),
  owl_reified_class(Selection,ReifiedClass).
  
ogp_task_selection(IndividualTask,OutputDict,Selection) :-
  rdf_equal(IndividualTask,knowrob_planning:'Integration_OGP'),!,
  get_dict(
    'http://knowrob.org/kb/knowrob_planning.owl#Integration_OGP_Outcome',
    OutputDict,Selection).
  
ogp_task_selection(IndividualTask,OutputDict,Selection) :-
  rdf_equal(IndividualTask,knowrob_planning:'Nullification_OGP'),!,
  get_dict(
    'http://knowrob.org/kb/knowrob_planning.owl#Nullification_OGP_Outcome',
    OutputDict,Selection).
  
ogp_task_selection(IndividualTask,OutputDict,Selection) :-
  rdf_equal(IndividualTask,knowrob_planning:'Quantification_OGP'),!,
  get_dict(
    'http://knowrob.org/kb/knowrob_planning.owl#Quantification_OGP_Outcome',
    OutputDict,Selection).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % helper
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

decompose(S,P,Domain,O) :-
  owl_description(Domain, Descr),
  class_statements(Domain, Descr, Types_Explicit),
  %%%%%%%%%%
  %%% find set of types
  %%%%%%%%%%
  findall(Type, (
    (
      member(Type,Types_Explicit) ;
      owl_property_range_on_subject(S,P,Type) ;
      % FIXME owl_property_range_on_subject seems to not infer domain of inverse property
      (
        owl_inverse_property(P,P_inv),
        rdf_phas(P_inv, rdfs:domain, Type)
      )
    ),
    Type \= 'http://www.w3.org/2002/07/owl#Thing',
    \+ rdfs_individual_of(Type, owl:'Restriction')
  ), Types),
  %%%%%%%%%%
  %%% select the "main type"
  %%%%%%%%%%
  owl_most_specific(Types, O_type),
  ( rdf_has(O_type, owl:unionOf, Union)
  -> (
    rdfs_list_to_prolog_list(Union, Members),
    owl:owl_common_ancestor(Members, Type_selected) )
  ; Type_selected = O_type ),
  !,
  %%%%%%%%%%
  %%% debugging
  %%%%%%%%%%
  print_message(informational,
    format('Decomposed: (~w,~w,~w).', [S,P,Type_selected])),
  %%%%%%%%%%
  %%% create individual and assert type(s)
  %%%%%%%%%%
  rdf_instance_from_class(Type_selected, O),
  forall((
    member(Type,Types), 
    Type \= Type_selected,
    \+ rdf_has(Type, owl:unionOf, _)),
    rdf_assert(O, rdf:'type', Type)
  ),
  rdf_assert(S,P,O).

class_statements(_, class(Cls), [Cls]) :- !.
class_statements(_, intersection_of(Intersection), List) :-
  findall(X, member(class(X), Intersection), List), !.
class_statements(Cls, union_of(_), [Cls]) :- !.
class_statements(_, _, []).
