/** <module> Planning and configuration in technical domains
  
  Copyright (C) 2017 Daniel Beßler
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

@author Daniel Beßler
@license BSD

*/

:- module(knowrob_planning,
    [
      create_agenda/2,
      create_agenda/3,
      agenda_items/2,
      agenda_items_sorted/2,
      agenda_perform_next/1,
      agenda_write/1,
      agenda_item/1,
      agenda_item_type/2,
      agenda_item_property/2,
      agenda_item_domain/2,
      agenda_item_cardinality/2,
      agenda_item_description/2,
      agenda_item_strategy/2,
      agenda_item_domain_compute/2,
      agenda_item_project/2,
      agenda_item_update/2,
      agenda_item_continuity_value/2,
      agenda_item_inhibition_value/2,
      agenda_item_inhibit/1,
      agenda_item_matches_pattern/2,
      agenda_item_in_focus/1,
      agenda_item_last_selected/1,
      agenda_pattern_property/2,
      agenda_pattern_domain/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob_owl')).
:- use_module(library('owl_planning')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).

:-  rdf_meta
      create_agenda(r,r),
      create_agenda(r,r,r),
      agenda_items(r,t),
      agenda_items_sorted(r,t),
      agenda_perform_next(r),
      agenda_write(r),
      agenda_item(r),
      agenda_item_type(r,r),
      agenda_item_property(r,r),
      agenda_item_domain(r,r),
      agenda_item_cardinality(r,?),
      agenda_item_description(r,t),
      agenda_item_strategy(r,r),
      agenda_item_domain_compute(r,r),
      agenda_item_update(r,+),
      agenda_item_project(r,r),
      agenda_item_inhibit(r),
      agenda_item_inhibition_value(r,?),
      agenda_item_continuity_value(r,?),
      agenda_item_matches_pattern(r,r),
      agenda_item_in_focus(r),
      agenda_item_last_selected(t),
      agenda_pattern_property(r,r),
      agenda_pattern_domain(r,r).
%
% TODO(DB): some ideas ...
%   - Think about support for computables properties/SWRL
%   - Also add items caused by specializable type?
%   - Keep history of decisions. This would be required for stuff like truth maintanance system
%   - Keep sorted structure of agenda, sort-in new agenda items
%       - owl based representation requires many db interactions
%       - use prolog list simply?
%       - use foreign language instead?
%   - Make agenda items valid OWL 
%       - use owl_individual_of for pattern matching
%

%% create_agenda(+Obj,-Agenda)
%
create_agenda(Obj, Agenda) :-
  rdf_instance_from_class(knowrob_planning:'AgendaStrategy', Strategy),
  create_agenda(Obj, Strategy, Agenda).

%% create_agenda(+Obj,+Strategy,-Agenda)
%
create_agenda(Obj, Strategy, Agenda) :-
  rdf_instance_from_class(knowrob_planning:'Agenda', Agenda),
  rdf_assert(Agenda, knowrob_planning:'strategy', Strategy),
  agenda_add_object(Agenda, Obj).

%% agenda_items(+Agenda,-Items)
%
agenda_items([X|Xs], [X|Xs]) :- !.
agenda_items(Agenda, Items)  :- findall(X, rdf_has(Agenda,knowrob_planning:'agendaItem',X), Items).

%% agenda_item(?Item)
%
agenda_item(Item) :- rdfs_individual_of(Item, knowrob_planning:'AgendaItem').

%% agenda_item_last_selected(?Item)
%
agenda_item_last_selected(Item) :-
  current_predicate(agenda_item_last_selected_, _),
  agenda_item_last_selected_(Item).

assert_last_selected_item(Item) :-
  ( \+ current_predicate(agenda_item_last_selected_, _) ;
    retract( agenda_item_last_selected_(_) )),
  agenda_item_description(Item, Descr),
  assertz( agenda_item_last_selected_(Descr) ).

%% agenda_pop(+Agenda,-Item)
%
agenda_pop(Agenda, Item)  :-
  agenda_items_sorted(Agenda, [X|_]),
  ( agenda_item_valid(X)
  -> (
    agenda_item_inhibit(X), % count how often item was selected
    rdf_retract(Agenda, knowrob_planning:'agendaItem', X),
    assert_last_selected_item(X),
    Item=X
  ) ; (
    agenda_pop(Agenda, Item)
  )).

agenda_remove(Agenda, Item)  :-
  ingore( rdf_retract(Agenda, knowrob_planning:'agendaItem', Item) ).

%% agenda_pop(+Agenda,+Item)
%
agenda_push(Agenda, Item) :-
  ( rdf_has(Agenda, knowrob_planning:'agendaItem', Item) ;
    rdf_assert(Agenda, knowrob_planning:'agendaItem', Item)), !.

agenda_add_object(Agenda, Root) :-
  rdf_assert(Agenda, knowrob_planning:'strategy', Strategy),
  % compute the current part tree
  bagof( Part, part_of_workpiece(Root, Strategy, Part), Parts ),
  % for each part infer agenda items
  forall((
    member(Obj, Parts),
    owl_unsatisfied_restriction(Obj, Descr),
    owl_satisfies_restriction_up_to(Obj, Descr, Item)
  ) , ( % assert RDF triples
    assert_agenda_item(Item, Agenda, Obj, Descr, _)
  )).

agenda_remove_object(Agenda, Object) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  % compute the current part tree
  bagof( Part, part_of_workpiece(Object, Strategy, Part), Parts ),
  % for each part infer agenda items
  forall((
    member(Obj, Parts),
    rdf_has(Item, knowrob_planning:'itemOf', Obj)
  ) , ( % retract RDF triples
    retract_agenda_item(Item)
  )).

retract_agenda_item(Item) :-
  forall(( rdf_retract(Item, _, _);
           rdf_retract(_, _, Item)), true).

assert_agenda_item(Item, Agenda, Cause, Cause_restriction, ItemId) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  once((
    rdf_has(Strategy, knowrob_planning:'focus', Focus),
    Item=..[T,S,P,Domain|_],
    agenda_item_in_focus_internal(item(T,S,P,Domain,_), Focus)
  )),
  assert_agenda_item(Item,ItemId),
  % the condition under which the item is valid
  rdf_instance_from_class(knowrob_planning:'AgendaCondition', Condition),
  rdf_assert(Condition, knowrob_planning:'causedByRestrictionOn', Cause),
  rdf_assert(Condition, knowrob_planning:'causedByRestriction', Cause_restriction),
  rdf_assert(ItemId, knowrob_planning:'itemCondition', Condition),
  agenda_push(Agenda, ItemId).

assert_agenda_item(integrate(S,P,Domain,Count), Item) :-
  rdf_instance_from_class(knowrob_planning:'IntegrateAgendaItem', Item),
  assert_agenda_item_P(Item, (S,P,Domain,Count)).
assert_agenda_item(decompose(S,P,Domain,Count), Item) :-
  rdf_instance_from_class(knowrob_planning:'DecomposeAgendaItem', Item),
  assert_agenda_item_P(Item, (S,P,Domain,Count)).
assert_agenda_item(detach(S,P,Domain,Count), Item) :-
  rdf_instance_from_class(knowrob_planning:'DetachAgendaItem', Item),
  assert_agenda_item_P(Item, (S,P,Domain,Count)).
assert_agenda_item(classify(S,Domain), Item) :-
  rdf_instance_from_class(knowrob_planning:'ClassifyAgendaItem', Item),
  assert_agenda_item_D(Item, (S,Domain)).
  
assert_agenda_item_P(Item, (S,P,Domain,Count)) :-
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  rdf_assert(Item, knowrob_planning:'itemCardinality', literal(type(xsd:int, Count))).,
  rdf_assert(Item, P, Domain)
assert_agenda_item_D(Item, (S,Domain)) :-
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  rdf_assert(Item, knowrob_planning:'itemDomain', Domain).

agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  rdf_has(Item, knowrob_planning:'itemDomain', Domain),
  \+ owl_individual_of(S,Domain).
agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'), !,
  agenda_item_description(Item, item(_,S,P,Domain,(Cause,Restr))),
  owl_satisfies_restriction_up_to(Cause, Restr, unspecify(S,P,Domain,_)).
agenda_item_valid(Item) :-
  agenda_item_description(Item, item(_,S,P,Domain,(Cause,Restr))),
  owl_satisfies_restriction_up_to(Cause, Restr, specify(S,P,Domain,_)).

%% agenda_item_description(?Item,?Description)
%
agenda_item_description(Item, item(integrate,S,P,Domain,(Cause,Restr))) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'),
  agenda_item_description_internal_P(Item, item(S,P,Domain,(Cause,Restr)))
agenda_item_description(Item, item(decompose,S,P,Domain,(Cause,Restr))) :-
  rdfs_individual_of(Item, knowrob_planning:'DecomposeAgendaItem'),
  agenda_item_description_internal_P(Item, item(S,P,Domain,(Cause,Restr))).
agenda_item_description(Item, item(detach,S,P,Domain,(Cause,Restr))) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'),
  agenda_item_description_internal_P(Item, item(S,P,Domain,(Cause,Restr))).
agenda_item_description(Item, item(classify,S,nil,Domain,(Cause,Restr))) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'),
  agenda_item_description_internal_D(Item, item(S,Domain,(Cause,Restr))).

agenda_item_description_internal_P(Item, item(S,P,Domain,(Cause,Restr))) :-
  agenda_item_property(Item,P),
  agenda_item_description_internal_D(Item, item(S,Domain,(Cause,Restr))).
agenda_item_description_internal_D(Item, item(S,Domain,(Cause,Restr))) :-
  agenda_item_domain(Item,Domain),
  rdf_has(Item, knowrob_planning:'itemOf', S),
  rdf_has(Item, knowrob_planning:'itemCondition', Condition),
  rdf_has(Condition, knowrob_planning:'causedByRestrictionOn', Cause),
  rdf_has(Condition, knowrob_planning:'causedByRestriction', Restr).

%% agenda_item_type(?Item,?Type)
%
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'), !,
  rdf_equal(Type, knowrob_planning:'IntegrateAgendaItem').
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'DecomposeAgendaItem'), !,
  rdf_equal(Type, knowrob_planning:'DecomposeAgendaItem').
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'), !,
  rdf_equal(Type, knowrob_planning:'DetachAgendaItem').
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_equal(Type, knowrob_planning:'ClassifyAgendaItem').

%% agenda_item_property(?Item,?P)
%
agenda_item_property(Item,P) :-
  rdf_has(Item, P, _),
  \+ rdfs_subproperty_of(P, knowrob_planning:'agendaPredicate'),
  \+ rdf_equal(P, rdf:type).
  
%% agenda_item_domain(?Item,?Domain)
%
agenda_item_domain(Item,Domain) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_has(Item, knowrob_planning:'itemDomain', Domain).
agenda_item_domain(Item,Domain) :-
  agenda_item_property(Item,P),
  rdf_has(Item,P,Domain).

%% agenda_item_cardinality(?Item,?Card)
%
agenda_item_cardinality(Item,Card) :-
  rdf_has(Item, knowrob_planning:'itemCardinality', V),
  strip_literal_type(V, V_),
  atom_number(V_,Cardinality), !.
agenda_item_cardinality(Item,1).

agenda_item_update_cardinality(Item,Card) :-
  rdf_retract(Item, knowrob_planning:'itemCardinality', _),
  rdf_assert(Item, knowrob_planning:'itemCardinality', litela(type(xsd:int,Card))), !.

%% agenda_item_strategy(?Item,?Strategy)
%
agenda_item_strategy(Item, Strategy) :-
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
  rdf_has(Agenda, knowrob_planning:'agendaStrategy', Strategy).

agenda_item_specialize_domain(Item, Domain) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_retract(Item, knowrob_planning:'itemDomain', _),
  rdf_assert(Item, knowrob_planning:'itemDomain', Domain).
agenda_item_specialize_domain(Item, Domain) :-
  agenda_item_property(Item,P),
  ignore( rdf_retract(Item, P, _) ),
  rdf_assert(Item, P, Domain), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda filtering based on focussing knowledge

%% agenda_item_in_focus(+Item)
%
agenda_item_in_focus(Item) :-
  agenda_item_strategy(Item,Strategy),
  agenda_item_description(Item,Item_descr),
  rdf_has(Strategy, knowrob_planning:'focus', Focus),
  agenda_item_in_focus_internal(Item_descr,Focus).

agenda_item_in_focus_internal(item(T,S,P,O,_), Focus) :-
  rdfs_individual_of(Focus, knowrob_planning:'PatternFocus'), !,
  agenda_item_matches_pattern_internal(item(T,S,P,O,_), Pattern).

% build a tree following any relations focussed in the control strategy
part_of_workpiece(S, Strategy, Part) :- part_of_workpiece(S, Strategy, Part, []).
part_of_workpiece(Root, _, Root, []).
part_of_workpiece(S, Strategy, Part, Cache) :-
  \+ member(S, Cache), owl_has(S, P, O),
  rdfs_individual_of(P, owl:'ObjectProperty'),
  % ignore triples that are not part of current focus
  agenda_item_in_focus_internal(item(_,S,P,O,_),Strategy),
  part_of_workpiece(O, Strategy, Part, [S|Cache]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda sorting based on selection criteria

%% agenda_items_sorted(+Agenda,-Sorted)
%
agenda_items_sorted(Agenda, Sorted) :-
  agenda_items(Agenda, Items),
  predsort(compare_agenda_items, Items, Sorted).

compare_agenda_items(Delta, Item1, Item2) :-
  agenda_item_strategy(Item1, Startegy),
  agenda_item_strategy(Item2, Startegy),
  strategy_selection_criteria(Startegy, Criteria),
  compare_agenda_items(Delta, Criteria, Item1, Item2).

compare_agenda_items(Delta, [Criterium|Rest], Item1, Item2) :-
  agenda_item_selection_value(Item1, Criterium, Val1),
  agenda_item_selection_value(Item2, Criterium, Val2),
  (  Val1 is Val2
  -> compare_agenda_items(Delta, Rest, Item1, Item2)
  ; ( % "smaller" elements (higher priority) first
    Val1 > Val2 -> Delta='<' ;  Delta='>'
  )).

strategy_selection_criteria(Strategy, Criteria) :-
  % sort criteria according to their priority (high priority first)
  findall(C, rdf_has(Strategy, knowrob_planning:'selection', C), Cs),
  predsort(compare_selection_criteria, Cs, Criteria).

compare_selection_criteria(Delta, C1, C2) :-
  selection_priority(C1, V1),
  selection_priority(C2, V2),
  ( V1 < V2 -> Delta='>' ; Delta='<' ). % high priority first

selection_priority(C,V) :-
  rdf_has(C, knowrob_planning:'selectionPriority', Val),
  strip_literal_type(Val, Val_stripped),
  atom_number(Val_stripped, V), !.
selection_priority(_,0.0).

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'PatternSelection'),
  rdf_has(Criterium, knowrob_planning:'pattern', Pattern),
  ( agenda_item_matches_pattern(Item, Pattern) -> Val=1.0 ; Val=0.0 ).

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'InhibitionSelection'),
  agenda_item_inhibition_value(Item, InhibitionValue),
  Val is -InhibitionValue.

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'ContinuitySelection'),
  agenda_item_continuity_value(Item, Val).

%% agenda_item_continuity_value(+Item,?ContinuityValue)
%
agenda_item_continuity_value(Item,ContinuityValue) :-
  agenda_item_last_selected(LastItem),
  agenda_item_continuity_value_internal(Item,LastItem,ContinuityValue), !.
agenda_item_continuity_value(_,0.0).

agenda_item_continuity_value_internal(Item1, Item2, 1.0) :-
  rdf_has(Item1, knowrob_planning:itemOf, S),
  rdf_has(Item2, knowrob_planning:itemOf, S), !.
agenda_item_continuity_value_internal(_, _, 0.0).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item inhibition: marks items as n-times selected before

%% agenda_item_inhibit(+Item,?InhibitionValue)
%
agenda_item_inhibition_value(Item, InhibitionValue) :-
  rdf_has(Item, knowrob_planning:'inhibitionValue', X),
  strip_literal_type(X,X_stripped),
  atom_number(X_stripped, InhibitionValue), !.
agenda_item_inhibition_value(_, 0.0).

%% agenda_item_inhibit(+Item)
%
agenda_item_inhibit(Item) :-
  atom(Item),
  agenda_item_inhibition_value(Item, Old),
  New is Old + 1,
  assert_agenda_item_inhibition(Item, New).

assert_agenda_item_inhibition(Item, Val) :-
  ignore( rdf_retract(Item, knowrob_planning:'inhibitionValue', _) ),
  rdf_assert(Item, knowrob_planning:'inhibitionValue', literal(type(xsd:float, Val))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item pattern matching

%% agenda_item_matches_pattern(+Item,+Pattern)
%
agenda_item_matches_pattern(Item, Pattern) :-
  agenda_item_matches_item_type(Item, Pattern),
  agenda_item_matches_object(Item, Pattern),
  agenda_item_matches_property(Item, Pattern),
  agenda_item_matches_domain(Item, Pattern).

agenda_item_matches_item_type(Item, Pattern) :-
  rdfs_individual_of(Pattern, knowrob_planning:'AgendaItem')
  -> (
    rdfs_individual_of(Pattern, Cls),
    \+ rdf_equal(Cls, knowrob_planning:'AgendaItem'),
    owl_subclass_of(Cls, knowrob_planning:'AgendaItem'),
    rdfs_individual_of(Item, Cls)
  ) ; true.

agenda_item_matches_object(Item,Pattern) :-
  owl_restriction_on_property(Pattern, knowrob_planning:'itemOf', Restr)
  -> (
    rdf_has(Item, knowrob_planning:'itemOf', S),
    rdf_has(Restriction, owl:onClass, Cls),
    owl_individual_of(S,Cls)
  ) ; true.

agenda_item_matches_property(Item, Pattern) :-
  (  agenda_pattern_property(Pattern,P_Pattern)
  -> ( agenda_item_property(Item,P), rdfs_subproperty_of(P, P_Pattern) )
  ;  true ).

agenda_item_matches_domain(Item, Pattern) :-
  (  agenda_pattern_domain(Pattern,Domain_Pattern)
  -> ( agenda_item_domain(Item,Domain), owl_specialization_of(Domain, Domain_Pattern) )
  ;  true ).

%% agenda_pattern_property(?Item,?P)
%
agenda_pattern_property(Pattern,P) :-
  owl_restriction_on_property(Pattern,P,_),
  \+ rdfs_subproperty_of(P, knowrob_planning:'agendaPredicate'),
  \+ rdf_equal(P, rdf:type).

%% agenda_pattern_domain(?Item,?P)
%
agenda_pattern_domain(Pattern,Domain) :-
  agenda_pattern_property(Pattern,P), !,
  owl_restriction_on_property(Pattern,P,Restr),
  ( rdf_has(Restr, owl:onClass, Domain) ;
    rdf_has(Restr, owl:hasValue, Domain) ).
agenda_pattern_domain(Pattern,Domain) :-
  rdf_has(Pattern, knowrob_planning:'itemDomain', Domain).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item domain computaion

%% agenda_item_domain_compute(?Item,?Domain)
%
agenda_item_domain_compute(Item, Domain) :-
  agenda_item_property(Item, P),
  agenda_item_domain(Item, D_item),
  % find set of restrictions that caused items of S with property P
  bagof(D, (
    agenda_item_match(Item, X),
    agenda_item_property(X, P_X),
    once(rdfs_subproperty_of(P, P_X)),
    agenda_item_domain(X, D)
  ), Domains),
  owl_most_specific_specialization(D_item, Domains, Domain).


agenda_item_domain_atomic(Item, [X|Xs]) :-
  agenda_item_domain_atomic(Item, X),
  agenda_item_domain_atomic(Item, Xs).
agenda_item_domain_atomic(Item, Domain) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_has(Domain, rdf:'type', owl:'Class').
agenda_item_domain_atomic(_, Domain) :-
  owl_individual_of(Domain, _), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item processing

%% agenda_perform_next(+Agenda)
%
agenda_perform_next(Agenda) :-
  agenda_pop(Agenda, Item),
  agenda_perform(Item).

agenda_perform(Item) :-
  agenda_item_domain_compute(Item, DomainIn),
  agenda_item_cardinality(Item, Card),
  % Perform domain specialization Card times and project the specialized domain
  bagof( DomainOut, (
    between(1,Card,_),
    agenda_perform_specialization(Item, DomainIn, DomainOut),
    agenda_item_project(Item, DomainOut)
  ), Selection ),
  % Update agenda according to newly asserted knowledge
  (  agenda_item_domain_atomic(Item, Selection)
  -> agenda_item_update(Item, Selection)
  ;  agenda_push(Item) ).

agenda_perform_specialization(Item, DomainIn, DomainOut) :-
  agenda_item_description(Item, Descr),
  agenda_item_strategy(Item,Strategy),
  % search for perform descriptions
  findall(Perform, (
    rdf_has(Strategy, knowrob_planning:'performPattern', X),
    rdf_has(X, knowrob_planning:'pattern', Pattern),
    agenda_item_matches_pattern(Item, Pattern),
    rdf_has(X, knowrob_planning:'perform', Perform)
  ), PerformDescriptions),
  (  PerformDescriptions=[]
  -> agenda_perform_just_do_it(Descr, DomainIn, DomainOut)
  ;  (
    forall( member(Perform,PerformDescriptions),
            agenda_perform_description(Item,Descr,Perform,DomainIn,DomainOut) )
  )).

agenda_perform_description(Item,Descr,Perform,DomainIn,DomainOut) :-
  rdfs_individual_of(Perform, knowrob_planning:'AgendaPerformProlog'), !,
  rdf_has(Perform, knowrob_planning:'command', literal(type(_,Goal))),
  call(Goal, Item, Descr, DomainIn, DomainOut).

agenda_perform_just_do_it(_,item(classify,S,_,_,_), Domain, Domain).
agenda_perform_just_do_it(_,item(decompose,S,P,_,_), Domain, Domain).
agenda_perform_just_do_it(_,item(detach,S,P,_,_), Domain, Domain) :-
  owl_has(S,P,O), owl_individual_of(O,Domain).
agenda_perform_just_do_it(_,item(integrate,S,P,_,_), Domain, O) :-
  owl_individual_of(O,Domain), \+ owl_has(_,P,O).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item projection

%% agenda_item_project(+Item,+Domain)
%
agenda_item_project(Item, Domain) :-
  atom(Item),
  agenda_item_description(Item, Descr),
  (  agenda_item_domain_atomic(Item, Domain)
  -> agenda_item_project_internal(Descr, Domain)
  ;  agenda_item_specialize_domain(Item, Domain) ).

agenda_item_project_internal(item(integrate,S,P,_,_), O)   :- rdf_assert(S,P,O).
agenda_item_project_internal(item(decompose,S,P,_,_), Cls) :- decompose(S,P,Domain,O).
agenda_item_project_internal(item(detach,S,P,_,_), O)      :- rdf_retract(S,P,O).
agenda_item_project_internal(item(classify,S,_,_,_), Cls)  :- rdf_assert(S,rdf:'type',Cls).

class_statements(class(Cls), [Cls]).
class_statements(intersection_of(Intersection), List) :-
  findall(X, member(class(X), Intersection), List).
class_statements(_, []).

decompose(S,P,Domain,O) :-
  owl_description(Domain, Descr),
  class_statements(Descr,Types),
  % create an instance, use most specific specialization of property range
  once(rdf_phas(P, rdfs:'range', Range) ; Types=[Range|_]),
  owl_most_specific_specialization(Range, Types, O_type),
  rdf_instance_from_class(O_type, O),
  % assert additional types
  forall((member(Type,Types), Type \= O_type), rdf_assert(O,rdf:'type',Type)),
  rdf_assert(S,P,O).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item updating

%% agenda_item_update(+Item,+Selection)
%
agenda_item_update(Item,Selection) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  % find more general items
  bagof(X, (
    agenda_item_match(Item, X),
    agenda_item_domain(X, Domain_X),
    once((
      member(Cls,Selection),
      owl_subclass_of(Cls,Domain_X)
    ))
  ), GeneralItems),
  % retract old items
  forall( member(X,GeneralItems), retract_agenda_item(X) ).

agenda_item_update(Item,Selection) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'), !
  % find more general items
  agenda_item_generalizations(Item, Selection, GeneralItems),
  % retract old items
  length(Selection, Selection_count),
  forall( member(X,GeneralItems), (
    agenda_item_cardinality(X,Card),
    ( Selection_count >= Card ->
    ( retract_agenda_item(X) ) ; (
      NewCard is Card - Selection_count, 
      agenda_item_update_cardinality(X,NewCard),
      agenda_push(X)
    ))
  )),
  forall( member(X,Selection), agenda_remove_object(X) ).

agenda_item_update(Item,Selection) :- % decompose or integrate
  % find more general items
  agenda_item_generalizations(Item, Selection, GeneralItems),
  %%% For each more general item add "deeper" items
  length(Selection, Selection_count),
  forall( member(X,GeneralItems), (
    agenda_item_cardinality(X,Card),
    ( Selection_count >= Card
    ->  ( % remove item and add "deeper" items if enough values were selected
      agenda_item_update_specify(X,Selection),
      retract_agenda_item(X)
    ) ; ( % update the cardinality otherwise
      NewCard is Card - Selection_count, 
      agenda_item_update_cardinality(X,NewCard),
      agenda_push(X)
    ))
  )),
  %%% Add selected object items
  forall( member(X,Selection), agenda_add_object(X) ).


agenda_item_update_specify(Item,Selection) :-
  % find class description of O according to item
  agenda_item_domain(Item, O_descr),
  agenda_item_property(Item, P),
  agenda_item_cardinality(Item, Card),
  rdf_has(Item, knowrob_planning:'itemOf', S),
  owl_cardinality(S,P,O_descr,Count),
  % nothing todo if cardinality fully specified already
  (( Count >= Card ) ;
  ( % add items for each of the selected values which are not subclass of O description
    rdf_has(Item, knowrob_planning:'itemCondition', C),
    rdf_has(C, knowrob_planning:'causedByRestrictionOn', Cause),
    rdf_has(C, knowrob_planning:'causedByRestriction', Cause_restriction),
    rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
    forall((
      member(O,Selection), \+ owl_individual_of(O,O_descr)
    ),(
      owl_satisfies_restriction_up_to(O,O_descr,NewItem),
      assert_agenda_item(NewItem, Agenda, Cause, Cause_restriction, _)
    ))
  )).

agenda_item_generalizations(Item, Selection, GeneralItems) :-
  agenda_item_property(Item, P),
  bagof(X, (
    agenda_item_match(Item, X),
    agenda_item_property(X, P_X),
    agenda_item_domain(X, Domain_X),
    once((
      rdfs_subproperty_of(P, P_X),
      member(O,Selection),
      owl_individual_of(O, Domain_X)
    ))
  ), GeneralItems).

agenda_item_match(Item, Match) :-
  rdf_has(Item,  knowrob_planning:'itemOf', S),
  rdf_has(Match, knowrob_planning:'itemOf', S),
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
  rdf_has(Agenda, knowrob_planning:'agendaItem', Match),
  agenda_item_type(Item,Type),
  agenda_item_type(Match,Type).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda pretty printing

%% agenda_write(+Agenda)
%
agenda_write(Agenda) :-
  atom(Agenda),
  agenda_items_sorted(Items),
  agenda_write(Items).
agenda_write([X|Xs]) :-
  writeln('agenda('),
  forall(member(Item,[X|Xs]), (
    agenda_item_description(Item,Descr),
    write('  o '), agenda_item_write(Descr), nl,
  )), writeln(')').
  
agenda_item_write(item(classify,S,_,Domain,_)) :-
  write_name(S), write(' classify  '), write_name(Domain).
agenda_item_write(item(Operator,S,P,Domain,_)) :-
  write_name(S), write(' '), write(Operator), write(' '), write_name(P), write('.'), write_name(Domain).
write_name(X) :- rdf_split_url(_, X_, X), write(X_).