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
      agenda_item_project/3,
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
      agenda_item_project(r,r,r),
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
%   - pre-sort selection criteria in create_agenda
%       - link them with nextSelection property (would disallow them to appear in multiple agendas)
%   - Make agenda items valid OWL 
%       - biggest issue is the domain value that is a class description, not an individual
%       - annotation properties could be used but would disable standart OWL reasoning
%       - cleanest solution is to make agenda items class descriptions instead of individuals,
%         but still issues with item property representation
%       - use owl_specialization_of for pattern matching
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
  once(( \+ current_predicate(agenda_item_last_selected_, _) ;
         retract(agenda_item_last_selected_(_)) )),
  agenda_item_description(Item, Descr),
  assertz( agenda_item_last_selected_(Descr) ).

%% agenda_pop(+Agenda,-Item)
%
agenda_pop(Agenda, Item)  :-
  agenda_items_sorted(Agenda, [X|_]),
  ( agenda_item_valid(X)
  -> (
    write('    [SELECT] '), writeln(X),
    agenda_item_inhibit(X), % count how often item was selected
    %rdf_retractall(Agenda, knowrob_planning:'agendaItem', X),
    assert_last_selected_item(X),
    Item=X
  ) ; (
    retract_agenda_item(X),
    agenda_pop(Agenda, Item) % perform next one
  )).

%% agenda_pop(+Agenda,+Item)
%
agenda_push(Agenda, Item) :-
  once(( rdf_has(Agenda, knowrob_planning:'agendaItem', Item) ;
         rdf_assert(Agenda, knowrob_planning:'agendaItem', Item) )).

agenda_add_object(Agenda, Root) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  % compute the current part tree
  bagof( Part, part_of_workpiece(Root, Strategy, Part), Parts ),
  % for each part infer agenda items
  forall((
    member(Obj, Parts),
    owl_unsatisfied_restriction(Obj, Descr),
    owl_satisfies_restriction_up_to(Obj, Descr, Item),
    % only assert relevant items
    once((
      rdf_has(Strategy, knowrob_planning:'focus', Focus),
      Item=..[T,S,P,Domain|_],
      agenda_item_in_focus_internal(item(T,S,P,Domain,_), Focus)
    ))
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
  write('    [RETRACT] '), writeln(Item),
  rdf_retractall(Item, _, _),
  rdf_retractall(_, _, Item).

assert_agenda_item(Item, Agenda, Cause, Cause_restriction, ItemId) :-
  assert_agenda_item(Item,ItemId),
  % the violated restriction that caused the item
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
  rdf_assert(Item, knowrob_planning:'itemCardinality', literal(type(xsd:int, Count))),
  rdf_assert(Item, P, Domain).
assert_agenda_item_D(Item, (S,Domain)) :-
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  rdf_assert(Item, knowrob_planning:'itemDomain', Domain).

%% agenda_item_description(?Item,?Description)
%
agenda_item_description(item(T,S,P,Domain,Cause),
                        item(T,S,P,Domain,Cause)) :- !.
agenda_item_description(Item, item(integrate,S,P,Domain,(Cause,Restr))) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'),
  agenda_item_description_internal_P(Item, item(S,P,Domain,(Cause,Restr))).
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
agenda_item_type(item(integrate,_,_,_,_), Type) :-
  rdf_equal(Type, knowrob_planning:'IntegrateAgendaItem'), !.
agenda_item_type(item(decompose,_,_,_,_), Type) :-
  rdf_equal(Type, knowrob_planning:'DecomposeAgendaItem'), !.
agenda_item_type(item(detach,_,_,_,_), Type) :-
  rdf_equal(Type, knowrob_planning:'DetachAgendaItem'), !.
agenda_item_type(item(classify,_,_,_,_), Type) :-
  rdf_equal(Type, knowrob_planning:'ClassifyAgendaItem'), !.
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'),
  rdf_equal(Type, knowrob_planning:'IntegrateAgendaItem'), !.
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'DecomposeAgendaItem'),
  rdf_equal(Type, knowrob_planning:'DecomposeAgendaItem'), !.
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'),
  rdf_equal(Type, knowrob_planning:'DetachAgendaItem'), !.
agenda_item_type(Item, Type) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'),
  rdf_equal(Type, knowrob_planning:'ClassifyAgendaItem'), !.

%% agenda_item_object(?Item,?S)
%
agenda_item_object(item(_,S,_,_,_),S) :- !.
agenda_item_object(Item,S) :- rdf_has(Item, knowrob_planning:'itemOf', S).

%% agenda_item_property(?Item,?P)
%
agenda_item_property(item(_,_,P,_,_),P) :- !.
agenda_item_property(Item,P) :-
  % FIXME: it's not so nice to query for the property IRI like this.
  %        could yield unwanted triples if there are non agendaPredicate properties defined.
  %        in fact this is not the case here, but anyway.
  %        maybe use annotation property instead/additionally?
  rdf_has(Item, P, _),
  P \= inverse_of(_),
  \+ rdfs_subproperty_of(P, knowrob_planning:'agendaPredicate'),
  \+ rdfs_subproperty_of(P, knowrob_planning:'itemCardinality'),
  \+ rdf_equal(P, rdf:type), !.
  
%% agenda_item_domain(?Item,?Domain)
%
agenda_item_domain(item(_,_,_,Domain,_),Domain) :- !.
agenda_item_domain(Item,Domain) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'),
  rdf_has(Item, knowrob_planning:'itemDomain', Domain), !.
agenda_item_domain(Item,Domain) :-
  agenda_item_property(Item,P),
  rdf_has(Item,P,Domain), !.

%% agenda_item_cardinality(?Item,?Card)
%
agenda_item_cardinality(Item,Card) :-
  rdf_has(Item, knowrob_planning:'itemCardinality', literal(type(_,V))),
  ( number(V) -> Card is V ; atom_number(V,Card) ), !.
agenda_item_cardinality(_,1).

agenda_item_update_cardinality(Item,Card) :-
  rdf_retractall(Item, knowrob_planning:'itemCardinality', _),
  rdf_assert(Item, knowrob_planning:'itemCardinality', litela(type(xsd:int,Card))), !.

%% agenda_item_strategy(?Item,?Strategy)
%
agenda_item_strategy(Item, Strategy) :-
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy).

%% agenda_item_specialize_domain(?Item, +Domain)
%
% Note: it is assumed that the domain is in fact a specialization.
%       May need to be enforced before calling this.
%
agenda_item_specialize_domain(Item, Domain) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_retractall(Item, knowrob_planning:'itemDomain', _),
  rdf_assert(Item, knowrob_planning:'itemDomain', Domain).
agenda_item_specialize_domain(Item, Domain) :-
  agenda_item_property(Item,P),
  ignore( rdf_retractall(Item, P, _) ),
  rdf_assert(Item, P, Domain), !.

%% agenda_item_valid(?Item)
%
% NOTE: `agenda_item_update` ensures (to some extend) that agenda items are valid 
%       or else deleted. This is just an additional test to be safe before performing the item.
%
agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'), !,
  rdf_has(Item, knowrob_planning:'itemOf', S),
  rdf_has(Item, knowrob_planning:'itemDomain', Domain),
  \+ owl_individual_of(S,Domain).
agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'), !,
  agenda_item_description(Item, item(_,S,P,Domain,(Cause,Restr))),
  owl_satisfies_restriction_up_to(Cause, Restr, unspecify(S,P,UpToDomain,_)),
  owl_specialization_of(Domain,UpToDomain), !.
agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'DecomposeAgendaItem'), !,
  agenda_item_description(Item, item(_,S,P,Domain,(Cause,Restr))),
  owl_satisfies_restriction_up_to(Cause, Restr, decompose(S,P,UpToDomain,_)),
  owl_specialization_of(Domain,UpToDomain), !.
agenda_item_valid(Item) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'), !,
  agenda_item_description(Item, item(_,S,P,Domain,(Cause,Restr))),
  owl_satisfies_restriction_up_to(Cause, Restr, integrate(S,P,UpToDomain,_)),
  owl_specialization_of(Domain,UpToDomain), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda filtering based on focussing knowledge

%% agenda_item_in_focus(+Item)
%
agenda_item_in_focus(Item) :-
  agenda_item_strategy(Item,Strategy),
  rdf_has(Strategy, knowrob_planning:'focus', Focus),
  agenda_item_description(Item,Descr),
  agenda_item_in_focus_internal(Descr,Focus), !.

agenda_item_in_focus_internal(Item, Focus) :-
  rdfs_individual_of(Focus, knowrob_planning:'PatternFocus'), !,
  rdf_has(Focus, knowrob_planning:'pattern', Pattern),
  agenda_item_matches_pattern(Item, Pattern).

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

compare_agenda_items('<', [], _, _) :- !. % random selection
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
  rdfs_individual_of(Criterium, knowrob_planning:'PatternSelection'), !,
  rdf_has(Criterium, knowrob_planning:'pattern', Pattern),
  ( agenda_item_matches_pattern(Item, Pattern) -> Val=1.0 ; Val=0.0 ).

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'InhibitionSelection'), !,
  agenda_item_inhibition_value(Item, InhibitionValue),
  Val is -InhibitionValue.

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'ContinuitySelection'), !,
  agenda_item_continuity_value(Item, Val).

%% agenda_item_continuity_value(+Item,?ContinuityValue)
%
agenda_item_continuity_value(Item,ContinuityValue) :-
  agenda_item_last_selected(LastItem),
  agenda_item_continuity_value_internal(Item,LastItem,ContinuityValue), !.
agenda_item_continuity_value(_,0.0).

agenda_item_continuity_value_internal(Item1, item(_,S,_,_,_), 1.0) :-
  rdf_has(Item1, knowrob_planning:itemOf, S), !.
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
  rdf_retractall(Item, knowrob_planning:'inhibitionValue', _),
  rdf_assert(Item, knowrob_planning:'inhibitionValue', literal(type(xsd:float, Val))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item pattern matching

%% agenda_item_matches_pattern(+Item,+Pattern)
%
agenda_item_matches_pattern(Item, Pattern) :-
  agenda_item_description(Item,Descr),
  agenda_item_matches_item_type(Descr, Pattern),
  agenda_item_matches_object(Descr, Pattern),
  agenda_item_matches_property(Descr, Pattern),
  agenda_item_matches_domain(Descr, Pattern).

agenda_item_matches_item_type(Item, Pattern) :-
  rdfs_individual_of(Pattern, knowrob_planning:'AgendaItem')
  -> ( agenda_item_type(Item, Type), owl_subclass_of(Type, Pattern) )
  ; true.

agenda_item_matches_object(Item,Pattern) :-
  owl_restriction_on_property(Pattern, knowrob_planning:'itemOf', Restr)
  -> ( agenda_item_object(Item,S), owl_individual_of(S, Restr) )
  ;  true.

agenda_item_matches_property(Item, Pattern) :-
  agenda_pattern_property(Pattern,P_Pattern)
  -> ( agenda_item_property(Item,P), rdfs_subproperty_of(P, P_Pattern) )
  ;  true.

agenda_item_matches_domain(Item, Pattern) :-
  agenda_pattern_domain(Pattern,Domain_Pattern)
  -> ( agenda_item_domain(Item,Domain), owl_specialization_of(Domain, Domain_Pattern) )
  ;  true.

%% agenda_pattern_property(?Item,?P)
%
agenda_pattern_property(Pattern,P) :-
  % NOTE: same remark as for `agenda_item_property`. This could potentially yield unwanted properties.
  rdfs_individual_of(Pattern, Cls),
  rdfs_individual_of(Cls, owl:'Restriction'),
  rdf_has(Cls, owl:onProperty, P),
  \+ rdfs_subproperty_of(P, knowrob_planning:'agendaPredicate'),
  \+ rdf_equal(P, rdf:type).

%% agenda_pattern_domain(?Item,?P)
%
agenda_pattern_domain(Pattern,Domain) :-
  agenda_pattern_property(Pattern,P),
  owl_restriction_on_property(Pattern,P,Restr),
  once(( rdf_has(Restr, owl:onClass, Domain) ;
         rdf_has(Restr, owl:hasValue, Domain) )), !.
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
  owl_most_specific_specializations(D_item, Domains, [Domain|_]).


owl_atomic([]).
owl_atomic([X|Xs]) :- owl_atomic(X), owl_atomic(Xs).
owl_atomic(Domain) :- atom(Domain), rdf_has(Domain, rdf:'type', owl:'Class'), !.
owl_atomic(Domain) :- atom(Domain), owl_individual_of(Domain, _), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item processing

%% agenda_perform_next(+Agenda)
%
agenda_perform_next(Agenda) :-
  once(agenda_pop(Agenda, Item)), % FIXME: there shouldn't be more solutions
  agenda_perform(Item).

agenda_perform(Item) :-
  agenda_item_domain_compute(Item, DomainIn),
  agenda_item_cardinality(Item, Card),
  % Perform domain specialization Card times and project the specialized domain
  bagof( Selected, (
    between(1,Card,_),
    agenda_perform_specialization(Item, DomainIn, DomainOut),
    agenda_item_project(Item, DomainOut, Selected)
  ), Selection ),
  % Update agenda according to newly asserted knowledge
  (  owl_atomic(Selection)
  -> agenda_item_update(Item, Selection)
  ;  agenda_push(Item) ).

agenda_perform_specialization(Item, DomainIn, DomainOut) :-
  agenda_item_description(Item, Descr),
  agenda_item_strategy(Item,Strategy),
  % search for perform descriptions
  findall(Perform, (
    rdf_has(Strategy, knowrob_planning:'performPattern', X),
    rdf_has(X, knowrob_planning:'pattern', Pattern),
    rdf_has(X, knowrob_planning:'perform', Perform),
    agenda_item_matches_pattern(Descr, Pattern)
  ), PerformDescriptions),
  (  PerformDescriptions=[]
  -> agenda_perform_just_do_it(Item, Descr, DomainIn, DomainOut)
  ; (
     agenda_perform_descriptions(Item,Descr,PerformDescriptions,DomainIn,DomainOut)
  )).

agenda_perform_descriptions(Item,Descr,[Perform|Rest],DomainIn,DomainOut) :-
  agenda_perform_description(Item,Descr,Perform,DomainIn,DomainOutA),
  agenda_perform_descriptions(Item,Descr,Rest,DomainOutA,DomainOut).
agenda_perform_descriptions(_,_,[],DomainIn,DomainIn).

agenda_perform_description(Item,Descr,Perform,DomainIn,DomainOut) :-
  rdfs_individual_of(Perform, knowrob_planning:'AgendaPerformProlog'), !,
  rdf_has(Perform, knowrob_planning:'command', literal(type(_,Goal_atom))),
  term_to_atom(Goal,Goal_atom),
  call(Goal, Item, Descr, DomainIn, DomainOut).

agenda_perform_just_do_it(_,item(classify,_,_,_,_), Domain, Domain) :- !.
agenda_perform_just_do_it(_,item(decompose,_,_,_,_), Domain, Domain) :- !.
agenda_perform_just_do_it(_,item(detach,S,P,_,_), Domain, O) :-
  owl_has(S,P,O), owl_individual_of(O,Domain), !.
agenda_perform_just_do_it(_,item(integrate,_,P,_,_), Domain, O) :-
  owl_individual_of(O,Domain), \+ owl_has(_,P,O), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item projection

%% agenda_item_project(+Item,+Domain,-Selected)
%
agenda_item_project(Item, Domain, Selected) :-
  atom(Item),
  agenda_item_description(Item, Descr),
  (  owl_atomic(Domain)
  -> agenda_item_project_internal(Descr, Domain, Selected)
  ;  agenda_item_specialize_domain(Item, Domain) ), !.

agenda_item_project_internal(item(integrate,S,P,_,_), O, O)    :- rdf_assert(S,P,O), debug_assertion(S,P,O).
agenda_item_project_internal(item(decompose,S,P,_,_), Cls, O)  :- decompose(S,P,Cls,O).
agenda_item_project_internal(item(detach,S,P,_,_), O, O)       :- rdf_retractall(S,P,O), debug_retraction(S,P,O).
agenda_item_project_internal(item(classify,S,_,_,_), Cls, Cls) :- rdf_assert(S,rdf:'type',Cls), debug_type_assertion(S,Cls).

class_statements(class(Cls), [Cls]).
class_statements(intersection_of(Intersection), List) :-
  findall(X, member(class(X), Intersection), List).
class_statements(_, []).

decompose(S,P,Domain,O) :-
  owl_description(Domain, Descr),
  class_statements(Descr,Types),
  % create an instance, use most specific specialization of property range
  once(( rdf_phas(P, rdfs:'range', Range) ;
         Range='http://www.w3.org/2002/07/owl#Thing' )),
  owl_most_specific_specializations(Range, Types, [O_type|_]),
  rdf_instance_from_class(O_type, O),
  % assert additional types
  forall((member(Type,Types), Type \= O_type), rdf_assert(O,rdf:'type',Type)),
  % assert decomposition
  rdf_assert(S,P,O),
  debug_assertion(S,P,O).

debug_retraction(S,P,O) :-
  write('    [RETRACT] '), write_name(S), write(' --'), write_name(P), write('--> '), write_name(O), writeln('.').
debug_assertion(S,P,O) :-
  write('    [ASSERT] '),  write_name(S), write(' --'), write_name(P), write('--> '), write_name(O), writeln('.').
debug_type_assertion(S,Cls) :-
  write('    [ASSERT] '),  write_name(S), write(' type '), write_name(Cls), writeln('.').

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
  % retract classify items with more general domain
  forall( member(X,GeneralItems), retract_agenda_item(X) ).

agenda_item_update(Item,Selection) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'), !,
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
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
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
  forall( member(X,Selection), agenda_add_object(Agenda,X) ).


agenda_item_update_specify(Item,Selection) :-
  % find class description of O according to item
  agenda_item_domain(Item, O_descr),
  agenda_item_property(Item, P),
  agenda_item_cardinality(Item, Card),
  rdf_has(Item, knowrob_planning:'itemOf', S),
  owl_cardinality(S,P,O_descr,Count),
  % nothing todo if cardinality fully specified already
  ( Count >= Card ;
  ( % add items for each of the selected values which are not subclass of O description
    rdf_has(Item, knowrob_planning:'itemCondition', C),
    rdf_has(C, knowrob_planning:'causedByRestrictionOn', Cause),
    rdf_has(C, knowrob_planning:'causedByRestriction', Cause_restriction),
    rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
    forall((
      member(O,Selection), \+ owl_individual_of(O,O_descr),
      owl_satisfies_restriction_up_to(O,O_descr,NewItem)
    ),(
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
      % domain could impose some unsattisfied restriction,
      % thus check for specializable and not for individual_of
      owl_specializable(O, Domain_X)
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
  agenda_items_sorted(Agenda,Items),
  agenda_write(Items).
agenda_write([X|Xs]) :-
  writeln('    [AGENDA]'),
  forall(member(Item,[X|Xs]), (
    agenda_item_description(Item,Descr),
    write('        o '), agenda_item_write(Descr), nl
  )).

agenda_item_write(item(classify,S,_,Domain,_)) :-
  write_name(S), write(' classify  '), write_name(Domain).
agenda_item_write(item(Operator,S,P,Domain,_)) :-
  write(Operator), write(' '),  write_name(S), write(' --'), write_name(P), write('--> '), write_description(Domain).

write_name(inverse_of(P)) :- write('inverse_of('), write_name(P), write(')'), !.
write_name(literal(type(_,V))) :- write(V), !.
write_name(X) :- atom(X), rdf_split_url(_, X_, X), write(X_).
write_description(Domain) :-
  atom(Domain),
  owl_description(Domain,Descr),
  rdf_readable(Descr,Readable),
  write(Readable).

rdf_readable(class(Cls),Out) :- rdf_readable_internal(Cls,Out), !.
rdf_readable(Descr,Out) :-
  (is_list(Descr) -> X=Descr ; Descr=..X),
  findall(Y_, (member(X_,X), once(rdf_readable_internal(X_,Y_))), Y),
  Out=Y.
rdf_readable_internal(class(X),Y) :- rdf_readable_internal(X,Y).
rdf_readable_internal(X,Y) :- atom(X), rdf_split_url(_, Y, X).
rdf_readable_internal(X,X) :- atom(X).
rdf_readable_internal(X,Y) :- compound(X), rdf_readable(X,Y).
