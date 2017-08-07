/** <module> Planning according to OWL specifications
  
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
      agenda_create/3,
      agenda_perform_next/1,
      agenda_items/2,
      agenda_items_sorted/2,
      agenda_item_type/2,
      agenda_item_property/2,
      agenda_item_domain/2,
      agenda_item_cardinality/2,
      agenda_item_description/2,
      agenda_item_strategy/2,
      agenda_item_matches_pattern/2,
      agenda_item_in_focus/1,
      agenda_pattern_property/2,
      agenda_pattern_domain/2,
      agenda_write/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob_owl')).
:- use_module(library('owl_planning')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).

:-  rdf_meta
      agenda_create(r,r,r),
      agenda_perform_next(r),
      agenda_items(r,-),
      agenda_items_sorted(r,-),
      agenda_item_type(t,r),
      agenda_item_property(t,r),
      agenda_item_domain(t,r),
      agenda_item_cardinality(t,?),
      agenda_item_description(r,t),
      agenda_item_strategy(r,r),
      agenda_item_matches_pattern(r,r),
      agenda_item_in_focus(r),
      agenda_pattern_property(r,r),
      agenda_pattern_domain(r,r),
      agenda_write(r).

:- dynamic agenda_selection_criteria_sorted/2,
           agenda_item_last_selected/1.
%
% TODO(DB): some ideas ...
%   - Think about support for computables properties/SWRL
%   - Also add items caused by specializable type?
%   - Keep history of decisions. for instance union descriptions may require this!
%

%% agenda_create(+Obj,+Strategy,-Agenda)
%
% Create new agenda based on inconsistencies of the resource `Obj` (and children).
% The generated agenda reflects knowledge base operations that need to
% be performed in order to transform the object to a consistent state wrt. 
% its class description.
% Only properties which are focussed in `Strategy` are considered.
%
% @param Obj       Potentially inconsistent object
% @param Strategy  Planning strategy
% @param Agenda    The agenda created
%
agenda_create(Obj, Strategy, Agenda) :-
  rdf_instance_from_class(knowrob_planning:'Agenda', Agenda),
  rdf_assert(Agenda, knowrob_planning:'strategy', Strategy),
  assertz(agenda_items_sorted(Agenda,[])),
  agenda_add_object(Agenda, Obj, 0).

%% agenda_items(+Agenda,-Items)
%
% The OWL representation of agenda item individuals is given by:
%   AgendaItem_XYY
%     type ItemType                                # one of DecomposeAgendaItem,...
%     type (itemOf only (Predicate some Domain))   # for decompose/integrate/detach items
%     type (itemOf only Domain)                    # for classify items
%     itemOf Subject                               # underspecified individual
%     itemCardinality Cardinality                  # needed additional cardinality or 1 for classify items
%     itemCondition Condition                      # description of unsattisfied restriction that caused the item
%       causedByRestrictionOn CauseSubject
%       causedByRestriction   CauseRestriction     # FIXME: not valid OWL!
%
% Agenda item patterns have the same structure. Each item which is a specialization
% of the pattern is matched.
%
% @param Agenda Planning agenda
% @param Items  Unordered agenda items
%
agenda_items([X|Xs], [X|Xs]) :- !.
agenda_items(Agenda, Items)  :- findall(X, rdf_has(Agenda,knowrob_planning:'agendaItem',X), Items).

%% agenda_items_sorted(+Agenda,-Items)
%
% @param Agenda Planning agenda
% @param Items  Sorted agenda items (sorted according to planning strategy)
%
:- dynamic agenda_items_sorted/2.

%% agenda_pop(+Agenda,-Item,-Descr)
%
agenda_pop(Agenda, Item, Descr)  :-
  agenda_items_sorted(Agenda, [X|RestItems]),
  agenda_items_sorted_update(Agenda, RestItems),
  agenda_item_description(X, X_Descr),
  ( agenda_item_valid(X_Descr, X)
  -> (
    % count how often item was selected
    agenda_item_inhibit(X),
    % remember last selected item
    retractall(agenda_item_last_selected(_)),
    assertz(   agenda_item_last_selected(X_Descr)),
    Item=X, Descr=X_Descr
  ) ; ( % retract invalid, pop next
    retract_agenda_item(X),
    agenda_pop(Agenda, Item, Descr)
  )).

%% agenda_push(+Agenda,+Item)
%
agenda_push(Agenda, Item) :-
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item) ; (
    rdf_assert(Agenda, knowrob_planning:'agendaItem', Item),
    agenda_sort_in(Agenda, Item)
  ), !.

%% agenda_add_object(+Agenda,+Root,+Depth)
%
agenda_add_object(Agenda, Root, Depth) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  % for each part infer agenda items
  forall(part_of_workpiece(Root, Strategy, Depth, (Obj,Obj_depth)),
         agenda_add_object_without_children(Agenda, Obj, Obj_depth)).

agenda_add_object_without_children(Agenda, Obj, Depth) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  forall(( % for each unsattisfied restriction add agenda items
     owl_unsatisfied_restriction(Obj, Descr),
     satisfies_restriction_up_to(Obj, Descr, Item),
     agenda_item_description_in_focus(Item,Strategy)
  ), assert_agenda_item(Item, Agenda, Obj, Descr, Depth, _)).

assert_agenda_item(Item, Agenda, Cause, Cause_restriction, Depth, ItemId) :-
  assert_agenda_item(Item,ItemId),
  agenda_item_depth_assert(ItemId,Depth),
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
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  assert_agenda_item_domain(Item, nil, Domain).
assert_agenda_item_P(Item, (S,P,Domain,Count)) :-
  rdf_assert(Item, knowrob_planning:'itemOf', S),
  rdf_assert(Item, knowrob_planning:'itemCardinality', literal(type(xsd:int, Count))),
  assert_agenda_item_domain(Item, P, Domain).

% TODO: wouldn't it be more elegant to use rdf:type instead of nil?
assert_agenda_item_domain(Item, nil, Domain) :-
  rdf_node(Id),
  rdf_assert(Id, rdf:type, owl:'Restriction'),
  rdf_assert(Id, owl:onProperty, knowrob_planning:'itemOf'),
  rdf_assert(Id, owl:allValuesFrom, Domain),
  rdf_assert(Item, rdf:'type', Id).
assert_agenda_item_domain(Item, P, Domain) :-
  rdf_node(Id1), rdf_node(Id2),
  rdf_assert(Id1, rdf:type, owl:'Restriction'),
  rdf_assert(Id1, owl:onProperty, knowrob_planning:'itemOf'),
  rdf_assert(Id1, owl:allValuesFrom, Id2),
  rdf_assert(Id2, rdf:type, owl:'Restriction'),
  rdf_assert(Id2, owl:onProperty, P),
  rdf_assert(Id2, owl:someValuesFrom, Domain),
  rdf_assert(Item, rdf:'type', Id1).

retract_agenda_item_domain(Item) :-
  forall((
    rdfs_individual_of(Item, Restr),
    rdf_has(Restr, owl:onProperty, knowrob_planning:'itemOf')
  ),(
    rdf_retractall(_, _, Restr),
    rdf_retractall(Restr, _, _)
  )).

%% agenda_remove_object(+Agenda,+Object)
%
agenda_remove_object(Agenda, Object) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  bagof( Part, part_of_workpiece(Object, Strategy, 0, Part), Parts ),
  forall(( % for each part retract agenda items
     member((Obj,_), Parts),
     agenda_item_subject(Item,Obj)
  ), retract_agenda_item(Item)).

retract_agenda_item(Item) :-
  % make sure item is deleted from the agenda item list
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
  agenda_items_sorted(Agenda, Items),
  delete(Items, Item, Items_new),
  agenda_items_sorted_update(Agenda, Items_new),
  % retract all the rdf facts
  rdf_retractall(Item, _, _),
  rdf_retractall(_, _, Item).

%% agenda_item_description(?Item,?Description)
%
% Map RDF agenda item to Prolog term representation of the form:
%    item(ItemType,Subject,Predicate,Domain,Cause)
% Where `Cause` is a tuple of a resource and restriction on resource
% that caused this item.
%
% @param Item        RDF agenda item
% @param Description Prolog agenda item
%
agenda_item_description(item(T,S,P,Domain,Reason),
                        item(T,S,P,Domain,Reason)) :- !.
agenda_item_description(Item, item(integrate,S,P,Domain,Reason)) :-
  rdfs_individual_of(Item, knowrob_planning:'IntegrateAgendaItem'),
  agenda_item_description_internal(Item, item(S,P,Domain,Reason)), !.
agenda_item_description(Item, item(decompose,S,P,Domain,Reason)) :-
  rdfs_individual_of(Item, knowrob_planning:'DecomposeAgendaItem'),
  agenda_item_description_internal(Item, item(S,P,Domain,Reason)), !.
agenda_item_description(Item, item(detach,S,P,Domain,Reason)) :-
  rdfs_individual_of(Item, knowrob_planning:'DetachAgendaItem'),
  agenda_item_description_internal(Item, item(S,P,Domain,Reason)), !.
agenda_item_description(Item, item(classify,S,nil,Domain,Reason)) :-
  rdfs_individual_of(Item, knowrob_planning:'ClassifyAgendaItem'),
  agenda_item_description_internal(Item, item(S,Domain,Reason)), !.
agenda_item_description_internal(Item, item(S,P,Domain,Reason)) :-
  rdf(Item, knowrob_planning:'itemOf', S),
  agenda_item_property(Item,P),
  agenda_item_domain(Item,Domain),
  agenda_item_reason(Item,Reason).

%% agenda_item_type(?Item,?Type)
%
% Maps an agenda item to its type (i.e., one of integrate, decompose, classify, ...)
%
% @param Item Agenda item
% @param Type Type of the item
%
agenda_item_type(item(integrate,_,_,_,_), 'http://knowrob.org/kb/knowrob_planning.owl#IntegrateAgendaItem') :- !.
agenda_item_type(item(decompose,_,_,_,_), 'http://knowrob.org/kb/knowrob_planning.owl#DecomposeAgendaItem') :- !.
agenda_item_type(item(detach,_,_,_,_),    'http://knowrob.org/kb/knowrob_planning.owl#DetachAgendaItem')    :- !.
agenda_item_type(item(classify,_,_,_,_),  'http://knowrob.org/kb/knowrob_planning.owl#ClassifyAgendaItem')  :- !.
agenda_item_type(Item, Type) :-
  rdf(Item, rdf:type, Type),
  rdfs_subclass_of(Type, knowrob_planning:'AgendaItem'), !.

%% agenda_item_subject(?Item,?S)
%
% Maps an agenda item to its subject (i.e., the object for which
% new specifications need to be asserted).
%
% @param Item Agenda item
% @param S    The subject of the agenda item
%
agenda_item_subject(item(_,S,_,_,_),S) :- !.
agenda_item_subject(Item,S) :- rdf_has(Item, knowrob_planning:'itemOf', S), !.

%% agenda_item_property(?Item,?P)
%
% Maps an agenda item to its predicate (i.e., the predicate which
% needs to be specified for the items subject).
%
% @param Item Agenda item
% @param P    The predicate of the agenda item
%
agenda_item_property(item(_,_,P,_,_),P) :- !.
agenda_item_property(Item,P) :-
  rdf(Item, rdf:type, Restr),
  rdf(Restr, owl:onProperty, knowrob_planning:'itemOf'),
  rdf(Restr, owl:allValuesFrom, PropertyRestr),
  rdf(PropertyRestr, owl:onProperty, P), !.
agenda_item_property(_Item,nil).
  
%% agenda_item_domain(?Item,?Domain)
%
% Maps an agenda item to its domain (i.e., the class description
% of values to be specified for the items subject along the items predicate).
%
% @param Item    Agenda item
% @param Domain  The domain of the agenda item
%
agenda_item_domain(item(_,_,_,Domain,_),Domain) :- !.
agenda_item_domain(Item,Domain) :-
  rdf(Item, rdf:type, Restr),
  rdf(Restr, owl:onProperty, knowrob_planning:'itemOf'),
  rdf(Restr, owl:allValuesFrom, ItemOfType),
  ( rdf(ItemOfType, owl:someValuesFrom, Domain) ;
    Domain = ItemOfType % classify item
  ), !.

% NOTE: it is assumed that the domain is in fact a specialization.
%       May need to be enforced before calling this.
agenda_item_specialize_domain(Item, Domain) :-
  agenda_item_property(Item,P),
  retract_agenda_item_domain(Item),
  assert_agenda_item_domain(Item,P,Domain).
  
%% agenda_item_reason(?Item,?Reason)
%
% Maps an agenda item to its reason (i.e., the unsattisfied restriction
% that caused this item to be generated).
%
% @param Item    Agenda item
% @param Reason  Tuple of a resource and restriction on resource that caused the item
%
agenda_item_reason(item(_,_,_,_,Reason),Reason) :- !.
agenda_item_reason(Item, (Cause,Restr)) :-
  rdf(Item, knowrob_planning:'itemCondition', Condition),
  rdf(Condition, knowrob_planning:'causedByRestrictionOn', Cause),
  rdf(Condition, knowrob_planning:'causedByRestriction', Restr), !.

%% agenda_item_cardinality(?Item,?Card)
%
% Maps an agenda item to its cardinality (i.e., the number of
% values that need to be specified in order to fullfill a restriction).
%
% @param Item  Agenda item
% @param Card  The cardinality of the agenda item
%
agenda_item_cardinality(Item,Card) :-
  rdf(Item, knowrob_planning:'itemCardinality', literal(type(_,V))),
  ( number(V) -> Card is V ; atom_number(V,Card) ), !.
agenda_item_cardinality(_,1).

agenda_item_update_cardinality(Item,Card) :-
  rdf_retractall(Item, knowrob_planning:'itemCardinality', _),
  ( atom(Card) -> Card_atom=Card ; atom_number(Card_atom,Card) ),
  rdf_assert(Item, knowrob_planning:'itemCardinality', litela(type(xsd:int,Card_atom))), !.

%% agenda_item_strategy(?Item,?Strategy)
%
% Maps an agenda item to its planning strategy.
%
% @param Item      Agenda item
% @param Strategy  The planning strategy
%
agenda_item_strategy(Item, Strategy) :-
  rdf_has(Agenda, knowrob_planning:'agendaItem', Item),
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy).

%% agenda_item_valid(?Item)
%
% NOTE: `agenda_item_update` ensures (to some extend) that agenda items are valid 
%       or else deleted. This is just an additional test to be safe before performing the item.
%
agenda_item_valid(item(classify,S,_,Domain,(Cause,Restr)), _Item) :- !,
  satisfies_restriction_up_to(Cause, Restr, classify(S,UpToDomain)),
  owl_specializable(Domain,UpToDomain), !.

agenda_item_valid(item(_,S,P,Domain,(Cause,Restr)), _Item) :-
  satisfies_restriction_up_to(Cause, Restr, X),
  X=..[_,S,P,UpToDomain,_],
  owl_specializable(Domain,UpToDomain), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda filtering based on focussing knowledge

%% agenda_item_in_focus(+Item)
%
% True iff the item is focussed according to its planning
% strategy. The focus is mainly determined using pattern matching
% on the item: The strategy declares agenda item patterns of focussed
% items and here we check if the given item matches one of the patterns.
%
% @param Item Agenda item
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

agenda_item_description_in_focus(Item,Strategy) :-
  (( Item=..[item,T,S,P,Domain|_] ;
   ( Item=..[T,S,P,Domain|_] ) ;
   ( Item=..[T,S,Domain], P=nil ))),
  rdf_has(Strategy, knowrob_planning:'focus', Focus),
  ( agenda_item_in_focus_internal(item(T,S,P,Domain,_), Focus) ; (
    rdf_has(P, owl:'inverseOf', P_inv), % also focus if inverse property focussed
    rdfs_type_of(S, Type),
    agenda_item_in_focus_internal(item(T,S,P_inv,Type,_), Focus)
  )).

%% part_of_workpiece(+S,+Strategy,+Depth,-Part)
%
% Build a sequence of plannable resources starting from `S` and
% following any relations focussed in the control strategy.
% 
part_of_workpiece(S, Strategy, Depth, Part) :- part_of_workpiece(S, Strategy, Depth, Part, []).
part_of_workpiece(Root, _, Depth, (Root,Depth), []).
part_of_workpiece(S, Strategy, Depth, Part, Cache) :-
  \+ member(S, Cache),
  once((
    rdf(S, P, O), atom(P),
    rdfs_individual_of(P, owl:'ObjectProperty'),
    % ignore "unfocussed" triples
    agenda_item_description_in_focus(item(_,S,P,O,_),Strategy)
  )),
  Depth_next is Depth + 1,
  part_of_workpiece(O, Strategy, Depth_next, Part, [S|Cache]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda sorting based on selection criteria

agenda_items_sorted_update(Agenda, SortedItems) :-
  retractall(agenda_items_sorted(Agenda,_)),
  assertz(   agenda_items_sorted(Agenda,SortedItems)).

%% agenda_sort_in(+Agenda,+Item)
%
agenda_sort_in(Agenda, Item) :-
  rdf_has(Agenda, knowrob_planning:'strategy', Strategy),
  strategy_selection_criteria(Strategy, Criteria),
  agenda_items_sorted(Agenda, Items),
  agenda_sort_in_(Criteria, Item, Items, Items_new),
  agenda_items_sorted_update(Agenda, Items_new).

agenda_sort_in_(_, Elem, [], [Elem]) :- !.
agenda_sort_in_(Criteria, Elem, [Head|Tail], [Elem,Head|Tail]) :-
  compare_agenda_items('<', Criteria, Elem, Head), !.
agenda_sort_in_(Criteria, Elem, [Head|Tail], [Head|Tail_sorted]) :-
  agenda_sort_in_(Criteria, Elem, Tail, Tail_sorted).

compare_agenda_items(Delta, Item1, Item2) :-
  agenda_item_strategy(Item1, Startegy),
  agenda_item_strategy(Item2, Startegy),
  strategy_selection_criteria(Startegy, Criteria),
  compare_agenda_items(Delta, Criteria, Item1, Item2).

compare_agenda_items('>', [], _, _) :- !. % no selection criterium left -> random selection
compare_agenda_items(Delta, [Criterium|Rest], Item1, Item2) :-
  agenda_item_selection_value(Item1, Criterium, Val1),
  agenda_item_selection_value(Item2, Criterium, Val2),
  (  Val1 is Val2
  -> compare_agenda_items(Delta, Rest, Item1, Item2)
  ; ( % "smaller" elements (higher priority) first
     Val1 > Val2 -> Delta='<' ;  Delta='>'
  )).

strategy_selection_criteria(Strategy, Criteria) :-
  agenda_selection_criteria_sorted(Strategy, Criteria), !.
strategy_selection_criteria(Strategy, Criteria) :-
  % sort criteria according to their priority (high priority first)
  findall(C, rdf_has(Strategy, knowrob_planning:'selection', C), Cs),
  predsort(compare_selection_criteria, Cs, Criteria),
  assertz(agenda_selection_criteria_sorted(Strategy, Criteria)).

compare_selection_criteria(Delta, C1, C2) :-
  selection_priority(C1, V1), selection_priority(C2, V2),
  ( V1 =< V2 -> Delta='>' ; Delta='<' ). % high priority first

selection_priority(C,V) :-
  rdf_has(C, knowrob_planning:'selectionPriority', Val),
  strip_literal_type(Val, Val_stripped),
  atom_number(Val_stripped, V), !.
selection_priority(_,0).

%% agenda_item_selection_value(+Item,+Criterium,?Val)
%
agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'PatternSelection'), !,
  rdf_has(Criterium, knowrob_planning:'pattern', Pattern),
  ( agenda_item_matches_pattern(Item, Pattern) -> Val=1 ; Val=0 ).

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'InhibitionSelection'), !,
  agenda_item_inhibition_value(Item, InhibitionValue),
  Val is -InhibitionValue.

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'ContinuitySelection'), !,
  agenda_item_continuity_value(Item, Val).

agenda_item_selection_value(Item, Criterium, Val) :-
  rdfs_individual_of(Criterium, knowrob_planning:'DepthSelection'), !,
  agenda_item_depth_value(Item, Val).

%% agenda_item_depth_value(+Item,?DepthValue)
%
agenda_item_depth_value(Item,DepthValue) :-
  rdf_has(Item, knowrob_planning:'itemDepth', literal(type(_,V))),
  ( number(V) -> DepthValue=V ; atom_number(V, DepthValue) ).

agenda_item_depth_assert(Item,DepthValue) :-
  rdf_retractall(Item, knowrob_planning:'itemDepth', _),
  ( atom(DepthValue) -> DepthValue_atom=DepthValue ; atom_number(DepthValue_atom,DepthValue) ),
  rdf_assert(Item, knowrob_planning:'itemDepth', literal(type(xsd:int, DepthValue_atom))).

%% agenda_item_continuity_value(+Item,?ContinuityValue)
%
agenda_item_continuity_value(Item,ContinuityValue) :-
  agenda_item_last_selected(LastItem),
  agenda_item_continuity_value_internal(Item,LastItem,ContinuityValue), !.
agenda_item_continuity_value(_,0).

agenda_item_continuity_value_internal(Item1, item(_,S,_,_,_), 1) :- agenda_item_subject(Item1,S), !.
agenda_item_continuity_value_internal(_, _, 0).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item inhibition: marks items as n-times selected before

%% agenda_item_inhibit(+Item,?InhibitionValue)
%
agenda_item_inhibition_value(Item, InhibitionValue) :-
  rdf_has(Item, knowrob_planning:'inhibitionValue', X),
  strip_literal_type(X,X_stripped),
  (number(X_stripped) -> InhibitionValue=X_stripped ; atom_number(X_stripped, InhibitionValue)), !.
agenda_item_inhibition_value(_, 0).

%% agenda_item_inhibit(+Item)
%
agenda_item_inhibit(Item) :-
  atom(Item),
  agenda_item_inhibition_value(Item, Old),
  New is Old + 1,
  assert_agenda_item_inhibition(Item, New).

assert_agenda_item_inhibition(Item, Val) :-
  rdf_retractall(Item, knowrob_planning:'inhibitionValue', _),
  ( atom(Val) -> Val_atom=Val ; atom_number(Val_atom,Val) ),
  rdf_assert(Item, knowrob_planning:'inhibitionValue', literal(type(xsd:int, Val_atom))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item pattern matching

%% agenda_item_matches_pattern(+Item,+Pattern)
%
% True iff `Item` is an instantiation of `Pattern`.
% Patterns are OWL class descriptions with restrictions imposed
% on different properties of agenda items.
%
% NOTE: Item might be a Prolog term which is not suitable for `owl_specializable`.
%       This is to allow checking if a not yet generated item matches a pattern
%       without actually asserting the item to RDF store.
%       RDF agenda items can be matched with the following code:
%          owl_specializable(Pattern, Item).
%
% @param Item Agenda item
% @param Item Agenda item pattern
%
agenda_item_matches_pattern(Item, Pattern) :-
  once(agenda_item_description(Item,Descr)),
  once(agenda_item_matches_item_type(Descr, Pattern)),
  once(agenda_item_matches_object(Descr, Pattern)),
  once(agenda_item_matches_property(Descr, Pattern)),
  once(agenda_item_matches_domain(Descr, Pattern)).

agenda_item_matches_item_type(Item, Pattern) :-
  ( rdf(Pattern, rdf:type, Pattern_Type),
    rdfs_subclass_of(Pattern_Type, knowrob_planning:'AgendaItem') )
  *-> (
    agenda_item_type(Item, Type),
    rdfs_subclass_of(Type, Pattern_Type) )
  ; true.

agenda_item_matches_object(Item,Pattern) :-
  rdf_has(Pattern, knowrob_planning:'itemOf', S)
  -> agenda_item_subject(Item,S)
  ;  true.

agenda_item_matches_property(Item, Pattern) :-
  agenda_pattern_property(Pattern,P_Pattern)
  -> (
    agenda_item_property(Item,P),
    % TODO: this should be code of owl_subproperty_of, and be used by owl_specializable
    ( rdfs_subproperty_of(P, P_Pattern) ;
    ( rdf_has(P, owl:inverseOf, P_inv),
      rdf_has(P_Pattern, owl:inverseOf, P_Pattern_inv),
      rdfs_subproperty_of(P_inv, P_Pattern_inv) ))
  ) ;  true.

agenda_item_matches_domain(Item, Pattern) :-
  agenda_pattern_domain(Pattern,Domain_Pattern)
  -> ( agenda_item_domain(Item,Domain), owl_specializable(Domain_Pattern, Domain) )
  ;  true.

agenda_pattern_property(Pattern,P)    :- agenda_item_property(Pattern,P), P \= nil.
agenda_pattern_domain(Pattern,Domain) :- agenda_item_domain(Pattern,Domain).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item domain/property inference

%% agenda_item_domain_compute(+Items,?Domain)
%
agenda_item_domain_compute(Items, Domain) :-
  findall(D, (
    member(Item, Items),
    agenda_item_domain(Item, Item_D),
    % TODO: also consider range of P on S here?
    %owl_property_range_on_subject(S,P,Domain)
    (  rdfs_individual_of(Item_D, owl:'Restriction')
    -> owl_restriction_subject_type(Item_D, D)
    ;  D = Item_D
    )
  ), Domains),
  owl_most_specific(Domains, Domain), !.

%% agenda_item_domain_compute(+Items,?P_specific)
%
agenda_item_property_compute(Items, P_specific) :-
  findall(P, (
    member(Item, Items),
    agenda_item_property(Item, P)
  ), Predicates),
  owl_most_specific_predicate(Predicates, P_specific), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item performing
% NOTE: performing specializes the items domain only, and does not assert selection to triple store!

%% agenda_perform_next(+Agenda)
%
% Pop agenda item with highest priority from agenda,
% perform the agenda item (i.e., specify triple for the
% items subject), and update the agenda accordingly.
%
% @param Agenda Planning agenda
%
agenda_perform_next(Agenda) :-
  agenda_pop(Agenda, Item, Descr),
  agenda_perform(Agenda, Item, Descr).

agenda_perform(Agenda, Item, Descr) :-
  agenda_item_siblings(Item, Siblings),
  agenda_item_domain_compute(Siblings, Domain_inferred),
  agenda_item_property_compute(Siblings, P_inferred),
  agenda_item_cardinality(Item, Card),
  % build term Operation(S,P_inferred) with Operation one of integrate, classify, ...
  Descr=..[_,Operation,S|_], PerformDescr=..[Operation,S,P_inferred],
  % Perform domain specialization Card times and project the specialized domain
  findall(Selected, (
    between(1,Card,_),
    agenda_perform_specialization(Item, Descr,
        PerformDescr, Domain_inferred, Domain_computed),
    agenda_item_project(Item,
        PerformDescr, Domain_computed, Selected)
  ), Selection ),
  % Update agenda according to asserted knowledge
  (  owl_atomic(Selection)
  -> agenda_item_update(PerformDescr, Agenda, Item, Siblings, Selection)
  ;  agenda_push(Agenda, Item) ).

agenda_perform_specialization(Item, Descr, PerformDescr, DomainIn, DomainOut) :-
  agenda_item_strategy(Item,Strategy),
  findall(Perform, (
    rdf_has(Strategy, knowrob_planning:'performPattern', X),
    rdf_has(X, knowrob_planning:'pattern', Pattern),
    rdf_has(X, knowrob_planning:'perform', Perform),
    agenda_item_matches_pattern(Descr, Pattern)
  ), PerformMethods),
  (  PerformMethods=[]
  -> agenda_perform_just_do_it(  PerformDescr, Item, DomainIn, DomainOut)
  ;  agenda_perform_descriptions(PerformDescr, Item, PerformMethods, DomainIn, DomainOut)
  ).

agenda_perform_descriptions(Descr,Item,[Perform|Rest],DomainIn,DomainOut) :-
  agenda_perform_description(Descr,Item,Perform,DomainIn,DomainOutA),
  agenda_perform_descriptions(Descr,Item,Rest,DomainOutA,DomainOut).
agenda_perform_descriptions(_,_,[],DomainIn,DomainIn).

agenda_perform_description(Descr,Item,Perform,DomainIn,DomainOut) :-
  rdfs_individual_of(Perform, knowrob_planning:'AgendaPerformProlog'), !,
  rdf_has(Perform, knowrob_planning:'command', literal(type(_,Goal_atom))),
  term_to_atom(Goal,Goal_atom),
  call(Goal, Descr, Item, DomainIn, DomainOut).

%% agenda_perform_just_do_it(+Descr,+Item,+DomainIn,-DomainOut)
%
% Perform with random object selection (in case of integrate/detach),
% and just use the domain for projection otherwhise (in case of decompose/classify).
agenda_perform_just_do_it(integrate(_,P), _, Domain, O) :-
  owl_individual_of(O, Domain),
  % enforce unique values for inverse functional properties
  (( rdfs_subproperty_of(P, Super),
     rdfs_individual_of(Super, owl:'InverseFunctionalProperty'))
  -> \+ rdf_has(_, P, O) ; true ), !.
agenda_perform_just_do_it(decompose(_,_), _, Domain, Domain) :- !.
agenda_perform_just_do_it( classify(_,_), _, Domain, Domain) :- !.
agenda_perform_just_do_it(   detach(S,P), _, Domain, O) :-
  rdf_has(S,P,O), owl_individual_of(O,Domain), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item projection
% NOTE: This actually asserts triples in the RDF store in case the domain of the agenda item is unique.

%% agenda_item_project(+Item,+Domain,-Selected)
%
agenda_item_project(Item, Descr, Domain, Selected) :-
  (  owl_atomic(Domain)
  -> agenda_item_project_internal(Descr, Domain, Selected)
  ; (agenda_item_specialize_domain(Item, Domain), Selected=Domain) ), !.

agenda_item_project_internal(decompose(S,P), Cls, O)   :- decompose(S,P,Cls,O), !.
agenda_item_project_internal(integrate(S,P),   O, O)   :- agenda_assert_triple(S,P,O), !.
agenda_item_project_internal( classify(S,_), Cls, Cls) :- planning_assert(S,'http://www.w3.org/1999/02/22-rdf-syntax-ns#type',Cls), !.
agenda_item_project_internal(   detach(S,P),   O, O)   :- planning_retract(S,P,O), !.

decompose(S,P,Domain,O) :-
  owl_description(Domain, Descr),
  % infer type of O
  class_statements(Domain, Descr, Types_Explicit),
  % TODO: better do this in perform method -> generate intersection of types to be asserted
  findall(Type, ((
    member(Type,Types_Explicit) ;
    owl_property_range_on_subject(S,P,Type) ),
    Type \= 'http://www.w3.org/2002/07/owl#Thing',
    \+ rdfs_individual_of(Type, owl:'Restriction')
  ), Types),
  once((
    owl_most_specific(Types, O_type),
    ( rdf_has(O_type, owl:unionOf, Union)
    -> (
      rdfs_list_to_prolog_list(Union, Members),
      owl:owl_common_ancestor(Members, Type_selected)
    ) ; Type_selected = O_type )
  )),
  % assert decomposition facts
  rdf_unique_id(Type_selected, O),
  planning_assert(O,'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Type_selected),
  forall((
    member(Type,Types), 
    Type \= Type_selected,
    \+ rdf_has(Type, owl:unionOf, _)),
    planning_assert(O, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Type)
  ),
  agenda_assert_triple(S,P,O).

%% agenda_assert_triple(+S,+P,+O)
%
agenda_assert_triple(S,P,O) :-
  rdf_has(P, owl:'inverseOf', P_inv), !,
  agenda_assert_triple_(O,P_inv,S).
agenda_assert_triple(S,P,O) :-
  agenda_assert_triple_(S,P,O).
agenda_assert_triple_(S,P,O) :-
  % infer more specific property type
  % NOTE: this is a look-ahead because it's not possible yet to specialize the property later
  %       can also not be done earlier because S may be the selected value for the item.
  % FIXME: this seems like a hack
  findall(P_restr, (
    rdfs_individual_of(S, Restr),
    once((
      rdfs_individual_of(Restr, owl:'Restriction'),
      % restriction on subproperty of P
      rdf_has(Restr, owl:onProperty, P_restr),
      rdfs_subproperty_of(P_restr, P),
      % and O is an individual of the restricted range
      owl_restriction_object_domain(Restr, Restr_cls),
      owl_individual_of(O, Restr_cls)
    ))
  ), Predicates),
  owl_most_specific_predicate([P|Predicates], P_specific),
  % assert the relation with more specific predicate P_specific
  planning_assert(S,P_specific,O), !.

planning_assert(S,P,O)  :- rdf_assert(S,P,O),     debug_assertion(S,P,O).
planning_retract(S,P,O) :- rdf_retractall(S,P,O), debug_retraction(S,P,O).

debug_retraction(S,P,O) :-
  write('    [RETRACT] '), write_name(S), write(' --'), write_name(P), write('--> '), write_name(O), writeln('.').
debug_assertion(S,P,O) :-
  write('    [ASSERT] '),  write_name(S), write(' --'), write_name(P), write('--> '), write_name(O), writeln('.').
debug_type_assertion(S,Cls) :-
  write('    [ASSERT] '),  write_name(S), write(' type '), write_name(Cls), writeln('.').

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Agenda item updating

%% agenda_item_update(+Agenda,+Item,+Siblings,+Selection)
%
agenda_item_update(classify(S,_),Agenda,Item,_Siblings,_Selection) :- !,
  % FIXME: remove other items created for disjunction of classes? or classify domain union?
  agenda_item_reason(Item, (Cause,_)),
  agenda_item_depth_value(Item,Depth),
  % the new class of the object could have additional restrictions on properties
  % just re-add the object so that these cause items on the agenda
  ( Cause = S -> Obj_depth is Depth ; Obj_depth is Depth + 1 ),
  agenda_add_object_without_children(Agenda, S, Obj_depth),
  % collect all obsolote classify items and retract
  findall(X, (
    agenda_item_match(Item, X),
    agenda_item_description(X, X_Descr),
    \+ agenda_item_valid(X_Descr, X)
  ), ObsoloteItems),
  forall( member(X,ObsoloteItems), retract_agenda_item(X) ).

agenda_item_update(detach(_,_),Agenda,_Item,Siblings,Selection) :- !,
  length(Selection, Selection_count),
  forall( member(X,Siblings), (
    agenda_item_cardinality(X,Card),
    ( Selection_count >= Card ->
    % FIXME: ensure that selection matches item domain
    ( retract_agenda_item(X) ) ; (
      NewCard is Card - Selection_count, 
      agenda_item_update_cardinality(X,NewCard),
      agenda_push(Agenda, X)
    ))
  )),
  forall( member(X,Selection), agenda_remove_object(X) ).

agenda_item_update(_,Agenda,Item,Siblings,Selection) :-
  agenda_item_depth_value(Item,Depth), Depth_next is Depth + 1,
  % for each more general item add "deeper" items
  length(Selection, Selection_count),
  forall( member(X,Siblings), (
    agenda_item_cardinality(X,Card),
    ( Selection_count >= Card
    -> ( % remove item and add "deeper" items if enough values were selected
      (  agenda_item_update_specify(Agenda,X,Selection)
      -> retract_agenda_item(X)
      ;  agenda_push(Agenda, X) )
    ) ; ( % update the cardinality otherwise
      NewCard is Card - Selection_count, 
      agenda_item_update_cardinality(X,NewCard),
      agenda_push(Agenda, X)
    ))
  )),
  % Add selected object items
  forall(member(X,Selection),
         agenda_add_object(Agenda,X,Depth_next)).

agenda_item_update_specify(Agenda,Item,Selection) :-
  agenda_item_domain(Item,O_descr),
  agenda_item_reason(Item,(Cause,Cause_restriction)),
  agenda_item_depth_value(Item, Depth),
  % add items for each of the selected values which are not subclass of O description
  forall((
     member(O,Selection),
     \+ owl_individual_of(O,O_descr)),(
     satisfies_restriction_up_to(O,O_descr,NewItem),
     assert_agenda_item(NewItem, Agenda, Cause, Cause_restriction, Depth, _)
  )), !.

%% agenda_item_siblings(+Item,-Siblings)
%
agenda_item_siblings(Item, Siblings) :-
  agenda_item_property(Item, P),
  agenda_item_domain(Item, Domain),
  findall(X, (
    agenda_item_match(Item, X),
    once((X=Item ; (
      agenda_item_property(X, P_X),
      ( P=P_X ; % NOTE: needed for classify items (P=nil)
        rdfs_subproperty_of(P, P_X) ;
        rdfs_subproperty_of(P_X, P) ),
      agenda_item_domain(X, Domain_X),
      % TODO: need to check in both directions?
      %( owl_specializable(Domain, Domain_X) ;
      %  owl_specializable(Domain_X, Domain) )
      owl_specializable(Domain_X, Domain)
    )))
  ), Siblings).

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
  agenda_write_(Items).
agenda_write_(Items) :-
  writeln('    [AGENDA]'),
  writeln('    -------------------------------------------------'),
  forall(member(Item,Items), (
    agenda_item_description(Item,Descr),
    write('    o '),
    
    agenda_item_strategy(Item,Strategy),
    strategy_selection_criteria(Strategy, Criteria),
    findall(Val, (
      member(C,Criteria),
      once(agenda_item_selection_value(Item,C,Val))
    ), SelectionVals),
    write(SelectionVals), write(' '),
    
    agenda_item_write(Descr), nl
  )),
  writeln('    -------------------------------------------------').

agenda_item_write(Atom) :-
  atom(Atom),
  agenda_item_description(Atom,Descr),
  agenda_item_write(Descr).
agenda_item_write(item(classify,S,_,Domain,_)) :-
  write_name(S), write(' classify  '), write_name(Domain).
agenda_item_write(item(Operator,S,P,Domain,_)) :-
  write(Operator), write(' '),  write_name(S), write(' --'), write_name(P), write('--> '), write_description(Domain).

write_name(inverse_of(P)) :- write('inverse_of('), write_name(P), write(')'), !.
write_name(P) :- atom(P), rdf_has(P, owl:inverseOf, Inv), write('inverse_of('), write_name(Inv), write(')'), !.
write_name(literal(type(_,V))) :- write(V), !.
write_name(X) :- atom(X), rdf_split_url(_, X_, X), write(X_).
write_description(Domain) :-
  atom(Domain),
  owl_description_recursive(Domain,Descr),
  rdf_readable(Descr,Readable), write(Readable).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Utility predicates

owl_atomic([]).
owl_atomic([X|Xs]) :- owl_atomic(X), owl_atomic(Xs).
owl_atomic(Domain) :- atom(Domain), rdf_has(Domain, rdf:'type', owl:'Class'), !.
owl_atomic(Domain) :- atom(Domain), owl_individual_of(Domain, _), !.

decomposable_property(P) :-
  rdfs_subproperty_of(P, knowrob_planning:'decomposablePredicate'), !.
decomposable_property(P1) :-
  rdf_has(P1, owl:inverseOf, P_inv),
  rdf_has(P2, owl:inverseOf, P_inv),
  rdfs_subproperty_of(P2, knowrob_planning:'decomposablePredicate'), !.

class_statements(_, class(Cls), [Cls]).
class_statements(_, intersection_of(Intersection), List) :-
  findall(X, member(class(X), Intersection), List).
class_statements(Cls, union_of(_), [Cls]).
class_statements(_, _, []).

satisfies_restriction_up_to(Cause, Restr, UpTo) :-
  owl_satisfies_restriction_up_to(Cause, Restr, X),
  ( X=specify(S,P,Domain,Card)
  -> ( decomposable_property(P) ->
         UpTo=decompose(S,P,Domain,Card) ;
         UpTo=integrate(S,P,Domain,Card)
  ) ; (
    UpTo=X
  )).
