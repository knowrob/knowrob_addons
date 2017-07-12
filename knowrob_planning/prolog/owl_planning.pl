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

:- module(owl_planning,
    [
      owl_specializable/2,
      owl_specialization_of/2,
      owl_satisfies_restriction_up_to/3,
      owl_most_specific_specializations/3,
      owl_restriction_on_property/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).

:-  rdf_meta
      owl_most_specific_specializations(r,t,-),
      owl_specializable(r,t),
      owl_specialization_of(r,r),
      owl_satisfies_restriction_up_to(r,t,t),
      owl_restriction_on_property(r,r,r).


owl_restriction_on_property(Resource, Property, Restriction) :-
  rdfs_individual_of(Resource, Cls),
  rdfs_individual_of(Cls, owl:'Restriction'),
  rdf_has(Restriction, owl:onProperty, Property).

  
%% owl_most_specific_specializations(?Base, ?Types, ?List)
%
owl_most_specific_specializations(Base, Types, List) :-
  % TODO: redundant with owl_type_of
  % TODO: would this work:
  %             owl_most_specific_specializations(`P some Thing`, [`P (1 1) Thing`], _)])
  %             owl_most_specific_specializations(`P any Thing`, [`P (1 1) Thing`], _)])
  %             owl_most_specific_specializations(`P (0 2) Thing`, [`P (1 1) Thing`], _)])
  bagof(Cls, (
    member(Cls, [Base|Types]),
    once(owl_subclass_of(Cls,Base)),
    % ensure there is no class in Types that is more specific then Cls
    forall((
      member(Cls_other, Types),
      Cls \= Cls_other
    ), (
      \+ owl_subclass_of(Cls_other, Cls)
    ))
  ), List).

%% owl_satisfies_restriction_up_to(?Resource, ?Restr, ?UpTo)
%
% 
% TODO: generate detach items for objects that vialote restrictions and are not specializable.
%       seems that it might be best to exclusively generate detach items with this mechanism.
%       then changing strategy could yield in detach items if wanted.

owl_satisfies_restriction_up_to(Resource, Restr, UpTo) :-
  owl_description(Restr, Descr),
  % TODO: think about this, caller can't distinguish between "can't" and "no need to"
  owl_specializable_(Resource,Descr),
  owl_satisfies_restriction_up_to_internal(Resource, Descr, X),
  ( X=specify(S,P,Domain,Card)
  -> (
    once(rdfs_subproperty_of(P, knowrob_planning:'decomposablePredicate')) ->
      UpTo=decompose(S,P,Domain,Card) ;
      UpTo=integrate(S,P,Domain,Card)
  ) ; (
    UpTo=X
  )).

owl_satisfies_restriction_up_to_internal(S, class(Cls), classify(S,Cls)) :-
  \+ owl_individual_of(S,Cls). % Cls is a atomic class

owl_satisfies_restriction_up_to_internal(S, intersection_of(List), UpTo) :-
  %forall(member(Cls,List), owl_specializable(S,Cls)),
  % for each class description Cls, check up to where S fullfills the description
  member(Cls,List),
  \+ owl_individual_of(S,Cls),
  owl_description(Cls, Cls_descr),
  owl_satisfies_restriction_up_to_internal(S, Cls_descr, UpTo).

owl_satisfies_restriction_up_to_internal(S, union_of(List), UpTo) :-
  % for each class description Cls in List to which S can be specialized, check up to where S fullfills the description
  % note that it is not really possible to provide a class selection list from within this methhod because
  % descriptions in List could be complex class descriptions -> must be handled in item domain computation
  member(Cls,List),
  \+ owl_individual_of(S,Cls),
  owl_description(Cls, Cls_descr),
  owl_specializable_(S, Cls_descr),
  owl_satisfies_restriction_up_to_internal(S, Cls_descr, UpTo).

owl_satisfies_restriction_up_to_internal(S, restriction(P,has_value(O)), specify(S,P,O,1)) :-
  \+ owl_has(S, P, O). % violation if value is not specified

owl_satisfies_restriction_up_to_internal(S, restriction(P,all_values_from(Cls)), UpTo) :-
  owl_description(Cls,Cls_descr),
  bagof(O, owl_has(S, P, O), Os), member(O,Os),
  \+ owl_individual_of(O, Cls), % For each value of P that is not instance of Cls check if it can be specialized
  (  owl_specializable_(O, Cls_descr)
  -> (  % O is specializable to Cls, thus restr. is fullfilled up to were O violates Cls
     owl_satisfies_restriction_up_to_internal(O, Cls_descr, UpTo)
  ) ; ( % O is not specializable, thus restr. is fullfilled up to (S P O)
     %owl_specializable_(S,restriction(P,some_values_from(Cls))),
     UpTo=detach(S,P,O,1) % FIXME: this will never be the case because owl_specializable_ called in owl_satisfies_restriction_up_to
  )).

owl_satisfies_restriction_up_to_internal(S, restriction(P,some_values_from(Cls)), UpTo) :-
  owl_description(Cls,Cls_descr),
  ( bagof(O, (owl_has(S, P, O), owl_specializable_(O, Cls_descr)), Os)
  -> (  % there are some values specializable to Cls, thus fullfilled up to were O \in Os violates Cls
    member(O,Os),
    owl_satisfies_restriction_up_to_internal(O, Cls_descr, UpTo)
  ) ; ( % no value of P is specializable to Cls, thus fullfilled up to (S P O)
    %owl_specializable_(S,restriction(P,some_values_from(Cls))),
    UpTo=specify(S,P,Cls,1)
  )).

owl_satisfies_restriction_up_to_internal(S, restriction(P,cardinality(Min,Max,Cls)), UpTo) :-
  owl_cardinality(S, P, Cls, Card),
  ( Card < Min
  -> (  % not enough values of type Cls
  owl_specializable_(S,restriction(P,some_values_from(Cls))),
    % find values of P that can be specializable to Cls
    once((bagof(O, (
      owl_has(S, P, O),
      \+ owl_individual_of(O, Cls),
      owl_specializable(O, Cls)
    ), Os) ; Os=[] )),
    owl_satisfies_min_cardinality_up_to(S, Card, Os,
        restriction(P,cardinality(Min,Max,Cls)), UpTo)
  ) ; ( % to many values of type Cls
    Count is Card - Max, Count > 0,
    UpTo=detach(S,P,Cls,Card) % FIXME: this will never be the case because owl_specializable_ called in owl_satisfies_restriction_up_to
  )).

owl_satisfies_min_cardinality_up_to(_, _, Os, restriction(_,cardinality(_,_,Cls)), UpTo) :-
  % some values of P could be specializable to Cls, for those Restr is fullfilled up to were they violate Cls 
  owl_description(Cls,Cls_descr),
  % find out up to which point the specializable objects satisfy the description
  member(O,Os),
  owl_satisfies_restriction_up_to_internal(O, Cls_descr, UpTo).
owl_satisfies_min_cardinality_up_to(S, Card, Os, restriction(P,cardinality(Min,_,Cls)), specify(S,P,Cls,Count)) :-
  % if not enough values are specializable, then it also violates the cardinality restriction
  length(Os, O_count), % number of specializable values
  Count is Min - Card - O_count,
  Count > 0.

%% owl_specialization_of(+Specific, +General)
% 
owl_specialization_of(Specific, General) :-
  rdfs_individual_of(Specific, owl:'Class'),!,
  owl_subclass_of(Specific,General).
owl_specialization_of(Specific, General) :-
  rdfs_individual_of(General, owl:'Class'),!,
  owl_individual_of(Specific, General).
owl_specialization_of(Resource, Resource).

%% owl_specializable(?Resource, ?Description)
%
% 
owl_specializable(Resource, Description) :-
  owl_description(Description, Description_pl),
  owl_specializable_(Resource, Description_pl).

owl_specializable_(Resource, class(Cls)) :-
  owl_individual_of(Resource, Cls), !.
owl_specializable_(Resource, class(Cls)) :-
  rdfs_individual_of(Resource, owl:'Class'),!,
  once(owl_subclass_of(Cls, Resource)).
owl_specializable_(Resource, class(Cls)) :-
  % specializable if one of the most specific types of resource is a generalization of Cls
  owl_type_of(Resource,  Cls_General),
  owl_subclass_of(Cls, Cls_General), !.

owl_specializable_(Resource, intersection_of(List)) :-
  % specializable if resource is specializable to all classes of the intersection
  forall( member(Cls,List), owl_specializable(Resource, Cls) ).

owl_specializable_(Resource, union_of(List)) :-
  % specializable if resource is specializable to at least one of the classes of the union
  member(Cls,List), owl_specializable(Resource, Cls), !.

owl_specializable_(Resource, restriction(P,all_values_from(Cls))) :-
  % specializable if all values of P are specializable to Cls
  owl_description(Cls, Cls_descr),
  once(( bagof(O, owl_has(Resource, P, O), Os) ; Os=[] )),
  forall( member(X,Os), owl_specializable_(X, Cls_descr) ), !.

owl_specializable_(Resource, restriction(P,some_values_from(Cls))) :-
  % specializable if one of the existing values can be specialized to Cls
  owl_description(Cls, Cls_descr),
  owl_has(Resource, P, O),
  owl_specializable_(O, Cls_descr), !.
owl_specializable_(Resource, restriction(P,some_values_from(Cls))) :-
  % specializable if it is consistent to add a new value of type Cls for P
  owl_decomposable_on_subject(Resource, P, Cls),
  owl_property_range_on_subject(Resource, P, intersection_of(List)),
  forall(member(Range,List), owl_subclass_of(Cls, Range)), !.

owl_specializable_(Resource, restriction(P,cardinality(Min,Max,Cls))) :-
  % specializable if cardinality is ok already
  owl_cardinality(Resource, P, Cls, Count_Cls),
  Count_Cls =< Max,
  Count_Cls >= Min, !.
owl_specializable_(Resource, restriction(P,cardinality(Min,_,Cls))) :-
  % specializable if we can add `Count_decompose` instances of Cls to satisfy min cardinality
  owl_cardinality(Resource, P, Cls, Count_Cls),
  Count_Cls < Min,
  Count_decompose is Min - Count_Cls,
  owl_decomposable_on_subject(Resource, P, Cls, Count_decompose),
  owl_property_range_on_subject(Resource, P, intersection_of(List)),
  forall(member(Range,List), owl_subclass_of(Cls, Range)), !.

owl_specializable_(Resource, restriction(P,has_value(O))) :-
  % specializable if P already has this value
  owl_has(Resource,P,O), !.
owl_specializable_(Resource, restriction(P,has_value(O))) :-
  % or specializable if we can add O as new value
  owl_property_range_on_subject(Resource, P, intersection_of(List)),
  forall(member(Range,List), owl_individual_of(O, Range)),
  owl_type_of(O, Cls),
  owl_decomposable_on_subject(Resource, P, Cls), !.


owl_decomposable_on_subject(Resource, P, Cls) :-
  owl_decomposable_on_subject(Resource, P, Cls, 1).
owl_decomposable_on_subject(Resource, P, Cls, Count_decompose) :-
  ( owl_cardinality_on_subject(Resource, P, Cls, cardinality(_,Max)) % infer cardinality restriction
  -> (
    owl_cardinality(Resource, P, Cls, Count),
    Max >= Count + Count_decompose
  ) ; true ).


owl_type_of(Resource, Cls) :-
  bagof(X, rdf_has(Resource, rdf:type, X), Types),
  member(Cls, Types),
  % ensure there is no class in Types that is more specific then Cls
  forall((
    member(Cls_other, Types),
    Cls \= Cls_other
  ), (
    \+ owl_subclass_of(Cls_other, Cls)
  )).
