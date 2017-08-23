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

:- module(owl_planning,
    [
      owl_specializable/2,
      owl_specialization_of/2,
      owl_satisfies_restriction_up_to/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl')).
:- use_module(library('knowrob_owl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).

:-  rdf_meta
      owl_specializable(r,t),
      owl_specialization_of(r,r),
      owl_satisfies_restriction_up_to(r,t,t).

%% owl_specialization_of(+Specific, +General)
% 
owl_specialization_of(Resource, Resource).
owl_specialization_of(Specific, General) :-
  rdfs_individual_of(Specific, owl:'Class'),!,
  atom(General), owl_subclass_of(Specific,General).
owl_specialization_of(Specific, General) :-
  atom(General), owl_individual_of(Specific, General).

%% owl_specializable(?Resource, ?Description)
%
% Infers if a resource can be specialized to a class description
% only with adding some facts (and without removing facts).
% Thus the resource is specializable if we can add facts to it
% so that it is transformed to a individual of or subclass of
% the class description.
%
% @param Resource RDF resource (class description or named individual)
% @param Description OWL class description
%
owl_specializable(Resource, Description) :-
  % already at least as specific as Description
  owl_specialization_of(Resource, Description), !.
owl_specializable(Resource, Description) :-
  owl_description(Description, Description_pl),
  owl_specializable_(Resource, Description_pl), !.

owl_specializable_('http://www.w3.org/2002/07/owl#Thing', _) :- !.

owl_specializable_(Resource, class(Cls)) :-
  % TODO: cardinality 0 restrictions always specializable?
  % - inverse property range could restrict the class
  rdfs_individual_of(Resource, owl:'Restriction'), !,
  % infer restricted class and inverse of restricted property
  owl_restriction(Resource, restriction(P, Facet)),
  once(( Facet=all_values_from(Restr_cls) ;
         Facet=some_values_from(Restr_cls) ;
       ( Facet=cardinality(Min,_,Restr_cls), Min > 0 ) )),
  owl_inverse_property(P, P_inv),
  % only specializable if the restricted class is
  % in fact a valid value for property P of Cls instances
  forall( owl_property_range_on_class(Cls, P, Cls_P_range),
          owl_specializable(Cls_P_range, Restr_cls) ),
  % trick: infer range of inverse P on restricted class Cls_inferred, which is a type of Resource.
  %        Cls_inferred must be specializable to Cls.
  forall( owl_property_range_on_class(Restr_cls, P_inv, Cls_inferred),
          owl_specializable(Cls_inferred, Cls) ).
owl_specializable_(Resource, class(Cls)) :-
  rdfs_individual_of(Resource, owl:'Class'), !,
  owl_specializable_class(Resource, class(Cls)).
owl_specializable_(Resource, class(Cls)) :-
  % specializable if one of the most specific types of resource is a generalization of Cls
  rdfs_type_of(Resource, Resource_type),
  owl_subclass_of(Cls, Resource_type), !.

owl_specializable_(Resource, restriction(P,Facet)) :-
  rdfs_individual_of(Resource, owl:'Restriction'), !,
  owl_description(Resource, Restr),
  owl_specializable_restriction_(Restr, restriction(P,Facet)).

owl_specializable_(Resource, restriction(P,all_values_from(Cls))) :-
  % specializable if all values of P are specializable to Cls
  owl_description(Cls, Cls_descr),
  forall( owl_has(Resource, P, O), owl_specializable_(O, Cls_descr) ).

owl_specializable_(Resource, restriction(P,some_values_from(Cls))) :-
  % specializable if one of the existing values can be specialized to Cls
  \+ rdfs_individual_of(Resource, owl:'Class'),
  owl_description(Cls, Cls_descr),
  owl_has(Resource, P, O),
  owl_specializable_(O, Cls_descr), !.
owl_specializable_(Resource, restriction(P,some_values_from(Cls))) :-
  % specializable if it is consistent to add a new value of type Cls for P
  owl_decomposable_on_resource(Resource, P, Cls),
  forall( owl_property_range_on_resource(Resource, P, Range),
          owl_specializable(Cls, Range) ),
  % and if Resource is a consistent value for inverse_of(P) on instances of Cls
  owl_inverse_property(P,P_inv),
  forall( owl_property_range_on_class(Cls, P_inv, Range_inv),
          owl_specializable(Resource, Range_inv) ).

owl_specializable_(Resource, restriction(P,cardinality(Min,Max,Cls))) :-
  % specializable if cardinality is ok already
  resource_cardinality(Resource, P, Cls, Count_Cls),
  Count_Cls =< Max, Count_Cls >= Min, !.
owl_specializable_(Resource, restriction(P,cardinality(Min,_,Cls))) :-
  % specializable if we can add `Count_decompose` instances of Cls to satisfy min cardinality ...
  resource_cardinality(Resource, P, Cls, Count_Cls), Count_Cls < Min,
  Count_decompose is Min - Count_Cls,
  owl_decomposable_on_resource(Resource, P, Cls, Count_decompose),
  forall( owl_property_range_on_resource(Resource, P, Range),
          owl_specializable(Cls, Range) ),
  % and if Resource is a consistent value for inverse_of(P) on instances of Cls
  owl_inverse_property(P,P_inv),
  forall( owl_property_range_on_class(Cls, P_inv, Range_inv),
          owl_specializable(Resource, Range_inv) ).

owl_specializable_(Resource, restriction(P,has_value(O))) :-
  % specializable if P already has this value
  owl_has(Resource,P,O), !.
owl_specializable_(Resource, restriction(P,has_value(O))) :-
  once((
    rdfs_type_of(O, Cls),
    owl_decomposable_on_resource(Resource, P, Cls)
  )),
  % specializable if we can add O as new value ...
  forall( owl_property_range_on_resource(Resource, P, Range),
          owl_specializable(O, Range) ),
  % and if Resource is a consistent value for inverse_of(P) on instance O
  owl_inverse_property(P,P_inv),
  forall( owl_property_range_on_subject(O, P_inv, Range_inv),
          owl_specializable(Resource, Range_inv) ).

owl_specializable_(Resource, intersection_of(List)) :-
  % specializable if resource is specializable to all classes of the intersection
  forall( member(Cls,List), owl_specializable(Resource, Cls) ).
owl_specializable_(Resource, union_of(List)) :-
  % specializable if resource is specializable to at least one of the classes of the union
  member(Cls,List), owl_specializable(Resource, Cls), !.

owl_specializable_(Resource, complement_of(Cls)) :-
  rdfs_individual_of(Resource, owl:'Class'),!,
  \+ owl_subclass_of(Resource, Cls).
owl_specializable_(Resource, complement_of(Cls)) :-
  \+ owl_individual_of(Resource, Cls).

owl_specializable_class(Intersection, class(Cls_b)) :-
  rdf_has(Intersection, owl:intersectionOf, Set), !,
  rdfs_list_to_prolog_list(Set, Members),
  forall( member(Cls_a,Members), owl_specializable_(Cls_a,class(Cls_b)) ).
owl_specializable_class(Union, class(Cls_b)) :-
  rdf_has(Union, owl:unionOf, Set), !,
  rdfs_list_to_prolog_list(Set, Members),
  member(Cls_a,Members), owl_specializable_(Cls_a,class(Cls_b)), !.
owl_specializable_class(Cls_a, class(Cls_b)) :-
  (  owl_subclass_of(Cls_b, Cls_a) ; (
    % check if there is a subclass of Resource that is also a subclass of Cls
    % FIXME: could be many subclasses! limit search somehow?
    rdfs_subclass_of(Sub, Cls_a),
    rdfs_subclass_of(Sub, Cls_b)
  )), !.

owl_specializable_restriction_(restriction(P1,Facet1), restriction(P2,Facet2)) :-
  rdfs_subproperty_of(P2,P1),
  owl_specializable_restriction_facet_(Facet1,Facet2), !.
owl_specializable_restriction_facet_(some_values_from(Cls1), some_values_from(Cls2)) :-
  owl_specializable(Cls1,Cls2).
owl_specializable_restriction_facet_(all_values_from(Cls1), all_values_from(Cls2)) :-
  owl_specializable(Cls1,Cls2).
owl_specializable_restriction_facet_(cardinality(Min1,Max1,Cls1), cardinality(Min2,Max2,Cls2)) :-
  Min1 =< Min2, Max1 >= Max2,
  owl_specializable(Cls1,Cls2).
owl_specializable_restriction_facet_(has_value(V), has_value(V)).

owl_decomposable_on_resource(Resource, P, Cls) :-
  owl_decomposable_on_resource(Resource, P, Cls, 1).
owl_decomposable_on_resource(Resource, P, Cls, Count_decompose) :-
  owl_cardinality_on_resource(Resource, P, Cls, cardinality(_,Max)), !,
  ( Max = infinite ; (
    resource_cardinality(Resource, P, Cls, Count),
    Max >= Count + Count_decompose
  )).
owl_decomposable_on_resource(_, _, _, _).

resource_cardinality(Resource, _, _, 0) :-
  rdfs_individual_of(Resource, owl:'Class'),!.
resource_cardinality(Resource, P, Cls, Card) :-
  owl_cardinality(Resource, P, Cls, Card).

%% owl_satisfies_restriction_up_to(?Resource, ?Restr, ?UpTo)
% 
% Infers at which points in a restriction description there are
% unsattisfied conditions for the provided resource.
%
% @param Resource OWL resource that does not sattisfy a restriction
% @param Restr    OWL resource of the unsattisfied restriction
% @param UpTo     Prolog term describing how to resolve the inconsistency
% 
% TODO: generate detach items for objects that vialote restrictions and are not specializable.
%       seems that it might be best to exclusively generate detach items with this mechanism.
%       then changing strategy could yield in detach items if wanted.
%
owl_satisfies_restriction_up_to(Resource, Restr, UpTo) :-
  owl_description(Restr, Descr),
  owl_specializable_(Resource,Descr),
  owl_satisfies_restriction_up_to_internal(Resource, Descr, UpTo).

owl_satisfies_restriction_up_to_internal(S, class(Cls), classify(S,Cls)) :-
  \+ owl_individual_of(S,Cls). % Cls is a atomic class

owl_satisfies_restriction_up_to_internal(S, intersection_of(List), UpTo) :-
  % for each class description Cls, check up to where S fullfills the description
  member(Cls,List),
  owl_description(Cls, Cls_descr),
  owl_satisfies_restriction_up_to_internal(S, Cls_descr, UpTo).

owl_satisfies_restriction_up_to_internal(S, union_of(List), UpTo) :-
  % for each class description Cls in List to which S can be specialized, check up to where S fullfills the description
  member(Cls,List),
  owl_description(Cls, Cls_descr),
  owl_specializable_(S, Cls_descr),
  owl_satisfies_restriction_up_to_internal(S, Cls_descr, UpTo).

owl_satisfies_restriction_up_to_internal(S, restriction(P,Facet), UpTo) :-
  % unwrap property chains into nested some_values_from restrictions
  rdf_has(P, owl:propertyChainAxiom, RDFList), !,
  rdfs_list_to_prolog_list(RDFList, PropChain),
  owl_propery_chain_restriction(PropChain, Facet, ChainRestr),
  owl_restriction_assert(ChainRestr, ChainRestrId),
  owl_description(ChainRestrId, ChainRestr_descr),
  owl_satisfies_restriction_up_to_internal(S, ChainRestr_descr, UpTo).

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
     UpTo=detach(S,P,O,1)
  )).

owl_satisfies_restriction_up_to_internal(S, restriction(P,some_values_from(Cls)), UpTo) :-
  owl_description(Cls,Cls_descr),
  ( bagof(O, (owl_has(S, P, O), owl_specializable_(O, Cls_descr)), Os)
  -> (  % there are some values specializable to Cls, thus fullfilled up to were O \in Os violates Cls
    member(O,Os),
    owl_satisfies_restriction_up_to_internal(O, Cls_descr, UpTo)
  ) ; ( % no value of P is specializable to Cls, thus fullfilled up to (S P O)
    UpTo=specify(S,P,Cls,1)
  )).

owl_satisfies_restriction_up_to_internal(S, restriction(P,cardinality(Min,Max,Cls)), UpTo) :-
  owl_cardinality(S, P, Cls, Card),
  ( Card < Min
  -> (  % not enough values of type Cls
    owl_specializable_(S,restriction(P,some_values_from(Cls))),
    % find values of P that can be specialized to Cls
    once((bagof(O, (
      owl_has(S, P, O),
      \+ owl_individual_of(O, Cls),
      owl_specializable(O, Cls)
    ), Os) ; Os=[] )),
    owl_satisfies_min_cardinality_up_to(S, Card, Os,
        restriction(P,cardinality(Min,Max,Cls)), UpTo)
  ) ; (( % to many values of type Cls
    Count is Card - Max, Count > 0,
    UpTo=detach(S,P,Cls,Card)
  ) ; (
    % cardinality is fine, check if restriction fails at a deeper level
    owl_description(Cls, Cls_descr),
    rdf_has(S,P,O),
    owl_individual_of(O,Cls),
    owl_satisfies_restriction_up_to_internal(O,Cls_descr,UpTo)
  ))
  ).

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

owl_propery_chain_restriction(Chain, Facet, Restr) :-
  owl_propery_chain_restriction_(Chain, Facet, some_values_from(Restr)).
owl_propery_chain_restriction_([], Facet, Facet) :- !.
owl_propery_chain_restriction_([P|Rest], Facet, Restr) :-
  rdf_has(P, owl:inverseOf, P_inv), !,
  ( rdf_has(P_inv, owl:propertyChainAxiom, RDFList)
  -> (
    rdfs_list_to_prolog_list(RDFList, PropChain),
    owl_inverse_property_chain(PropChain, PropChain_inv),
    append(PropChain_inv,Rest,Expanded),
    owl_propery_chain_restriction_(Expanded, Facet, Restr)
  ) ; (
    owl_propery_chain_restriction_(Rest, Facet, Sub),
    Restr=some_values_from(restriction(P,Sub))
  )).
owl_propery_chain_restriction_([P|Rest], Facet, Restr) :-
  rdf_has(P, owl:propertyChainAxiom, RDFList), !,
  rdfs_list_to_prolog_list(RDFList, PropChain),
  append(PropChain,Rest,Expanded),
  owl_propery_chain_restriction_(Expanded, Facet, Restr).
owl_propery_chain_restriction_([P|Rest], Facet, some_values_from(restriction(P,Sub))) :-
  owl_propery_chain_restriction_(Rest, Facet, Sub).

