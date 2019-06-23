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

:- module(ogp_assembly,
    [
      ogp_execute_assembly/3,
      subassemblage_queue/2,
      subassemblage_queue_pop/3,
      subassemblage_queue_peak/2
    ]).
/** <module> OGP assembly method.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/event_graph')).
:- use_module(library('ogp')).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % assembly procedure
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:-  rdf_meta ogp_execute_assembly(r,r,r).

%% ogp_execute_assembly(+OGP,+Goal,-Entity)
%
% Execute an assembly task, following a *OGP* method.
% *Goal* is an assemblage concept that shall be
% materialized. *Entity* is the symbolic
% representation of the manifested assemblage.
%
ogp_execute_assembly(OGP,Goal,Entity) :-
  rdfs_individual_of(Goal,owl:'Class'),!,
  ogp_assemblage_create(Goal,Entity),
  subassemblage_queue(Entity,Assemblages,Constraints,Queue),
  findall(A-A, member(A,Assemblages), Pairs),
  dict_pairs(Dict,_,Pairs),
  ogp_execute_assembly_(OGP,Dict,Constraints,Queue),
  % retract created assemblages that were not used
  % i.e. because some other existing assemblage was used instead
  forall((
    member(A,Assemblages),
    \+ get_dict(A,Dict,A)),
    ogp_assemblage_retract(A)
  ).

ogp_execute_assembly(_OGP,Goal,_Entity) :-
  rdfs_individual_of(Goal,owl:'NamedIndividual'),!,
  % TODO: allow to start with an existing assemblage
  print_message(error, format('ogp_execute_assembly does not accept assemblage individuals.')),
  fail.

ogp_execute_assembly_(_, _, _, Q) :-
  subassemblage_queue_empty(Q),!.

ogp_execute_assembly_(OGP, Assemblages, Constraints, Q1) :-
  %% pop next item
  subassemblage_queue_pop(Q1, A_id, Q2),
  %% try to use an existing assemblage first, else a newly created one,
  %% TODO: but not for the last element in the queue!
  %% FIXME: i think it also needs to be prevented that this one is overwritten then!
  (( %\+ subassemblage_queue_empty(Q2),
     get_dict(A_id,Assemblages,A_id),
     ogp_existing_assemblage(A_id,Assemblages,Constraints,Assemblage),
     b_set_dict(A_id,Assemblages,Assemblage)
  ); Assemblage = A_id ),
  print_message(informational, format('Next assemblage: ~w.', [Assemblage])),
  %% imply the subassembly relation by a list of decisions
  %% that the planner takes into account.
  findall([integrate,S,P,O], (
    member(<(C_id,X_id), Constraints),
    get_dict(C_id, Assemblages, Child),
    get_dict(X_id, Assemblages, Parent),
    ogp_assemblage_link(Parent,Child,[S,P,O])
  ), Decisions),
  %%
  ogp_assemblage_proceed(OGP,Assemblage,Decisions->_),
  !, % it is not allowed to go back, once proceeded
  %% recursion
  ogp_execute_assembly_(OGP, Assemblages, Constraints, Q2).

ogp_existing_assemblage(A_id,Dict,Constraints,Existing) :-
  assemblage_concept(A_id,Concept),
  findall(A, (
    owl_individual_of(A,Concept),
    \+ get_dict(A,Dict,_),
    \+ get_dict(_,Dict,A)
  ), As),
  list_to_set(As,As_set),
  member(Existing,As_set),
  % all children of the existing assemblages must match the ones
  % that were selected in earlier steps
  forall(
    direct_child(Existing,Existing_child), (
    member(<(B_id,A_id),Constraints),
    get_dict(B_id, Dict, Existing_child)
  )),
  % all parents of the existing assemblage must be used in later steps
  forall(
    direct_parent(Existing,Existing_parent),
    once((
      member(<(A_id,P_id),Constraints),
      get_dict(P_id, Dict, P_id),
      assemblage_concept(P_id, P_Concept),
      owl_individual_of(Existing_parent, P_Concept),
      b_set_dict(P_id, Dict, Existing_parent)
    ))
  ).

%% FIXME: what about linkedByAssemblage restrictions?
direct_parent(Existing,Existing_parent) :-
  subassemblage(Existing_parent, Existing).
direct_child(Existing,Existing_child) :-
  subassemblage(Existing, Existing_child).

assemblage_concept(A,Concept) :-
  rdf_has(A,rdf:type,Concept),
  once((owl_subclass_of(Concept,knowrob_assembly:'Assemblage'))), !.

%%
ogp_assemblage_proceed(OGP,Assemblage,D0->Dx) :-
  ogp_characterize_entity(OGP,Assemblage,D0->Dx),
  ogp_assemblage_materialization(OGP,Assemblage,Dx),
  !, % no backtracking allowed after materialization succeeded
  print_message(debug(ogp), format('Asserting assemblage: ~w`.', [Assemblage])),
  ogp_decisions_assert(Dx).

%%
ogp_assemblage_materialization(OGP,_,_) :-
  \+ ogp_hasMaterializationStep(OGP), !.
ogp_assemblage_materialization(_OGP,_Assemblage,_Decisions) :-
  %assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts),
  writeln('not implemented ogp_assemblage_materialization'), % TODO
  %% TODO
  % - get primary / secondary parts 
  % - get grasps
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % sub-assemblage queue
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:-  rdf_meta
      subassemblage_queue(r,+),
      subassemblage_queue_pop(+,r,-),
      subassemblage_queue_peak(+,r).

%% subassemblage_queue(+Goal,-Queue) is det.
%
% @param Goal An assemblage concept or individual.
% @param Queue A partially ordered queue of sub-assemblages.
%
subassemblage_queue(Goal,Queue) :-
  subassemblage_queue(Goal,_,_,Queue).

subassemblage_queue(Goal,Assemblages,Constraints,Queue) :-
  rdfs_individual_of(Goal,owl:'Class'),!,
  ogp_assemblage_create(Goal,Entity),
  subassemblage_queue(Entity,Assemblages,Constraints,Queue).

subassemblage_queue(X0,Assemblages_set,Constraints,Queue) :-
  subassembly_constraints(X0,[],Constraints),
  findall(A, (A=X0 ;
    member(<(A,_),Constraints) ; 
    member(<(_,A),Constraints)),
    Assemblages),
  list_to_set(Assemblages,Assemblages_set),
  esg_assert(Assemblages_set,Constraints,ESG),
  esg_to_list(ESG,Queue),!.

subassemblages_create(Parent,Assemblages,Children) :-
  findall([Child,ChildChildren], (
    assemblage_linksAssemblage_restriction(Parent,ChildConcept),
    once((
    ( member(Child,Assemblages),
      owl_individual_of(Child,ChildConcept),
      ChildChildren=[] );
    ( ogp_assemblage_create(ChildConcept,Child),
      subassemblages_create(Child,Assemblages,ChildChildren) )))
  ), Children).

subassembly_constraints(Root,X,Constraints) :-
  subassemblages_create(Root,X,RootSubassemblages),
  RootSequence=[Root,RootSubassemblages],
  subassemblages_list(RootSequence,Assemblages),
  list_to_set(Assemblages,Assemblages_set),
  %%
  findall(C, (
    subassemblages_constraint(RootSequence,C) ;
    parent_assemblage_constraint(Assemblages_set,X,C)),
    Constraints).

parent_assemblage_constraint([A|_],X,C) :-
  \+ member(A,X),
  assemblage_linkedByAssemblage_restriction(A,ParentConcept),
  ogp_assemblage_create(ParentConcept,Parent),
  subassembly_constraints(Parent,[A],Constraints),
  ( C = <(A,Parent) ; member(C,Constraints) ).
parent_assemblage_constraint([_|As],X,C) :-
  parent_assemblage_constraint(As,X,C).

subassemblages_constraint([Parent,Children],<(X,Y)) :-
  member(Child,Children), (
  ( Child=[X,_], Y=Parent )
  ; subassemblages_constraint(Child,<(X,Y)) ).

subassemblages_list([],[]) :- !.
subassemblages_list([Parent,Children],[Parent|Ys]) :-
  findall(Y, (
    member(Next,Children),
    subassemblages_list(Next, Next_s),
    member(Y,Next_s)),
    Ys).

subassemblage_queue_pop(In,Elem,Out) :-
  esg_pop(In,-Elem,X),
  esg_pop(X,+Elem,Out).

subassemblage_queue_peak(In,Elem) :-
  esg_peak(In,-Elem).

subassemblage_queue_empty([]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % Helper
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

ogp_assemblage_create(Concept,Assemblage) :-
  owl_property_range_on_class(Concept,knowrob_assembly:usesConnection,ConnConcept),!,
  rdf_instance_from_class(Concept,Assemblage),
  rdf_instance_from_class(ConnConcept,Connection),
  rdf_assert(Assemblage,knowrob_assembly:usesConnection,Connection).

ogp_assemblage_retract(Assemblage) :-
  rdf_has(Assemblage,knowrob_assembly:usesConnection,Conn),
  rdf_retractall(Assemblage,_,_),
  rdf_retractall(_,_,Assemblage),
  rdf_retractall(Conn,_,_),
  rdf_retractall(_,_,Conn).

ogp_assemblage_connection(Assemblage,Connection) :-
  rdf_has(Assemblage,knowrob_assembly:'usesConnection',Connection),!.
ogp_assemblage_connection(Assemblage,Connection) :-
  assemblage_concept(Assemblage,Concept),
  owl_property_range_on_class(Concept,knowrob_assembly:usesConnection,ConnConcept),!,
  rdf_instance_from_class(ConnConcept,Connection),
  rdf_assert(Assemblage,knowrob_assembly:usesConnection,Connection).

ogp_assemblage_link(Parent, Child, [ParentConn,P,Affordance2]) :-
  ogp_assemblage_connection(Parent,ParentConn),
  ogp_assemblage_connection(Child,ChildConn),
  assemblage_connection_affordance(ParentConn,AffType),
  rdf_has(ChildConn,knowrob_assembly:'needsAffordance',Affordance1),
  %%
  rdf_has(Obj,knowrob:'hasAffordance',Affordance1),
  rdf_has(Obj,knowrob:'hasAffordance',Affordance2),
  Affordance1 \= Affordance2,
  rdfs_individual_of(Affordance2,AffType), !,
  %%
  rdf_equal(P, knowrob_assembly:'consumesAffordance').
