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
:- use_module(library('knowrob/ESG')).
:- use_module(library('ogp')).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % assembly procedure
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:-  rdf_meta ogp_execute_assembly(r,r,r).

ogp_execute_assembly(OGP,Goal,Entity) :-
  ogp_assemblage_create(Goal,Entity),
  subassemblage_queue(Entity,AssemblageSequence,Queue),
  ogp_execute_assembly_(OGP,AssemblageSequence,Queue).

ogp_execute_assembly_(_, _, Q) :-
  subassemblage_queue_empty(Q),!.

ogp_execute_assembly_(OGP, AssemblageSequence, Q1) :-
  subassemblage_queue_pop(Q1, Assemblage, Q2),
  %%
  print_message(informational, format('Next assemblage: ~w.', [Assemblage])),
  %%
  %get_dict(AssemblageConcept, AssemblageDict, Assemblage),
  %%
  ogp_assemblage_proceed(OGP,Assemblage,_),
  !, % it is not allowed to go back, once proceeded
  forall(
    % attach Assemblage to its parent.
    ogp_assemblage_parent(Assemblage,AssemblageSequence,Parent),
    ogp_assemblage_link(Parent,Assemblage)
  ),
  %%
  ogp_execute_assembly_(OGP, AssemblageSequence, Q2).

%%
ogp_assemblage_proceed(OGP,Assemblage,Decisions) :-
  ogp_characterize_entity(OGP,Assemblage,Decisions),
  ogp_assemblage_materialization(OGP,Assemblage,Decisions),
  !, % no backtracking allowed after materialization succeeded
  ogp_decisions_assert(Decisions).

%%
ogp_assemblage_materialization(OGP,_,_) :-
  \+ ogp_hasMaterializationStep(OGP), !.
ogp_assemblage_materialization(OGP,_Assemblage,Decisions) :-
  %assembly_primary_part(Assemblage, PrimaryPart, SecondaryParts),
  writeln('not implemented ogp_assemblage_materialization'), % TODO
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % sub-assemblage queue
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:-  rdf_meta
      subassemblage_queue(r,+),
      subassemblage_queue_pop(+,r,-),
      subassemblage_queue_peak(+,r).

%% subassemblage_queue
subassemblage_queue(Entity,Queue) :-
  subassemblage_queue(Entity,_,Queue).

subassemblage_queue(Goal,AssemblageSequence,Queue) :-
  rdfs_individual_of(Goal,owl:'Class'),!,
  ogp_assemblage_create(Goal,Entity),
  subassemblage_queue(Entity,AssemblageSequence,Queue).

subassemblage_queue(X0,AssemblageSequence,Queue) :-
  subassemblages_create(X0,Subassemblages),
  AssemblageSequence=[X0,Subassemblages],
  subassemblages_list(AssemblageSequence,Assemblages),
  subassemblages_constraints(AssemblageSequence,Constraints),
  esg_assert(Assemblages,Constraints,ESG),
  esg_to_list(ESG,Queue),!.

subassemblages_create(Parent,Children) :-
  findall([Child,ChildChildren], (
    assemblage_linksAssemblage_restriction(Parent,ChildConcept),
    ogp_assemblage_create(ChildConcept,Child),
    subassemblages_create(Child,ChildChildren)
  ), Children).

subassemblages_constraints([Parent,Children],Constraints) :-
  findall(<(X,Y), (
    member(Child,Children), (
    ( Child=[X,_], Y=Parent );
    ( subassemblages_constraints(Child,ChildConstraints),
      member(<(X,Y), ChildConstraints) )
    )
  ), Constraints).

subassemblages_list([Parent,Children],Assemblages) :-
  findall(X, (
    ( X = Parent ) ; (
    member(Child,Children), 
    subassemblages_list(Child,Xs),
    member(X,Xs))
  ), Assemblages).

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

ogp_assemblage_child(Parent,AssemblageDict,Child) :-
  assemblage_linksAssemblage_restriction(Parent,ChildConcept),
  get_dict(ChildConcept,AssemblageDict,Child).

ogp_assemblage_parent(X,[Parent,Children],Parent) :-
  once(member([X,_],Children)),!.
ogp_assemblage_parent(X,[_,Children],Parent) :-
  member(Child,Children),
  ogp_assemblage_parent(X,Child,Parent),!.

%ogp_assemblage_parent(Child,AssemblageDict,Parent) :-
  %get_dict(ChildConcept,AssemblageDict,Child),
  %get_dict(_,AssemblageDict,Parent), Parent \= Child,
  %assemblage_linksAssemblage_restriction(Parent,ChildConcept).

ogp_assemblage_link(Parent,Child) :-
  rdf_has(Parent,knowrob_assembly:'usesConnection',ParentConn),
  assemblage_connection_affordance(ParentConn,AffType),
  rdf_has(Child,knowrob_assembly:'usesConnection',ChildConn),
  rdf_has(ChildConn,knowrob_assembly:'needsAffordance',Affordance1),
  %%
  rdf_has(Obj,knowrob:'hasAffordance',Affordance1),
  rdf_has(Obj,knowrob:'hasAffordance',Affordance2),
  Affordance1 \= Affordance2,
  rdfs_individual_of(Affordance2,AffType), !,
  %%
  rdf_assert(ParentConn, knowrob_assembly:'consumesAffordance', Affordance2).
