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

:- module(ogp,
    [
      ogp_characterize_entity/3,
      ogp_run/2,
      ogp_quantification_criteria/2,
      ogp_decisions_assert/1,
      ogp_decision_assert/1,
      ogp_hasMaterializationStep/1
    ]).
/** <module> Ontology Guided Planning (OGP) procedure.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('ogp_agenda')).
:- use_module(library('ogp_task')).

% TODO handle this
ogp_hasMaterializationStep(_OGP) :- fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % OGP procedure
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_characterize_entity(r,r,t),
            ogp_run(t,t).

%% ogp_characterize_entity(+OGP,+Entity,-Decisions) is nondet.
%
% @param OGP::iri OGP method individual.
% @param Entity::iri The entity to characterize.
% @param Decisions::list A list of triples that characterize the entity.
%
ogp_characterize_entity(OGP,Entity,Decisions_in->Decisions_out) :-
  ogp_agenda_create(OGP,Entity,A0),
  %% create dicts of externally specified decisions
  findall([TaskDict_in,O0], (
    member([T0,S0,P0,O0],Decisions_in),
    TaskDict_in = _{
        ogp:        OGP,
        type:      T0,
        subject:   S0,
        predicate: P0}
  ), D0),
  %%
  ogp_run(A0->_,D0->Dx),
  %% create list of quadruples representing each a decision
  findall([T,S,P,O], (
    member([TaskDict,O],Dx),
    ogp_task_triple(TaskDict,S,P,_),
    get_dict(type, TaskDict, T)),
    Decisions_out).

%%
ogp_run(A,D) :- ogp_run(A,D,[]).
ogp_run(A0->A0,D0->D0,_) :- ogp_agenda_isEmpty(A0), !.
ogp_run(A0->Ax,D0->Dx,B0) :-
  ogp_agenda_pop(A0->A1,D0,TaskDict),!,
  ogp_task_triple(TaskDict,S,P,D),
  print_message(debug(ogp), format('Task selected: (~w,~w,~w).', [S,P,D])),
  %%%%%%%%%%%%
  %%% Loop detection
  %%%%%%%%%%%%
  ( \+ member(TaskDict,B0) -> true ;
    fail
    %throw(ogp_error(knowrob_planning:'OGP_LOOP',[A0,D0]))
  ),
  %%%%%%%%%%%%
  %%% This step
  %%%%%%%%%%%%
  ((
    ogp_agenda_specialize_task(A1,TaskDict->TaskDict_x),
    ( ogp_run_characterization(A1->A2,TaskDict_x,Selection) -> true ; (
      print_message(informational, format('Task execution failed: (~w,~w).', [S,P])),
      fail)),
    %%
    print_message(informational, format('New characteristic: `~w ~w ~w`.', [S,P,Selection])),
    %%
    D1=[[TaskDict_x,Selection]|D0], B1=[],
    % re-add if cardinality has not reached zero
    ogp_task_decrease_card(TaskDict),
    get_dict(cardinality,TaskDict,Card),
    (  Card>0
    -> ogp_agenda_push(A2->A3,TaskDict)
    ;  A3=A2 )
  );(
    ogp_task_inhibit(TaskDict),
    A3=A0, D1=D0, B1=[TaskDict|B0]
  )),
  %%%%%%%%%%%%
  %%% Next steps
  %%%%%%%%%%%%
  ogp_run(A3->Ax,D1->Dx,B1).
% ogp_agenda_pop fails indicates that only invalid items were left on the agenda
ogp_run((OGP,S,_)->(OGP,S,[]),D0->D0,_).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % OGP task ordering
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

:- rdf_meta ogp_quantification_criteria(r,t).

%%
ogp_quantification_criteria(OGP, Criteria) :-
  rdf_has(OGP,knowrob_planning:hasPrioritizedOrdering,Collection),
  findall(C, rdf_has(Collection, dul:hasMember, C), Cs),
  predsort(compare_quantification_criteria, Cs, Criteria).

compare_quantification_criteria(Delta, C1, C2) :-
  rdf_has_prolog(C1,dul:hasDataValue,V1),
  rdf_has_prolog(C2,dul:hasDataValue,V2),
  ( V1 =< V2 -> Delta='>' ; Delta='<' ). % high priority first

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

ogp_run_characterization(Agenda, TaskDict, Selection) :-
  %% has-value restriction
  ogp_task_domain(TaskDict,Selection),
  rdfs_individual_of(Selection, owl:'NamedIndividual'), !,
  ogp_agenda_update(Agenda, TaskDict, Selection).

ogp_run_characterization(Agenda, TaskDict, Selection) :-
  ogp_task_initialize(TaskDict,InputDict,IndividualTask),
  ogp_run_characterization_WF(TaskDict,IndividualTask,InputDict,OutputDict),
  ogp_task_selection(IndividualTask,OutputDict,Selection),
  ogp_agenda_update(Agenda, TaskDict, Selection).

%%
ogp_run_characterization_WF(TaskDict,_IndividualTask,_InputDict,OutputDict) :-
  % TODO: Execute characterization workflows, if available
  ogp_run_characterization_FIXED(TaskDict,OutputDict).

%%
ogp_run_characterization_FIXED(TaskDict,OutputDict) :-
  ogp_task_integration(TaskDict),!,
  ogp_task_triple(TaskDict,_,P,Domain),
  ( rdfs_individual_of(Domain, owl:'Class') ->
    owl_individual_of(O,Domain) ;
    O = Domain ),
  % enforce unique values for inverse functional properties
  ( owl_inverse_functional(P) -> \+ rdf_has(_, P, O) ; true ),
  rdf_equal(Key,knowrob_planning:'Integration_OGP_Outcome'),
  dict_pairs(OutputDict,_,[Key-O]).

ogp_run_characterization_FIXED(TaskDict,OutputDict) :-
  ogp_task_classification(TaskDict),!,
  ogp_task_triple(TaskDict,_,_,Domain),
  rdf_equal(Key,knowrob_planning:'Classification_OGP_Outcome'),
  dict_pairs(OutputDict,_,[Key-Domain]).

ogp_run_characterization_FIXED(TaskDict,OutputDict) :-
  ogp_task_nullification(TaskDict),!,
  ogp_task_triple(TaskDict,S,P,Domain),
  ( rdfs_individual_of(Domain, owl:'Class') ->
  ( rdf_has(S,P,O), owl_individual_of(O,Domain) ) ;
  ( rdf_has(S,P,Domain), O = Domain )),
  rdf_equal(Key,knowrob_planning:'Nullification_OGP_Outcome'),
  dict_pairs(OutputDict,_,[Key-O]).

ogp_run_characterization_FIXED(TaskDict,OutputDict) :-
  ogp_task_quantification(TaskDict),!,
  %% domain must already be a region
  ogp_task_triple(TaskDict,_,_,Region),
  rdfs_individual_of(Region,dul:'Region'),
  %%
  rdf_equal(Key,knowrob_planning:'Quantification_OGP_Outcome'),
  dict_pairs(OutputDict,_,[Key-Region]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%
ogp_decisions_assert([]) :- !.
ogp_decisions_assert([D|Ds]) :-
  ogp_decision_assert(D),
  ogp_decisions_assert(Ds).

%%
ogp_decision_assert([detach,S,P,Entity]) :-
  rdf_retractall(S,P,Entity).

ogp_decision_assert([integrate,S,P,Entity]) :-
  rdf_assert(S,P,Entity).

ogp_decision_assert([quantify,S,P,DataValue]) :-
  kb_assert(S,P,DataValue).

ogp_decision_assert([classify,S,_,Class]) :-
  rdf_assert(S,rdf:type,Class).
