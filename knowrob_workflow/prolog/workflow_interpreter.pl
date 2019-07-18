 /** 
  Copyright (C) 2019 Mihai Pomarlan
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

  @author Mihai Pomarlan
  @license BSD
*/
:- module(workflow_interpreter,
    [
        execute_task/3,
        execute_atomic_task/3
    ]).

:- multifile execute_blackbox_task/3.
:- meta_predicate execute_blackbox_task(0, ?, ?, ?).


:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/owl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease, 'http://ease-crc.org/ont/EASE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_wf, 'http://ease-crc.org/ont/EASE-WF.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

:-  rdf_meta
  execute_task(r, r, ?),
  execute_blackbox_task(r, r, ?),
  execute_atomic_task(r, r, ?).

% exit_status_too_many_workflows(Task, InputParamFn, OutputParamFn) :-
%   =(OutputParamFn, _{'http://ease-crc.org/ont/EASE.owl#ActionStatus':}).

exit_status_not_a_robot_task(_Task, _InputParamFn, OutputParamFn) :-
  =(OutputParamFn, _{'http://ease-crc.org/ont/EASE.owl#ActionStatus':'http://ease-crc.org/ont/EASE-WF.owl#FailWFUninterpretableTask'}).

exit_status_no_triggerable_transition(_Invocation, _Task, _Workflow, OutputParamFn) :-
  =(OutputParamFn, _{'http://ease-crc.org/ont/EASE.owl#ActionStatus':'http://ease-crc.org/ont/EASE-WF.owl#FailWFNoContinuation'}).

exit_status_many_triggerable_transitions(_Invocation, _Task, _Workflow, OutputParamFn) :-
  =(OutputParamFn, _{'http://ease-crc.org/ont/EASE.owl#ActionStatus':'http://ease-crc.org/ont/EASE-WF.owl#FailWFNondeterministicContinuation'}).

execute_blackbox_task(_Task, _InputParamFn, _OutputParamFn) :-
  % Unless someone else actually defines code to execute for various tasks, this is a failure.
  fail.

execute_atomic_task(Task, InputParamFn, OutputParamFn) :-
  owl_individual_of(Task, ease_wf:'OGPBlackBoxTask'),
  execute_blackbox_task(Task, InputParamFn, OutputParamFn),
  %% for the moment, will not backtrack because the real world needs special handling to undo actions
  !.

execute_atomic_task(Task, InputParamFn, OutputParamFn) :-
  \+ owl_individual_of(Task, ease_wf:'OGPBlackBoxTask'),
  execute_task(Task, InputParamFn, _, OutputParamFns),
  %% for the moment, will not backtrack because the real world needs special handling to undo actions
  =(OutputParamFns, [OutputParamFn|_]),!.

get_param_fn(_, [], ParamFn, ParamFn).
get_param_fn(VariableParamFn, [[Param, Variable, ':']|Bindings], BaseParamFn, ParamFn) :-
    get_dict(Variable, VariableParamFn, Value),
    put_dict(Param, BaseParamFn, Value, CrParamFn),
    get_param_fn(VariableParamFn, Bindings, CrParamFn, ParamFn).
get_param_fn(VariableParamFn, [[Param, Value, '=']|Bindings], BaseParamFn, ParamFn) :-
    put_dict(Param, BaseParamFn, Value, CrParamFn),
    get_param_fn(VariableParamFn, Bindings, CrParamFn, ParamFn).

update_variable_param_fn(VariableParamFn, [], _, VariableParamFn).
update_variable_param_fn(VariableParamFn, [[Variable, Output, ':']|Bindings], OutputParamFn, NewVariableParamFn) :-
    get_dict(Output, OutputParamFn, Value),
    put_dict(Variable, VariableParamFn, Value, CrVariableParamFn),
    update_variable_param_fn(CrVariableParamFn, Bindings, OutputParamFn, NewVariableParamFn).
update_variable_param_fn(VariableParamFn, [[Variable, Value, '=']|Bindings], OutputParamFn, NewVariableParamFn) :-
    put_dict(Variable, VariableParamFn, Value, CrVariableParamFn),
    update_variable_param_fn(CrVariableParamFn, Bindings, OutputParamFn, NewVariableParamFn).

get_workflow(Task, Workflow) :-
  owl_has(Workflow, ease_wf:'isWorkflowFor', Task),
  owl_individual_of(Workflow, ease_wf:'OGPWorkflow').

get_factual_binding_R(Context, TypeRestriction, TargetRoles, Bdg) :-
  owl_has(Context, ease_wf:'hasBinding', B),
  owl_individual_of(B, ease_wf:'FactualBinding'),
  owl_individual_of(B, TypeRestriction),
  owl_has(B, ease_wf:'hasBindingRole', R),
  owl_has(B, ease_wf:'hasBindingFiller', F),
  member(R, TargetRoles),
  (owl_individual_of(B, ease_wf:'RoleRoleBinding') ->
    =(Bdg, [R, F, ':']) ;
    =(Bdg, [R, F, '='])).

get_factual_binding_F(Context, TypeRestriction, TargetFillers, Bdg) :-
  owl_has(Context, ease_wf:'hasBinding', B),
  owl_individual_of(B, ease_wf:'FactualBinding'),
  owl_individual_of(B, TypeRestriction),
  owl_has(B, ease_wf:'hasBindingFiller', F),
  owl_has(B, ease_wf:'hasBindingRole', R),
  member(F, TargetFillers),
  (owl_individual_of(B, ease_wf:'RoleRoleBinding') ->
    =(Bdg, [R, F, ':']) ;
    =(Bdg, [R, F, '='])).

get_succedence(Workflow, Predecessor, CS) :-
  owl_has(Workflow, ease_wf:'hasSuccedence', CS),
  owl_individual_of(CS, ease_wf:'ConditionalSuccedence'),
  owl_has(CS, ease_wf:'hasPredecessor', Predecessor).

get_counterfactual_binding(CS, B) :-
  owl_has(CS, ease_wf:'hasBinding', B),
  owl_individual_of(B, ease_wf:'CounterfactualBinding').

condition_passes(Condition, VariableParamFn) :-
  owl_individual_of(Condition, ease_wf:'RoleFillerBinding'),
  owl_has(Condition, ease_wf:'hasBindingRole', Role),
  owl_has(Condition, ease_wf:'hasBindingFiller', FillerPattern),
  get_dict(Role, VariableParamFn, FillerPattern),!.

condition_passes(Condition, VariableParamFn) :-
  owl_individual_of(Condition, ease_wf:'RoleRoleBinding'),
  owl_has(Condition, ease_wf:'hasBindingRole', Role),
  owl_has(Condition, ease_wf:'hasBindingFiller', TargetRole),
  get_dict(Role, VariableParamFn, FillerPattern),
  get_dict(TargetRole, VariableParamFn, FillerPattern),!.

conditions_pass([], _) :- !.
conditions_pass([Condition|Conditions], VariableParamFn) :-
  condition_passes(Condition, VariableParamFn),
  conditions_pass(Conditions, VariableParamFn),!.

is_triggerable(CondSucc, VariableParamFn) :-
  findall(B, get_counterfactual_binding(CondSucc, B), ConditionList), list_to_set(ConditionList, Conditions),
  conditions_pass(Conditions, VariableParamFn),!.

get_triggerable_condsucc([], _, []) :- !.
get_triggerable_condsucc([CondSucc|CondSuccs], VariableParamFn, TriggerableCondSuccs) :-
  \+ is_triggerable(CondSucc, VariableParamFn),
  get_triggerable_condsucc(CondSuccs, VariableParamFn, TriggerableCondSuccs),!.
get_triggerable_condsucc([CondSucc|CondSuccs], VariableParamFn, TriggerableCondSuccs) :-
  is_triggerable(CondSucc, VariableParamFn),
  get_triggerable_condsucc(CondSuccs, VariableParamFn, TCSs),
  =(TriggerableCondSuccs, [CondSucc|TCSs]),!.

select_workflow(Task, UniqueWorkflows, InputParamFn, OutputParamFn) :-
  %% Have no workflow for the task, hence it is atomic. Execute it as such.
  length(UniqueWorkflows, 0),
  execute_atomic_task(Task, InputParamFn, OutputParamFn).

select_workflow(_Task, UniqueWorkflows, InputParamFn, OutputParamFn) :-
  %% Have exactly 1 workflow for the task, then begin stepping through it.
  length(UniqueWorkflows, 1),
  member(MW, UniqueWorkflows),
  enter_workflow(MW, InputParamFn, OutputParamFn).

select_workflow(_Task, UniqueWorkflows, InputParamFn, OutputParamFn) :-
  length(UniqueWorkflows, X),
  \+ member(X, [0, 1]),
  max_priority_workflow(UniqueWorkflows, 0, 0, MW),
  enter_workflow(MW, InputParamFn, OutputParamFn).

max_priority_workflow([], _, MWF, MWF).
max_priority_workflow([WF|Workflows], CMaxPrio, CMaxWF, MWF) :- 
  owl_has(WF, ease_wf:'hasPriority', literal(type(_, CPrio))),
  ((CMaxPrio < CPrio) -> 
    max_priority_workflow(Workflows, CPrio, WF, MWF) ;
    max_priority_workflow(Workflows, CMaxPrio, CMaxWF, MWF)).

enter_workflow(MW, InputParamFn, OutputParamFn) :-
  owl_has(MW, dul:'defines', Invocation),
  owl_individual_of(Invocation, ease_wf:'TaskInvocation'),
  owl_has(Invocation, dul:'definesTask', EntryTask),
  owl_individual_of(EntryTask, ease_wf:'EntryOGPWorkflow'),!,
  step_workflow_task(Invocation, EntryTask, MW, InputParamFn, _{}, OutputParamFn).

execute_task(Task, InputParamFn, OutputParamFn) :-
  % Do not attempt to execute a non-OGPTask, these are not meant for the interpreter
  \+ owl_individual_of(Task, ease_wf:'OGPTask'),
  exit_status_not_a_robot_task(Task, InputParamFn, OutputParamFn).

execute_task(Task, InputParamFn, InputParamFn) :-
  owl_individual_of(Task, ease_wf:'EntryOGPWorkflow').

execute_task(Task, InputParamFn, InputParamFn) :-
  owl_individual_of(Task, ease_wf:'ExitOGPWorkflow').

execute_task(Task, InputParamFn, OutputParamFn) :-
  \+ owl_individual_of(Task, ease_wf:'EntryOGPWorkflow'),
  \+ owl_individual_of(Task, ease_wf:'ExitOGPWorkflow'),
  owl_individual_of(Task, ease_wf:'OGPTask'),
  findall(Workflow, get_workflow(Task, Workflow), WorkflowList),
  list_to_set(WorkflowList, UW),
  select_workflow(Task, UW, InputParamFn, OutputParamFn).

step_workflow_task(Invocation, Task, Workflow, InputParamFn, VariableParamFn, OutputParamFn) :-
  %%% execute task Task to construct a list OutputParamFn
  execute_task(Task, InputParamFn, TaskOutputParamFn),
  step_workflow_posttask(Invocation, Task, Workflow, VariableParamFn, TaskOutputParamFn, OutputParamFn),!.

step_workflow_posttask(_Invocation, Task, _, _, OutputParamFn, OutputParamFn) :-
  %%% if Task was of type ExitOGPWorkflow, return its OutputParamFn
  owl_individual_of(Task, ease_wf:'ExitOGPWorkflow'),!.

step_workflow_posttask(Invocation, Task, Workflow, VariableParamFn, TaskOutputParamFn, OutputParamFn) :-
  \+ owl_individual_of(Task, ease_wf:'ExitOGPWorkflow'),
  %%% get all O so that Task definesOutput O
  findall(O, owl_has(Task, 'http://www.ease-crc.org/ont/EASE-WF.owl#definesOutput', O), TaskOutputList), list_to_set(TaskOutputList, TaskOutputs),
  %%% get all factual RoleRole bindings B so that (Invocation hasBinding B) and (B hasFiller O); denote B hasRole R
  findall(B, get_factual_binding_F(Invocation, 'http://www.ease-crc.org/ont/EASE-WF.owl#RoleRoleBinding', TaskOutputs, B), BindingsList), list_to_set(BindingsList, Bindings),
  %%% update: for every binding (R: O), VariableParamFn(R) = TaskOutputParamFn(O)
  update_variable_param_fn(VariableParamFn, Bindings, TaskOutputParamFn, NewVariableParamFn),
  %%% find all CS such that (CS is ConditionalSuccedence) and (Workflow hasSuccedence CS) and (CS hasPredecessor Invocation); denote B one counterfactual binding such that CS hasBinding B
  findall(CS, get_succedence(Workflow, Invocation, CS), CSList), list_to_set(CSList, CondSuccs),
  %%% select the unique CS so that each of its bindings B holds in the NewVariableParamFn
  get_triggerable_condsucc(CondSuccs, NewVariableParamFn, TriggerableCondSuccs),
  length(TriggerableCondSuccs, N),
  (=(N, 0) ->
    (exit_status_no_triggerable_transition(Task, Workflow, OutputParamFn),!);
    (=(N, 1) ->
      (
        %%% find STask so that CS hasSuccessor STask
        =(TriggerableCondSuccs, [CondSucc]),
        owl_has(CondSucc, 'http://www.ease-crc.org/ont/EASE-WF.owl#hasSuccessor', Successor),
        owl_has(Successor, dul:'definesTask', STask),
        %%% step_workflow_task by going to STask, carry-over the VariableParamFn
        %%% get all I so that STask definesInput I
        findall(I, owl_has(STask, 'http://www.ease-crc.org/ont/EASE-WF.owl#definesInput', I), TaskInputsList), list_to_set(TaskInputsList, TaskInputs),
        %%% get all factual bindings B so that (Successor hasBinding B) and (B hasRole I); denote B hasFiller F
        findall(B, get_factual_binding_R(Successor, _, TaskInputs, B), InputBindingsList), list_to_set(InputBindingsList, InputBindings),
        %%% construct a list InputParamFn of form (I, F)
        get_param_fn(NewVariableParamFn, InputBindings, _{}, InputParamFn),
        step_workflow_task(Successor, STask, Workflow, InputParamFn, NewVariableParamFn, OutputParamFn),!
      );
      (exit_status_many_triggerable_transitions(Task, Workflow, OutputParamFn),!))),!.

