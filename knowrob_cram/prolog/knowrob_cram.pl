/**  <module> knowrob_cram

  Copyright (C) 2013-15 Moritz Tenorth, Asil Kaan Bozcuoglu, Daniel Beßler
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

  @author Moritz Tenorth, Asil Kaan Bozcuoglu, Daniel Beßler
  @license BSD
*/

:- module(knowrob_cram,
    [
      cram_start_action/6,
      cram_start_action/5,
      cram_finish_action/2,
      cram_set_subaction/2,
      cram_add_image_to_event/2,
      cram_add_failure_to_action/5,
      cram_create_desig/2,
      cram_equate_designators/3,
      cram_add_desig_to_action/2,
      cram_add_desig_to_action/3,
      cram_set_object_acted_on/2,
      cram_set_detected_object/3,
      cram_set_perception_request/2,
      cram_set_perception_result/2,
      cram_logged_query/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

:-  rdf_meta
    cram_start_action(r, +, +, r, r, r),
    cram_start_action(r, +, +, r, r),
    cram_finish_action(r, +),
    cram_set_subaction(r, r),
    cram_add_image_to_event(r, r),
    cram_add_failure_to_action(r, r, +, +, r),
    cram_create_desig(r, r),
    cram_equate_designators(r, r, +),
    cram_add_desig_to_action(r, r),
    cram_add_desig_to_action(r, r, r),
    cram_set_object_acted_on(r, r),
    cram_set_detected_object(r, r, r),
    cram_set_perception_request(r, r),
    cram_set_perception_result(r, r),
    cram_logged_query(r, +, r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


%% cram_start_action(+Type, +TaskContext, +StartTime, ?PrevAction, -ActionInst) is det.
%
% Create an action instance, set properties like start time and task context.
% Returns identifier of the generated instance.
%
% @param Type         OWL action class for the action to be created
% @param TaskContext  String describing the task context
% @param StartTime    POSIX timestamp (number with seconds sine 1970)
% @param PrevAction   Instance of the previous action in the task (possibly unbound)
% @param ParentTask   Instance of the parent-task in the task tree hierarchy (possibly unbound)
% @param ActionInst   Returned reference to the created action instance
%
cram_start_action(Type, TaskContext, StartTime, PrevAction, ParentTask, ActionInst) :-

  cram_start_action(Type, TaskContext, StartTime, PrevAction, ActionInst) ,

  %subtask information is asserted
  (nonvar(ParentTask) -> (
      cram_set_subaction(ParentTask, ActionInst)) ; (true)).

cram_start_action(Type, TaskContext, StartTime, PrevAction, ActionInst) :-

  % create action instance
  rdf_instance_from_class(Type, 'LoggingGraph', ActionInst),
  rdf_assert(ActionInst, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  
  % set task context property
  rdf_assert(ActionInst, knowrob:taskContext, literal(type(xsd:string, TaskContext)), 'LoggingGraph'),


  % create timepoint instance and set as start time
  create_timepoint(StartTime, StTime),
  rdf_assert(StTime, rdf:type, knowrob:'TimePoint', 'LoggingGraph'),
  rdf_assert(StTime, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(ActionInst, knowrob:startTime, StTime, 'LoggingGraph'),

  %previous action information is asserted
  (nonvar(PrevAction) -> (
      rdf_assert(ActionInst, knowrob:previousEvent, PrevAction, 'LoggingGraph'),
      rdf_assert(PrevAction, knowrob:nextEvent, ActionInst, 'LoggingGraph')) ; (true)).

%% cram_finish_action(+ActionInst, +EndTime) is det.
%
% Finish an action, i.e. set the end time.
%
% @param ActionInst Action instance to which the information is to be added
% @param EndTime    POSIX timestamp (number with seconds sine 1970)
%
cram_finish_action(ActionInst, EndTime) :-

  % create timepoint instance and set as end time
  create_timepoint(EndTime, ETime),
  rdf_assert(ETime, rdf:type, knowrob:'TimePoint', 'LoggingGraph'),
  rdf_assert(ETime, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(ActionInst, knowrob:endTime, ETime, 'LoggingGraph').

%% cram_logged_query(+QueryingInst, +Query, -BindingInst) is det.
%
% Call the predicate Query and set the query-binding relation
% between QueryingInst and BindingInst based on variable bindings
% of Query term.
%
% @param QueryingInst The event that caused the query
% @param Query The query
% @param BindingInst Variable bindings
%
cram_logged_query(QueryingInst, Query, BindingInst) :-
  % Convert to atom (note: this replaces var names with generated names)
  term_to_atom(Query, QueryAtom),
  rdf_assert(QueryingInst, knowrob:'queryText', QueryAtom, 'LoggingGraph'),
  % VarValues contains list of assigned values after the call
  term_variables(Query, VarValues),
  % Read atom back to term in order to find out the generated variable names
  atom_to_term(QueryAtom, _, BindingNames),
  findall(VarName, member((VarName = _), BindingNames), VarNames),
  % zip [name,value] pairs
  zip(VarNames, VarValues, Bindings),
  % run the query
  call(Query),
  % log the query
  rdf_instance_from_class(knowrob:'QueryBinding', 'LoggingGraph', BindingInst),
  rdf_assert(BindingInst, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(QueryingInst, knowrob:'queryBinding', BindingInst, 'LoggingGraph'),
  forall( member([Name,Value], Bindings), (
    rdf_instance_from_class(knowrob:'VariableBinding', 'LoggingGraph', VariableBindingInst),
    rdf_assert(VariableBindingInst, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
    rdf_assert(VariableBindingInst, knowrob:'nameString', Name, 'LoggingGraph'),
    rdf_assert(VariableBindingInst, knowrob:'variableValue', Value, 'LoggingGraph'),
    rdf_assert(BindingInst, knowrob:'variableBinding', VariableBindingInst, 'LoggingGraph')
  )).

%% cram_set_subaction(+Super, +Sub) is det.
%
% Set the sub-action relation between Super and Sub
%
% @param Super  Upper action instance
% @param Sub    Subaction instance
%
cram_set_subaction(Super, Sub) :-
  rdf_assert(Super, knowrob:subAction, Sub, 'LoggingGraph').



%% cram_add_image_to_event(+Event, +ImageURL) is det.
%
% Link an image to an action/event instance
%
% @param ActionInst Event/action instance to which the information is to be added
% @param ImageURL   String with URL of the image (e.g. file://, package://, http://)
%
cram_add_image_to_event(Event, ImageURL) :-
  rdf_assert(Event, knowrob:linkToImageFile, literal(type(xsd:string, ImageURL)), 'LoggingGraph').





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Failure handling
%


%% cram_add_failure_to_action(+ActionInst, +FailureType, +FailureLabel, +FailureTime, -FailureInst) is det.
%
% Create a new failure instance of type Type and link it to the given action instance.
%
% @param ActionInst    Action instance to which the information is to be added
% @param FailureType   OWL class of the object to be created
% @param FailureLabel  Label to be set for the failure
% @param FailureTime   POSIX timestamp (number with seconds sine 1970)
% @param FailureInst   Returned reference to the created failure instance
%
cram_add_failure_to_action(ActionInst, FailureType, FailureLabel, FailureTime, FailureInst) :-

  rdf_instance_from_class(FailureType, 'LoggingGraph', FailureInst),
  rdf_assert(FailureInst, rdfs:label, literal(type(xsd:string, FailureLabel)), 'LoggingGraph'),

  create_timepoint(FailureTime, StTime),
  rdf_assert(StTime, rdf:type, knowrob:'TimePoint', 'LoggingGraph'),
  rdf_assert(StTime, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(FailureInst, knowrob:startTime, StTime, 'LoggingGraph'),

  rdf_assert(ActionInst, knowrob:eventFailure, FailureInst, 'LoggingGraph').


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Designator handling
%

%% cram_create_desig(+DesigType, -DesigInst) is det.
%
% Create a new designator instance of type DesigType
%
% @param DesigType  OWL class of the designator to be created
% @param DesigInst  Returned reference to the created designator instance
%
cram_create_desig(DesigType, DesigInst) :-
  rdf_assert(DesigInst, rdf:type, DesigType, 'LoggingGraph').


%% cram_equate_designators(+PreDesig, +SuccDesig, +EquationTime) is det.
%
% Set SuccDesig as successorDesignator of PreDesig and assert the EquationTime.
%
% @param PreDesig     Older designator
% @param SuccDesig    New designator to be equated to the old one
% @param EquationTime Time to be stored for the equation event
%
cram_equate_designators(PreDesig, SuccDesig, EquationTime) :-
  create_timepoint(EquationTime, EqTime),
  rdf_assert(EqTime, rdf:type, knowrob:'TimePoint', 'LoggingGraph'),
  rdf_assert(EqTime, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(PreDesig, knowrob:successorDesignator, SuccDesig, 'LoggingGraph'),
  rdf_assert(SuccDesig, knowrob:equationTime, EqTime, 'LoggingGraph').


%% cram_add_desig_to_action(+ActionInst, +DesigInst) is det.
%
% Add designator instance to the given action instance.
%
% @param ActionInst Action instance to which the information is to be added
% @param DesigInst  Designator instance to be added
%
cram_add_desig_to_action(ActionInst, DesigInst) :-
  cram_add_desig_to_action(ActionInst, knowrob:designator, DesigInst, 'LoggingGraph').


%% cram_add_desig_to_action(+ActionInst, +Property, +DesigInst) is det.
%
% Add designator instance to the given action instance using the given
% property.
%
% @param ActionInst Action instance to which the information is to be added
% @param Property   Property identifier to be used for linking the designator (e.g. knowrob:goalPose)
% @param DesigInst  Designator instance to be added
%
cram_add_desig_to_action(ActionInst, Property, DesigInst) :-
  rdf_assert(ActionInst, Property, DesigInst, 'LoggingGraph').


%% cram_set_object_acted_on(ActionInst, ObjectInst) is det.
%
% Set the objectActedOn for an action to the given value.
%
% @param ActionInst Action instance to which the information is to be added
% @param ObjectInst Object instance (designator instance) to be set as objectActedOn
%
cram_set_object_acted_on(ActionInst, ObjectInst) :-
  rdf_assert(ActionInst, knowrob:objectActedOn, ObjectInst, 'LoggingGraph').


%% cram_set_detected_object(ActionInst, ObjectType, ObjectInst) is det.
%
% Create an object of type ObjectType and set it as detectedObject
% of the given action.
%
% @param ActionInst Action instance to which the information is to be added
% @param ObjectType OWL class of the object to be created
% @param ObjectInst Returned reference to the created object instance
%
cram_set_detected_object(ActionInst, ObjectType, ObjectInst) :-
  rdf_instance_from_class(ObjectType, 'LoggingGraph', ObjectInst),
  rdf_assert(ObjectInst, rdf:type, owl:'NamedIndividual', 'LoggingGraph'),
  rdf_assert(ActionInst, knowrob:detectedObject, ObjectInst, 'LoggingGraph').


%% cram_set_perception_request(+ActionInst, +Req) is det.
%
% Set the perception request designator for an action
%
% @param ActionInst Action instance to which the information is to be added
% @param Req        Instance of perception request designator
%
cram_set_perception_request(ActionInst, Req) :-
  rdf_assert(ActionInst, knowrob:perceptionRequest, Req, 'LoggingGraph').


%% cram_set_perception_result(+ActionInst, +Res) is det.
%
% Set the perception result designator for an action
%
% @param ActionInst Action instance to which the information is to be added
% @param Res        Instance of perception result designator
%
cram_set_perception_result(ActionInst, Res) :-
  rdf_assert(ActionInst, knowrob:perceptionResult, Res, 'LoggingGraph').



