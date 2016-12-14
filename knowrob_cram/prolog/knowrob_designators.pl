/** <module> knowrob_designators

  Copyright (C) 2013-15 Daniel Beßler, Asil Kaan Bozcuoglu, Moritz Tenorth
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

  @author Daniel Beßler, Asil Kaan Bozcuoglu, Moritz Tenorth
  @license BSD
*/

:- module(knowrob_designators,
    [
        designator_object/2,
        designator_template/3,
        
        designator_assert/3,
        designator_assert/4,
        designator_read_template/3,
        designator_read_template/4,
        designator_assert_dimensions/2,
        designator_assert_dimensions/3,
        designator_assert_color/2,
        designator_assert_color/3,
        designator_add_perception/3,
        designator_add_perception/4,
        
        designator_between/3,
        
        task_designator_exp/2,
        action_designator_exp/2,
        
        designator_publish/1,
        designator_publish/2,
        designator_publish_image/1,
        show_entity/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_objects')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    designator_object(r,r),
    designator_template(r,?,?),
    designator_assert(r,r,r),
    designator_assert(r,r,+,r),
    designator_read_template(r,r,r),
    designator_read_template(r,r,+,r),
    designator_assert_dimensions(r,r),
    designator_assert_dimensions(r,r,+),
    designator_assert_color(r,r),
    designator_assert_color(r,r,+),
    designator_add_perception(r,r,+),
    designator_add_perception(r,r,?,+),
    designator_between(r,r,r),
    task_designator_exp(r,r),
    action_designator_exp(r,r),
    designator_publish(r),
    designator_publish(r,+),
    designator_publish_image(r),
    show_entity(r).


:- assert(log_pbl(fail)).
log_publisher(Pbl) :-
    log_pbl(fail),
    jpl_new('org.knowrob.cram.LogdataPublisher', [], Pbl),
    jpl_list_to_array(['org.knowrob.cram.LogdataPublisher'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Pbl, Arr], _),
    retract(log_pbl(fail)),
    assert(log_pbl(Pbl)),!.
log_publisher(Pbl) :-
    log_pbl(Pbl).

%% designator_object(+Designator,?ObjInstance) is nondet.
%
% Check name of object individual that corresponds to desingator individual.
%
% @param Designator    Identifier of the designator
% @param ObjInstance   Identifier of the object
% 
designator_object(Designator, ObjInstance) :-
  atom(Designator),
  rdf_split_url(_, ObjLocal, Designator),
  atom_concat('http://knowrob.org/kb/cram_log.owl#Object_', ObjLocal, ObjInstance).


%% designator_template(+Experiment, ?Response, ?Template) is nondet.
%
% Check the object templates for perception responses.
%
% @param Experiment    Identifier of the experiment
% @param Response      The response of the perception for this object
% @param Template      Identifier of the template
% 
designator_template(Experiment, Response, Template) :-
  atom(Experiment),
  owl_individual_of(Experiment, knowrob:'RobotExperiment'),
  once(experiment_map(Experiment, Map)),
  designator_template(Map, Response, Template).

%% designator_template(+Map, ?Response, ?Template) is nondet.
%
% Check the object templates for perception responses.
%
% @param Map           Identifier of the semantic map where the template is defined
% @param Response      The response of the perception for this object
% @param Template      Identifier of the template
% 
designator_template(Map, Response, Template) :-
  atom(Map),
  owl_individual_of(Map, knowrob:'SemanticEnvironmentMap'),
  owl_has(Template, knowrob:'perceptionResponse', literal(type(_,Response))),
  atom_concat(_, '_TEMPLATE', Template),
  % Make sure template instance is defined in Map
  rdf_split_url(MapUrl, _, Map),
  rdf_split_url(MapUrl, _, Template).

%% designator_assert(?ObjInstance, +Designator, +Map) is nondet.
%
% Assert symbolic representation of logged designator.
%
% @param ObjInstance   Identifier of the object
% @param Designator    Identifier of the designator
% @param Template      Identifier of the semantic map
% 
designator_assert(ObjInstance, Designator, Map) :-
  once(mng_designator(Designator, DesignatorJava)),
  jpl_is_object(DesignatorJava),
  designator_assert(ObjInstance, Designator, DesignatorJava, Map).

%% designator_assert(?ObjInstance, +Designator, +DesignatorJava, +Map) is nondet.
%
% Assert symbolic representation of logged designator.
%
% @param ObjInstance    Identifier of the object
% @param Designator     Identifier of the designator
% @param DesignatorJava Designator JAVA object
% @param Template       Identifier of the semantic map
% 
designator_assert(ObjInstance, Designator, _, _) :-
  designator_object(Designator, ObjInstance),
  % Check if already asserted
  rdf_has(ObjInstance, rdf:type, _), !.

designator_assert(ObjInstance, Designator, DesignatorJava, Map) :-
  designator_object(Designator, ObjInstance),
  % Assert object type
  rdf_assert(ObjInstance, rdf:type, knowrob:'SpatialThing-Localized'),
  % Assert link to semantic map
  rdf_assert(ObjInstance, knowrob:'describedInMap', Map),
  
  once((designator_read_template(ObjInstance, Designator, DesignatorJava, Map) ; true)),
  once((designator_assert_dimensions(ObjInstance, Designator, DesignatorJava) ; true)),
  once((designator_assert_color(ObjInstance, Designator, DesignatorJava) ; true)).


%% designator_read_template(+ObjInstance, +Designator, +Map) is nondet.
%
% Assert template properties of logged designator.
%
% @param ObjInstance   Identifier of the object
% @param Designator    Identifier of the designator
% @param Template      Identifier of the semantic map
% 
designator_read_template(ObjInstance, Designator, Map) :-
  once(mng_designator(Designator, DesignatorJava)),
  designator_read_template(ObjInstance, Designator, DesignatorJava, Map).

%% designator_read_template(+ObjInstance, +Designator, +DesignatorJava, +Map) is nondet.
%
% Assert template properties of logged designator.
%
% @param ObjInstance    Identifier of the object
% @param Designator     Identifier of the designator
% @param DesignatorJava Designator JAVA object
% @param Template       Identifier of the semantic map
% 
designator_read_template(ObjInstance, Designator, DesignatorJava, Map) :-
  jpl_is_object(DesignatorJava),
  mng_designator_props(Designator, DesignatorJava, 'RESPONSE', Response),
  designator_read_template_from_map(ObjInstance, Map, Response).

designator_read_template(ObjInstance, Designator, DesignatorJava, Map) :-
  jpl_is_object(DesignatorJava),
  mng_designator_props(Designator, DesignatorJava, 'TYPE', Response),
  designator_read_template_from_map(ObjInstance, Map, Response).

designator_read_template(ObjInstance, _, _, Map) :-
  designator_read_template_from_map(ObjInstance, Map, 'SpatialThing-Localized').

designator_read_template_from_map(ObjInstance, Map, Response) :-
  once(designator_template(Map, Response, TemplateInstance)),
  
  findall([Prop,Value], (
    rdf_has(TemplateInstance, Prop, Value),
    % Only handle knowrob properties
    % TODO: handle all properties? Why skipping some?
    ( rdf_split_url('http://knowrob.org/kb/knowrob.owl#', _, Prop)
    ; Prop = 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' )
  ), Props),
  
  forall(
    member([Prop,Value],Props),
    rdf_assert(ObjInstance, Prop, Value)
  ).


%% designator_assert_dimensions(+ObjInstance, +Designator) is nondet.
%
% Assert dimension properties of logged designator.
%
% @param ObjInstance   Identifier of the object
% @param Designator    Identifier of the designator
% 
designator_assert_dimensions(ObjInstance, Designator) :-
  once(mng_designator(Designator, DesignatorJava)),
  designator_assert_dimensions(ObjInstance, Designator, DesignatorJava).

%% designator_assert_dimensions(+ObjInstance, +Designator, +DesignatorJava) is nondet.
%
% Assert dimension properties of logged designator.
%
% @param ObjInstance    Identifier of the object
% @param Designator     Identifier of the designator
% @param DesignatorJava Designator JAVA object
% 
designator_assert_dimensions(ObjInstance, _, _) :-
  rdf_has(ObjInstance, knowrob:'depthOfObject', _).

designator_assert_dimensions(ObjInstance, _, _) :-
  rdf_has(ObjInstance, knowrob:'pathToCadModel', _),
  object_assert_dimensions(ObjInstance, 1.0, 1.0, 1.0).

designator_assert_dimensions(ObjInstance, Designator, DesignatorJava) :-
  jpl_is_object(DesignatorJava),
  mng_designator_props(Designator, DesignatorJava, 'BOUNDINGBOX.DIMENSIONS-3D', [H,W,D]),
  object_assert_dimensions(ObjInstance, H, W, D).

designator_assert_dimensions(ObjInstance, _, _) :-
  object_assert_dimensions(ObjInstance, 0.2, 0.2, 0.2).


%% designator_assert_color(+ObjInstance, +Designator) is nondet.
%
% Assert color properties of logged designator.
%
% @param ObjInstance   Identifier of the object
% @param Designator    Identifier of the designator
% 
designator_assert_color(ObjInstance, Designator) :-
  once(mng_designator(Designator, DesignatorJava)),
  designator_assert_color(ObjInstance, Designator, DesignatorJava).

%% designator_assert_color(+ObjInstance, +Designator, +DesignatorJava) is nondet.
%
% Assert color properties of logged designator.
%
% @param ObjInstance    Identifier of the object
% @param Designator     Identifier of the designator
% @param DesignatorJava Designator JAVA object
% 
designator_assert_color(ObjInstance, _, _) :-
  rdf_has(ObjInstance, knowrob:'mainColorOfObject', _).

designator_assert_color(ObjInstance, Designator, DesignatorJava) :-
  mng_designator_props(Designator, DesignatorJava, 'COLOR', Col),
  object_assert_color(ObjInstance, Col).

%% designator_add_perception(+ObjInstance, +Designator, +Time) is nondet.
%
% Add pose to list of perceived poses of an object.
%
% @param ObjInstance  Identifier of the object
% @param Designator   Identifier of the designator
% @param Time         The time when the object was perceived
% 
designator_add_perception(ObjInstance, Designator, Time) :-
  designator_add_perception(ObjInstance, Designator, _PoseList, Time).

%% designator_add_perception(+ObjInstance, +Designator, ?Pose, +Time) is nondet.
%
% Add pose to list of perceived poses of an object.
%
% @param ObjInstance  Identifier of the object
% @param Designator   Identifier of the designator
% @param Pose         The pose of the object
% @param Time         The time when the object was perceived
% 
designator_add_perception(ObjInstance, Designator, Pose, Time) :-
  var(Pose),
  mng_designator_location(Designator, Pose),
  designator_add_perception(ObjInstance, Designator, Pose, Time).

designator_add_perception(ObjInstance, Designator, Pose, Time) :-
  is_list(Pose),
  create_pose(mat(Pose), Matrix),
  designator_add_perception(ObjInstance, Designator, Matrix, Time).

designator_add_perception(ObjInstance, _Designator, Pose, Time) :-
  atom(Pose),
  rdf_instance_from_class(knowrob:'SemanticMapPerception', Perception),
  rdf_assert(Perception, knowrob:'startTime', Time),
  rdf_assert(Perception, knowrob:'eventOccursAt', Pose),
  set_object_perception(ObjInstance, Perception).

%% designator_between(?Designator, ?PreAction, ?PostAction) is nondet.
%
% Find a designator that was equated after PreAction and before PostAction.
%
% @param Designator   Identifier of the designator
% @param PreAction    Identifier of the action that was performed before the designator was equated
% @param PostAction   Identifier of the action that was performed after the designator was equated
% 
designator_between(Designator, PreAction, PostAction) :-
  task_end(PreAction, T0),
  task_start(PostAction, T1),
  rdfs_individual_of(Designator, knowrob:'CRAMDesignator'),
  rdf_has(Designator, knowrob:'equationTime', T),
  time_term(T0, T_Val0),
  time_term(T1, T_Val1),
  time_term(T, T_Val),
  T_Val =< T_Val1, T_Val >= T_Val0.

show_entity(Entity) :-
  entity(Entity, Descr),
  Descr = [_,Type|Descr_],
  jpl_new('org.knowrob.interfaces.mongo.types.Designator', [], Desig),
  jpl_call(Desig, 'put', ['_designator_type', Type], _),
  read_entity_desig(Desig, Descr_),
  designator_publish(Entity, Desig).

read_entity_desig(_, []).
read_entity_desig(Desig, [[Key,Val]|Descr]) :-
  term_to_atom(Key, KeyAtom),
  (  is_list(Val)
  -> (
    jpl_new('org.knowrob.interfaces.mongo.types.Designator', [], Val_),
    Val = [_, Type|Nested],
    jpl_call(Val_, 'put', ['_designator_type', Type], _),
    read_entity_desig(Val_, Nested)
  ) ; term_to_atom(Val, Val_) ),
  jpl_call(Desig, 'put', [KeyAtom, Val_], _),
  read_entity_desig(Desig, Descr), !.
read_entity_desig(Desig, [_|Descr]) :-
  read_entity_desig(Desig, Descr).

%% designator_publish(+Designator) is nondet.
%
% Publish designator on ROS topic.
%
% @param Designator   Identifier of the designator
% 
designator_publish(Designator) :-
  once(designator_publish(Designator, _DesignatorJava)).

%% designator_publish(+Designator, +DesignatorJava) is nondet.
%
% Publish designator on ROS topic.
%
% @param Designator   Identifier of the designator
% @param DesignatorJava Designator JAVA object
% 
designator_publish(Designator, DesignatorJava) :-
  var(DesignatorJava),
  atom(Designator),
  once(mng_designator(Designator, DesignatorJava)),
  designator_publish(Designator, DesignatorJava).

designator_publish(Designator, DesignatorJava) :-
  atom(Designator),
  jpl_is_object(DesignatorJava),
  
  log_publisher(Client),
  jpl_call(Client, 'publishDesignator', [DesignatorJava], _),
  
  (designator_publish_image(Designator) ; true).

%% designator_publish_image(+Input) is nondet.
%
% Publish designator image on ROS topic.
%
% @param Input Identifier of the designator, a task or the path to the file
% 
designator_publish_image(Input) :-
  task_outcome(Task, Input),
  designator_publish_image(Task), !.

designator_publish_image(Input) :-
  once(rdf_has(Input, rdf:'type', _)), !, % owl individuals have capturedImage or don't get published
  rdf_has(Input, knowrob:'capturedImage', Img),
  rdf_has(Input, knowrob:'startTime', T),
  rdf_has(Img, knowrob:'linkToImageFile', literal(type(_, Path))),

  % Find directory that contains the experiment active at given timepoint
  experiment(Experiment, T),
  rdf_has(Directory, knowrob:experiment, Experiment),
  rdf_has(Directory, rdf:type, knowrob:'RobotExperimentDirectory'),
  
  atomic_list_concat([_Prefix, Dir], '#', Directory),
  atomic_list_concat([Dir, Path], '/', CompletePath),
  designator_publish_image(CompletePath), !.

designator_publish_image(Input) :-
  atom(Input),
  log_publisher(Client),
  jpl_call(Client, 'publishImage', [Input], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Designator expressions
%

%% action_designator_exp(?Task, [?Mode, ?QueryPattern]) is nondet.
%
%  Find a Designator for given expression.
%
%  @param Action Returned Action
%  @param QueryPattern Identifier of given Designator pattern
%
%  Example: ?- action_designator_exp(A, [an, action, [to, grasp]]).
% 
action_designator_exp(Action, QueryPattern) :-
    mng_desig_matches(Designator, QueryPattern),
    rdf_has(Action, knowrob:'designator', Designator).

%% task_designator_exp(?Task, [?Mode, ?QueryPattern]) is nondet.
%
%  Find a Task for given type and designator expression.
%
%  @param Task Returned Task
%  @param Mode Identifier of given Task type
%  @param QueryPattern Identifier of given Designator pattern
%
%  Example: ?- task_designator_exp(T, [perform, [an, action, [to, grasp]]]).
% 
task_designator_exp(Task, [Mode, QueryPattern]) :-
    action_designator_exp(Action, QueryPattern),
    subtask_typed(Task, Action, Mode).
