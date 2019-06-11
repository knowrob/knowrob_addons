/** <module> knowrob_saphari

  Copyright (C) 2015 by Daniel Beßler

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


:- module(knowrob_saphari,
    [
      agent_marker/4,
      agent_connection_marker/5,
      
      action_designator/3,
      
      intrusion_link/4,
      intrusion_link/5,
      
      saphari_action_events/2,
      saphari_collision_events/2,
      saphari_move_down_events/1,
      saphari_intrusion_events/1,
      
      saphari_visualize_experiment/1,
      saphari_visualize_agents/1,
      saphari_visualize_human/2,
      saphari_visualize_human/3,
      highlight_intrusions/4,
      
      
      saphari_active_task/1,
      saphari_slot_description/3,
      saphari_slot_description/4,
      saphari_slot/1,
      saphari_slot/2,
      saphari_slot_state/2,
      saphari_slot_state/3,
      saphari_slot_pose/3,
      saphari_empty_slot/1,
      saphari_empty_slot/2,
      saphari_empty_slots/1,
      saphari_taken_slot/1,
      saphari_taken_slot/2,
      saphari_object_mesh/2,
      saphari_object_class/3,
      saphari_object_properties/3,
      saphari_object_on_table/1,
      saphari_object_on_table/2,
      saphari_object_in_basket/1,
      saphari_object_in_basket/2,
      saphari_object_in_gripper/1,
      saphari_object_in_gripper/2,
      saphari_perceived_object/1,
      saphari_perceived_object/2,
      saphari_perceived_objects/1,
      saphari_perceived_objects/2,
      saphari_next_object/4,
      saphari_grasping_point/2,
      saphari_basket_goal/1,
      saphari_basket_state/1,
      saphari_objects_on_table/1,
      saphari_objects_in_gripper/1,
      saphari_marker_update/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/comp_temporal')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/marker_vis')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/srdl2')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob_cram')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(saphari, 'http://knowrob.org/kb/saphari.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(openni_human, 'http://knowrob.org/kb/openni_human1.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(boxy, 'http://knowrob.org/kb/Boxy.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta saphari_visualize_human(r,r,r),
            saphari_visualize_human(r,r),
            saphari_visualize_agents(r),
            saphari_collision_events(r,r),
            saphari_action_events(r,r),
            saphari_move_down_events(r),
            saphari_intrusion_events(r),
            saphari_visualize_experiment(r),
            highlight_intrusions(r,r,r,r),
            agent_marker(r,r,r,r),
            action_designator(r,r,r),
            intrusion_link(r,r,r,r),
            intrusion_link(r,r,r,r,r),
            agent_connection_marker(r,r,r,r,r).

action_designator(TaskContext, Timepoint, Designator) :-
  % action with taskContext
  owl_individual_of(Action, knowrob:'CRAMAction'),
  once((
    rdf_has(Action, knowrob:'taskContext', literal(type(_,TaskContext))),
    % attached designator
    rdf_has(Action, knowrob:'subAction', WithDesignators),
    owl_individual_of(WithDesignators, knowrob:'WithDesignators'),
    rdf_has(WithDesignators, knowrob:'designator', Desig),
    % finally the equated designator
    rdf_has(Desig, knowrob:'equatedDesignator', Designator),
    rdf_has(Designator, knowrob:'equationTime', Timepoint)
  )).

agent_marker(Link, _Prefix, Identifier, MarkerId) :-
  term_to_atom(object_without_children(Link), LinkAtom),
  atom_concat(Identifier, '_', Buf),
  atom_concat(Buf, LinkAtom, MarkerId).

agent_connection_marker(Link0, Link1, _Prefix, Identifier, MarkerId) :-
  term_to_atom(cylinder_tf(Link0,Link1), CylinderAtom),
  atom_concat(Identifier, '_', Buf),
  atom_concat(Buf, CylinderAtom, MarkerId).

intrusion_link(Human, HumanPrefix, Timeppoint, HumanLink) :-
  intrusion_link(Human, HumanPrefix, Timeppoint, 1.5, HumanLink).

intrusion_link(Human, HumanPrefix, Timeppoint, Threshold, HumanLink) :-
  sub_component(Human, HumanLink),
  owl_has(HumanLink, srdl2comp:'urdfName', literal(HumanUrdfName)),
  atom_concat(HumanPrefix, HumanUrdfName, HumanTf),
  mng_lookup_position('/shoulder_kinect_rgb_frame', HumanTf, Timeppoint, Position),
  nth0(0, Position, XPosition),
  XPosition < Threshold.
  
highlight_intrusion_danger(MarkerId) :-
  marker(MarkerId, MarkerObject),
  marker_highlight(MarkerObject, [1.0,0.0,0.0,1.0]).

highlight_intrusions(HumanIdentifier, Human, HumanPrefix, Timeppoint) :-
  % Find all links that intersect with the safety area of the robot
  findall(HumanLink,
    intrusion_link(Human, HumanPrefix, Timeppoint, HumanLink),
    IntrusionLinks),
  
  forall(member(HumanLink0, IntrusionLinks), ((
    agent_marker(HumanLink0, HumanPrefix, HumanIdentifier, MarkerId),
    % Highlight link marker
    highlight_intrusion_danger(MarkerId),
    % Highlight connection between marker
    forall(member(HumanLink1, IntrusionLinks), ((
      succeeding_link(HumanLink0, HumanLink1),
      agent_connection_marker(HumanLink0, HumanLink1, HumanPrefix, HumanIdentifier, ConnectionMarkerId),
      highlight_intrusion_danger(ConnectionMarkerId)
    ) ; true))
  ) ; true)).

saphari_visualize_human(HumanPrefix, Timeppoint) :-
  saphari_visualize_human(HumanPrefix, HumanPrefix, Timeppoint).

saphari_visualize_human(HumanIdentifier, HumanPrefix, Timeppoint) :-
  marker(stickman(openni_human:'iai_human_robot1'), _, HumanIdentifier),
  marker_tf_prefix(HumanIdentifier, HumanPrefix),
  marker_update(HumanIdentifier, Timeppoint),
  highlight_intrusions(HumanIdentifier, openni_human:'iai_human_robot1', HumanPrefix, Timeppoint).

saphari_collision_events(Type, Events) :-
  CollisionTypes = ['LIGHT-COLLISION', 'STRONG-COLLISION', 'CONTACT', 'SEVERE-COLLISION'],
  member(Type, CollisionTypes),
  findall(Event, (
    rdf_has(Event, knowrob:'taskContext', literal(type(_,Type))),
    owl_individual_of(Event, knowrob:'CRAMAction')
  ), Events).

saphari_action_events(Type, Events) :-
  ActionTypes = [
      'REPLACEABLE-FUNCTION-MOVE-DOWN-UNTIL-TOUCH',
      'REPLACEABLE-FUNCTION-MOVE-ARM-AWAY',
      'REPLACEABLE-FUNCTION-SAFELY-PERFORM-ACTION',
      'REPLACEABLE-FUNCTION-MOVE-OVER-OBJECT'
  ],
  member(Type, ActionTypes),
  findall(Event, (
    owl_individual_of(Event, knowrob:'CRAMAction'),
    rdf_has(Event, knowrob:'taskContext', literal(type(_,Type)))
  ), Events).

saphari_move_down_events(Events) :-
  findall(Event, (
    owl_individual_of(Event, knowrob:'CRAMAction'),
    rdf_has(Event, knowrob:'taskContext', literal(type(_,'REPLACEABLE-FUNCTION-MOVE-DOWN-UNTIL-TOUCH')))
  ), Events).

saphari_intrusion_events(Events) :-
  findall(Event, (
    owl_individual_of(Event, knowrob:'CRAMAction'),
    rdf_has(Event, knowrob:'taskContext', literal(type(_,'HUMAN-INTRUSION')))
  ), Events).

%saphari_visualize_agents(Timepoint) :-
%  add_agent_visualization('BOXY', boxy:'boxy_robot1', Timepoint, '', ''),
%  
%  forall(event(saphari:'HumanIntrusion', Intrusion, Timepoint), ((
%    owl_has(Intrusion, knowrob:'designator', D),
%    mng_designator_props(D, 'TF-PREFIX', Prefix),
%    saphari_visualize_human(Prefix, Timepoint)
%  ) ; true)).

human_tf_prefix(UserIdJava, Prefix) :-
  jpl_call(UserIdJava, intValue, [], UserId),
  atom_concat('/human', UserId, PrefixA),
  atom_concat(PrefixA, '/', Prefix).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REVIEW 2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

saphari_visualize_humans(Timepoint) :-
  time_term(Timepoint, Time),
  MinTimepoint is Time - 5.0,
  
  mng_designator_distinct_values('designator.USER-ID', UserIds),
  forall(member(UserIdJava, UserIds), ((
    ((
      mng_query_latest('logged_designators', one(_), '__recorded', Time, [
        ['__recorded', '>=', date(MinTimepoint)],
        ['designator.USER-ID', '=', UserIdJava]])
    )
    -> (
      human_tf_prefix(UserIdJava, Prefix),
      saphari_visualize_human(Prefix, Timepoint)
    ) ; (
      human_tf_prefix(UserIdJava, Prefix),
      marker_remove(Prefix)
    ))
  ) ; true)).

saphari_visualize_agents(Timepoint) :-
  marker_update(agent(boxy:'boxy_robot1'), Timepoint),
  saphari_visualize_humans(Timepoint).

unasserted_perceived_object(StartTime, EndTime, Perception, Obj) :-
  owl_individual_of(Perception, knowrob:'UIMAPerception'),
  rdf_has(Perception, knowrob:'perceptionResult', Obj),
  
  % Only assert once
  rdf_split_url(_, ObjID, Obj),
  atom_concat('http://knowrob.org/kb/cram_log.owl#Object_', ObjID, InstanceUrl),
  not( owl_individual_of(InstanceUrl, knowrob:'SpatialThing-Localized') ),
  
  % Assert objects which were perceived between StartTime and EndTime
  owl_has(Perception, knowrob:'endTime', Time),
  time_later_then(Time, StartTime),
  time_earlier_then(Time, EndTime).

assert_perceived_objects(StartTime, EndTime, Map) :-
  findall([Obj,LocList], (
     unasserted_perceived_object(StartTime, EndTime, _Perception, Obj),
     designator_location(Obj,LocList)
  ), Objects), !,
  forall( member([Obj,LocList], Objects), (
    create_pose(LocList, Loc),
    add_object_as_semantic_instance(Obj, Loc, EndTime, Map, _Instance)
  )).

saphari_visualize_map(Experiment, Timepoint) :-
  experiment_map(Experiment, Map),
  
  % XXX: Running into computable error "Would end up in deadlock"
  % Create_pose causes this maybe also add_object_as_semantic_instance.
  % Howto fix it?
  %assert_perceived_objects(StartTime, Timepoint, Map),
  marker_update(object(Map), Timepoint).

saphari_visualize_experiment(Timepoint) :-
  once(experiment(Experiment, Timepoint)),
  marker_highlight_remove(all),
  saphari_visualize_map(Experiment, Timepoint),
  saphari_visualize_agents(Timepoint), !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REVIEW 2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vector_length([X0,Y0,Z0],Length) :-
  Length is sqrt( X0*X0 + Y0*Y0 + Z0*Z0 ).

vector_normalize([X0,Y0,Z0], [X1,Y1,Z1]) :-
  vector_length([X0,Y0,Z0],Length),
  X1 is X0 / Length, Y1 is Y0 / Length, Z1 is Z0 / Length.

vector_average([X0,Y0,Z0], [X1,Y1,Z1], [X2,Y2,Z2]) :-
  X2 is 0.5*(X0+X1), Y2 is 0.5*(Y0+Y1), Z2 is 0.5*(Z0+Z1).

vector_add([X0,Y0,Z0], [X1,Y1,Z1], [X2,Y2,Z2]) :-
  X2 is X0+X1, Y2 is Y0+Y1, Z2 is Z0+Z1.

vector_sub([X0,Y0,Z0], [X1,Y1,Z1], [X2,Y2,Z2]) :-
  X2 is X0-X1, Y2 is Y0-Y1, Z2 is Z0-Z1.

vector_mul([X0,Y0,Z0], [X1,Y1,Z1], [X2,Y2,Z2]) :-
  X2 is X0*X1, Y2 is Y0*Y1, Z2 is Z0*Z1.

vector_mul([X0,Y0,Z0], Scalar, [X2,Y2,Z2]) :-
  number(Scalar),
  X2 is X0*Scalar, Y2 is Y0*Scalar, Z2 is Z0*Scalar.

saphari_marker_update(T) :-
  saphari_active_task(Task,T),
  marker_update(agent('http://knowrob.org/kb/Saphari.owl#saphari_robot1'), T),
  ignore(saphari_object_marker_update(Task,T)),
  ignore(saphari_human_marker_update(T)),
  ignore(saphari_intrusion_marker_update(T)).

saphari_object_marker_update(Task,T) :-
  saphari_perceived_objects(Objs, T), !,
  forall( ( current_predicate(v_saphari_marker,_), v_saphari_marker(Obj) ), (
     % remove all markers that do not correspond to an object that was
     % perceived with latest perception event and that's not inside of the basket
     (  once(( member(Obj, Objs) ; saphari_object_in_basket(Obj,T,Task) ))
     -> true
     ; (
       retract( v_saphari_marker(Obj) ),
       marker_remove(object(Obj))
     ))
  )),
  forall( member(Obj, Objs), (
    saphari_object_marker(Obj),
    marker_update(object(Obj), T)
  )).

saphari_human_marker_update(T) :-
  % Find all active PerceivePerson events
  findall(Event, event(knowrob_cram:'PerceivePerson', Event, T), Events),
  % Remove existing marker without corresponding event
  forall(marker(stickman(Person), M), (
    (  member(Event,Events), rdf_has(Event, knowrob:'detectedPerson', Person)
    -> true
    ;  marker_remove(M)
    )
  )),
  % Update perceived human
  forall(member(Event,Events), saphari_human_marker_update(T, Event)).

saphari_human_marker_update(T, Event) :-
  rdf_has(Event, knowrob:'detectedPerson', Person),
  rdf_has(Person, srdl2comp:'tfPrefix', literal(type(_,TfPrefix))),
  marker(stickman(Person), M, TfPrefix),
  marker_tf_prefix(M, TfPrefix),
  marker_update(M,T).

saphari_intrusion_marker_update(T) :-
  % Remove previous highlights
  ignore(marker_highlight_remove(all)),
  
  % Find all active HumanIntrusion events
  findall(Event, event(saphari:'HumanIntrusion', Event, T), IntrusionEvents),
  
  % highlight intruding body parts of perceived humans
  forall(marker(stickman(Person), _), (
    rdf_has(Person, srdl2comp:'tfPrefix', literal(type(_,TfPrefix))),
    rdf_has(Person, knowrob:'designator', D),
    
    % find all intruding body parts
    findall(Part, (
      member(Event,IntrusionEvents),
      rdf_has(Event, knowrob:'designator', D),
      rdf_has(Event, knowrob:'bodyPartsUsed', literal(type(_,Part)))
    ), Parts),
    
    % Finally highlight intruding parts
    forall(member(Part0,Parts), (
      atom_concat('http://knowrob.org/kb/openni_human1.owl#iai_human_', Part0, HumanLink0),
      term_to_atom(object_without_children(HumanLink0), X0),
      atomic_list_concat([TfPrefix,'_',X0], Marker0),
      marker_highlight(Marker0, [1.0,0.0,0.0,1.0]),
      
      % highlight cylinder marker
      forall(member(Part1,Parts), (
        atom_concat('http://knowrob.org/kb/openni_human1.owl#iai_human_', Part1, HumanLink1),
        (  succeeding_link(HumanLink0, HumanLink1)
        -> (
          term_to_atom(cylinder_tf(HumanLink0,HumanLink1), X1),
          atomic_list_concat([TfPrefix,'_',X1], Marker1),
          marker_highlight(Marker1, [1.0,0.0,0.0,1.0])
        ) ;  true )
      ))
    ))
  )).

saphari_object_marker(Obj) :-
  current_predicate(v_saphari_marker,_),
  v_saphari_marker(Obj), !.

saphari_object_marker(Obj) :-
  assert( v_saphari_marker(Obj) ).



saphari_slot(SlotIdentifier) :-
  saphari_active_task(_TaskIdentifier),
  saphari_slot(_TaskIdentifer, SlotIdentifier).

saphari_slot(TaskIdentifier, SlotIdentifier) :-
  rdf_has(TaskIdentifier, knowrob:objectActedOn, Basket),
  rdf_has(SlotIdentifier, knowrob:physicalPartOf, Basket).

saphari_slot_description(SlotIdentifier, ObjectClass, (Translation, Orientation)) :-
  saphari_active_task(TaskIdentifier),
  saphari_slot_description(TaskIdentifier, SlotIdentifier, ObjectClass, (Translation, Orientation)).

saphari_slot_description(TaskIdentifier, SlotIdentifier, ObjectClass, (Translation, Orientation)) :-
  saphari_slot(TaskIdentifier, SlotIdentifier),
  rdf_has(SlotIdentifier, knowrob:perceptionResponse, literal(ObjectClass)),
  saphari_slot_pose(SlotIdentifier, Translation, Orientation).

saphari_slot_state(SlotIdentifier, InstanceDescription) :-
  saphari_active_task(TaskIdentifier),
  saphari_slot_state(TaskIdentifier, SlotIdentifier, InstanceDescription).

saphari_slot_state(TaskIdentifier, SlotIdentifier, InstanceDescription) :-
  saphari_slot(TaskIdentifier, SlotIdentifier),
  once(
  (saphari_slot_release_action(SlotIdentifier, ReleasingAction) -> 
    rdf_has(ReleasingAction, knowrob:'objectActedOn', DesignatorId),
    saphari_object_properties(DesignatorId, ObjectClass, PoseStamped),
    InstanceDescription = (DesignatorId, ObjectClass, PoseStamped)
  ; InstanceDescription = empty)).

saphari_slot_release_action(SlotIdentifier, ReleasingAction) :-
  % TODO: Make sure that this action is part of the current task!
  % TODO: Not very efficient here, better use slot assignment events
  rdfs_instance_of(ReleasingAction, knowrob:'ReleasingGraspOfSomething'),
  rdf_has(ReleasingAction, knowrob:'goalLocation', Loc),
  mng_designator_props(Loc, 'SLOT-ID', SlotIdentifier).

saphari_slot_pose(SlotIdentifier, Translation, Orientation) :-
  get_time(T),
  map_frame_name(MapFrame),
  object_pose_at_time(SlotIdentifier, T, [MapFrame, _, Translation, Orientation]).

%saphari_active_task(Task) :-
%  % for now assume a task is active when no endTime asserted
%  rdf_has(Task, rdf:type, saphari:'SaphariTaskDescription'),
%  not( rdf_has(Task, knowrob:endTime, _) ), !.

%saphari_active_task(Task, T) :-
%  time_term(T, T_term),
%  rdf_has(Task, rdf:type, saphari:'SaphariTaskDescription'),
%  interval(Task, [T0, T1]),
%  T_term >= T0,
%  T_term =< T1, !.

saphari_active_task(Task) :-
  saphari_latest_task(Task).

saphari_active_task(Task,T) :-
  saphari_latest_task(Task,T).

saphari_latest_task(Task) :-
  current_time(T), saphari_latest_task(Task, T).

saphari_latest_task(Task, Time) :-
  rdfs_individual_of(Task, saphari:'SaphariTaskDescription'),
  rdf_has(Task, knowrob:'startTime', T0),
  time_term(T0, T0_term),
  time_term(Time, Time_term),
  T0_term =< Time_term,
  % Make sure that there is no perception event happening after Task
  % we are only interested in the very last perception event (before Time)
  not((
    rdfs_individual_of(Task1, saphari:'SaphariTaskDescription'),
    rdf_has(Task1, knowrob:'startTime', T1),
    time_term(T1, T1_term),
    T1_term =< Time_term,
    T1_term > T0_term
  )), !.

% Find list of empty slots with corresponding desired object classes for the slots
saphari_empty_slot((SlotId, ObjectClass, Pose)) :-
  saphari_active_task(TaskIdentifier),
  saphari_empty_slot(TaskIdentifier, (SlotId, ObjectClass, Pose)).

saphari_empty_slot(TaskId, (SlotId, ObjectClass, Pose)) :-
  saphari_slot_state(TaskId, SlotId, empty),
  saphari_slot_description(TaskId, SlotId, ObjectClass, Pose).

saphari_empty_slots(Slots) :-
  saphari_active_task(TaskId),
  findall((SlotId,ObjectClass,Pose), (
      saphari_slot_state(SlotId, empty),
      saphari_slot_description(TaskId, SlotId, ObjectClass, Pose)
  ), Slots).

% Find occupied slots with corresponding slot information
saphari_taken_slot((SlotId, ObjectClass, Pose)) :-
  saphari_active_task(TaskIdentifier),
  saphari_taken_slot(TaskIdentifier, (SlotId, ObjectClass, Pose)).

saphari_taken_slot(TaskId, (SlotId, ObjectClass, Pose)) :-
  saphari_slot(TaskId, SlotId),
  not( saphari_empty_slot(TaskId, (SlotId, _, _)) ),
  saphari_slot_description(TaskId, SlotId, ObjectClass, Pose).

% Find a mesh that corresponds to the object class
saphari_object_mesh(ObjectClass, ObjectMesh) :-
  rdf_has(ClsIndividual, knowrob:'perceptionResponse', ObjectClass),
  rdf_has(ClsIndividual, knowrob:'pathToCadModel', ObjectMesh).

% Read class property from designator
saphari_object_class(Identifier, Designator, Class) :-
  mng_designator_props(Identifier, Designator, ['RESPONSE'], Class), !.
saphari_object_class(Identifier, Designator, Class) :-
  mng_designator_props(Identifier, Designator, ['TYPE'], Class), !.

% Read some object properties
saphari_object_properties(DesignatorId, ObjectClass, (FrameId, TimeStamp, (Translation, Orientation))) :-
  (  mng_designator(DesignatorId, DesignatorJava) ; (
     mng_designator(DesignatorId, ParentDesig, [], 'designator.OBJ._id'),
     jpl_call(ParentDesig, get, ['OBJ'], DesignatorJava)
  )),
  saphari_object_class(DesignatorId, DesignatorJava, ObjectClass),
  mng_designator_location(DesignatorJava, PoseMatrix),
  matrix(PoseMatrix, Translation, Orientation),
  mng_designator_props(DesignatorId, DesignatorJava, ['AT', 'POSE'], DesigPoseStamped),
  jpl_get(DesigPoseStamped, 'frameID', FrameId),
  jpl_get(DesigPoseStamped, 'timeStamp', TimeStampIso),
  jpl_call(TimeStampIso, 'toSeconds', [], TimeStamp).

saphari_object_on_table(ObjectId) :-
  current_time(T), saphari_object_on_table(ObjectId, T).

saphari_object_on_table(ObjectId, T) :-
  saphari_perceived_objects(Perceptions, T),
  member(ObjectId, Perceptions),
  not(saphari_object_in_gripper(ObjectId,T)),
  not(saphari_object_in_basket(ObjectId,T)).
  
% TODO(daniel): check if actions successfull ?

saphari_object_in_basket(ObjectId) :-
  rdf_has(Release, knowrob:'objectActedOn', ObjectId),
  rdfs_instance_of(Release, knowrob:'ReleasingGraspOfSomething').
  
saphari_object_in_basket(ObjectId, Time) :-
  rdf_has(Release, knowrob:'objectActedOn', ObjectId),
  event_before(knowrob:'ReleasingGraspOfSomething', Release, Time).

saphari_object_in_basket(ObjectId, Time, _Task) :-
  rdf_has(ObjectId, knowrob:'successorDesignator', Designator),
  rdf_has(Release, knowrob:'objectActedOn', Designator),
  event_before(knowrob:'ReleasingGraspOfSomething', Release, Time).

% TODO: why checking for task interval?
%saphari_object_in_basket(ObjectId, Time, Task) :-
%  rdf_has(ObjectId, knowrob:'successorDesignator', Designator),
%  rdf_has(Release, knowrob:'objectActedOn', Designator),
%  event_before(knowrob:'ReleasingGraspOfSomething', Release, Time),
%  interval(Task, [T0, T1]),
%  rdf_has(Release, knowrob:'startTime', Release_T), time_term(Release_T, Release_T_term),
%  Release_T_term >= T0,
%  Release_T_term =< T1.

saphari_object_in_gripper(ObjectId) :-
  rdf_has(Grasp, knowrob:'objectActedOn', ObjectId),
  rdfs_instance_of(Grasp, knowrob:'GraspingSomething'),
  not(saphari_object_in_basket(ObjectId)).

saphari_object_in_gripper(ObjectId, Time) :-
  rdf_has(Grasp, knowrob:'objectActedOn', ObjectId),
  event_before(knowrob:'GraspingSomething', Grasp, Time),
  not(saphari_object_in_basket(ObjectId, Time)), !.

% Yields in a list of designator ids that were perceived in the last perception event
saphari_perceived_objects(PerceivedObjectIds) :-
  current_time(T), saphari_perceived_objects(PerceivedObjectIds, T).

saphari_perceived_objects(PerceivedObjectIds, Time) :-
  saphari_latest_perception(Event, Time),
  findall(ObjId, rdf_has(Event, knowrob:'perceptionResult', ObjId), PerceivedObjectIds).

saphari_perceived_object(PerceivedObjectId) :-
  current_time(T), saphari_perceived_object(PerceivedObjectId, T).

saphari_perceived_object(PerceivedObjectId, Time) :-
  saphari_latest_perception(Event, Time),
  rdf_has(Event, knowrob:'perceptionResult', PerceivedObjectId).

saphari_latest_perception(Perc0, Time) :-
  rdfs_individual_of(Perc0, knowrob:'UIMAPerception'),
  rdf_has(Perc0, knowrob:'endTime', T0),
  time_term(T0, T0_term),
  time_term(Time, Time_term),
  T0_term =< Time_term,
  % Make sure that there is no perception event happening after Perc0
  % we are only interested in the very last perception event (before Time)
  not((
    rdfs_individual_of(Perc1, knowrob:'UIMAPerception'),
    rdf_has(Perc1, knowrob:'endTime', T1),
    time_term(T1, T1_term),
    T1_term =< Time_term,
    T1_term > T0_term
  )), !.


% Find next possible target object for putting it into the basket
% by matching desired classes in empty slots with latest perceived object classes
saphari_next_object(SlotId, (SlotTranslation, SlotRotation), ObjectClass, DesigId) :-
  saphari_empty_slot((SlotId, ObjectClass, (SlotTranslation, SlotRotation))),
  saphari_perceived_object(DesigId),
  saphari_object_properties(DesigId, ObjectClass, _),
  saphari_object_on_table(DesigId).

saphari_grasping_point(ObjectId, _GraspingPose) :-
  saphari_object_properties(ObjectId, ObjectClass, _),
  saphari_object_mesh(ObjectClass, _ObjectMesh),
  % TODO: compute grasping pose. Howto do that? By using CAD model segmentation?
  false.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% ROS message helper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% SlotGoalDescription[] basket_goal
saphari_basket_goal(SlotGoalDescriptions) :-
  saphari_active_task(TaskId),
  findall((SlotId,ObjectClass),
    saphari_slot_description(TaskId, SlotId, ObjectClass),
    SlotGoalDescriptions
  ).

% SlotStateDescription[] basket_state
saphari_basket_state(SlotStateDescriptions) :-
  findall((SlotId,ObjectId,ObjectClass,PoseMatrix), (
    saphari_slot_state(SlotId, (ObjectId, ObjectClass, PoseMatrix))
  ), SlotStateDescriptions).

% ObjectInstanceDescription[] objects_on_table
saphari_objects_on_table(ObjectInstanceDescriptions) :-
  findall((ObjectId,ObjectClass,PoseStamped), (
    saphari_object_on_table(ObjectId),
    saphari_object_properties(ObjectId, ObjectClass, PoseStamped)
  ), ObjectInstanceDescriptions).

% ObjectInstanceDescription[] objects_on_table
saphari_objects_in_gripper(ObjectInstanceDescriptions) :-
  findall((ObjectId,ObjectClass,PoseStamped), (
    saphari_object_in_gripper(ObjectId),
    saphari_object_properties(ObjectId, ObjectClass, PoseStamped)
  ), ObjectInstanceDescriptions).
