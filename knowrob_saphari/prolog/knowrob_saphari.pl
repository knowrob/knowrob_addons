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
      saphari_slot_state/2,
      saphari_slot_state/3,
      saphari_slot_pose/3,
      saphari_empty_slots/1,
      saphari_object_mesh/2,
      saphari_object_class/3,
      saphari_object_properties/3,
      saphari_object_on_table/1,
      saphari_object_in_basket/1,
      saphari_object_in_gripper/1,
      saphari_perceived_objects/1,
      saphari_next_object/4,
      saphari_grasping_point/2,
      saphari_basket_goal/1,
      saphari_basket_state/1,
      saphari_objects_on_table/1,
      saphari_objects_in_gripper/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('srdl2')).
:- use_module(library('knowrob_cram')).
:- use_module(library('knowrob_objects')).

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

agent_marker(Link, Prefix, Identifier, MarkerId) :-
  term_to_atom(object_without_children(Link), LinkAtom),
  atom_concat(Identifier, '_', Buf),
  atom_concat(Buf, LinkAtom, MarkerId).

agent_connection_marker(Link0, Link1, Prefix, Identifier, MarkerId) :-
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
    owl_individual_of(Event, knowrob:'CRAMAction'),
    rdf_has(Event, knowrob:'taskContext', literal(type(_,Type)))
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

% Fact that defines the axctive taskId
%:- assert( saphari_active_task(none) ).

% saphari_slot_description(TaskId, SlotId, ObjectCLass)
% Facts that define which objects should be put into which slot w.r.t. the task.
%:- assert( saphari_slot_description('Task0', 'Slot0', 'Scalpel') ).
%:- assert( saphari_slot_description('Task0', 'Slot1', 'KidneyDish') ).
%:- assert( saphari_slot_description('Task0', 'Slot2', 'Scissors') ).

saphari_slot_description(SlotIdentifier, ObjectClass, (Translation, Orientation)) :-
  saphari_active_task(TaskIdentifier),
  saphari_slot_description(TaskIdentifier, SlotIdentifier, ObjectClass, (Translation, Orientation)).

saphari_slot_description(TaskIdentifier, SlotIdentifier, ObjectClass, (Translation, Orientation)) :-
  rdf_has(TaskIdentifier, knowrob:objectActedOn, Basket),
  rdf_has(SlotIdentifier, knowrob:physicalPartOf, Basket),
  rdf_has(SlotIdentifier, knowrob:perceptionResponse, literal(ObjectClass)),
  saphari_slot_pose(SlotIdentifier, Translation, Orientation).

% Facts that define the current state of a slot (empty or the corresponding designator id)
%:- assert( saphari_slot_state('Slot0', empty) ).
%:- assert( saphari_slot_state('Slot1', empty) ).
%:- assert( saphari_slot_state('Slot2', empty) ).

saphari_slot_state(SlotIdentifier, InstanceDescription) :-
  saphari_active_task(TaskIdentifier),
  saphari_slot_state(TaskIdentifier, SlotIdentifier, InstanceDescription).

saphari_slot_state(TaskIdentifier, SlotIdentifier, InstanceDescription) :-
  rdf_has(TaskIdentifier, knowrob:objectActedOn, Basket),
  rdf_has(SlotIdentifier, knowrob:physicalPartOf, Basket),
  ((  rdf_has(Assignment, rdf:type, saphari:'SaphariSlotAssignment'),
      rdf_has(Assignment, knowrob:objectActedOn, SlotIdentifier),
      rdf_has(Assignment, knowrob:designator, DesignatorId),
      saphari_object_properties(DesignatorId, ObjectClass, PoseStamped),
      InstanceDescription = (DesignatorId, ObjectClass, PoseStamped)
  ) ; InstanceDescription = empty).

saphari_slot_pose(SlotIdentifier, Translation, Orientation) :-
  get_time(T),
  object_pose_at_time(SlotIdentifier, T, Translation, Orientation).

saphari_active_task(Task) :-
  % for now assume a task is active when no endTime asserted
  rdf_has(Task, rdf:type, saphari:'SaphariTaskDescription'),
  not( rdf_has(Task, knowrob:endTime, _) ).

%saphari_basket_initialize(Slots) :-
%  saphari_active_task(TaskIdentifier),
%  saphari_basket_initialize(TaskIdentifier, Slots).

%saphari_basket_initialize(TaskIdentifier, Slots) :-
%  % TODO: remove previous basket definition
%  % TODO: howto owl logging?
%  rdf_instance_from_class(saphari:'Basket', Basket),
%  rdf_assert(TaskIdentifier, knowrob:objectActedOn, Basket),
%  forall( member((PerceptionResponse,Pose),Slots), (
%      rdf_instance_from_class(saphari:'BasketSlot', BasketSlot),
%      rdf_assert(BasketSlot, knowrob:perceptionResponse, literal(type(xsd:string, PerceptionResponse))),
%      rdf_assert(BasketSlot, knowrob:physicalPartOf, Basket)
%      % TODO: assert pose
%  )).

%saphari_basket_put(SlotIdentifier, DesignatorId, Assignment) :-
%  rdf_instance_from_class(saphari:'SaphariSlotAssignment', Assignment),
%  rdf_assert(Assignment, knowrob:objectActedOn, SlotIdentifier),
%  rdf_assert(Assignment, knowrob:designator, DesignatorId).


% Reset the slot state and assert active experiment
%saphari_task_initialize(TaskId) :-
%  % Assert the active task id
%  retract(saphari_active_task(_)),
%  assert(saphari_active_task(TaskId)),
%  % Assume all slots are empty
%  forall(
%    saphari_slot_description(TaskId, SlotId, _), (
%    retract( saphari_slot_state(SlotId,_) ),
%    assert( saphari_slot_state(SlotId, empty) )
%  )).

%saphari_slot_assign(SlotId, ObjId, ObjClass, PoseMatrix) :-
%  saphari_active_task(TaskId),
%  saphari_slot_description(TaskId, SlotId, ObjClass),
%  assert( saphari_slot_state(SlotId, (ObjId,ObjClass,PoseMatrix)) ).

% Find list of empty slots with corresponding desired object classes for the slots
saphari_empty_slots(Slots) :-
  saphari_active_task(TaskId),
  findall((SlotId,ObjectClass,Pose), (
      saphari_slot_state(SlotId, empty),
      saphari_slot_description(TaskId, SlotId, ObjectClass, Pose)
  ), Slots).

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
  mng_designator(DesignatorId, DesignatorJava),
  saphari_object_class(DesignatorId, DesignatorJava, ObjectClass),
  mng_designator_location(DesignatorJava, PoseMatrix),
  matrix_translation(PoseMatrix, Translation),
  matrix_rotation(PoseMatrix, Orientation),
  mng_designator_props(DesignatorId, DesignatorJava, ['AT', 'POSE'], DesigPoseStamped),
  jpl_get(DesigPoseStamped, 'frameID', FrameId),
  jpl_get(DesigPoseStamped, 'timeStamp', TimeStampIso),
  jpl_call(TimeStampIso, 'toSeconds', [], TimeStamp).

saphari_object_on_table(ObjectId) :-
  saphari_perceived_objects(Perceptions),
  member(ObjectId, Perceptions),
  not(saphari_object_in_gripper(ObjectId)),
  not(saphari_object_in_basket(ObjectId)).
  
saphari_object_in_basket(ObjectId) :-
  saphari_slot_state(_, (ObjectId,_,_)).

saphari_object_in_gripper(ObjectId) :-
  rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
  rdf_has(Grasp, knowrob:'objectActedOn', ObjectId),
  rdf_has(Grasp, knowrob:'endTime', Grasp_T),
  time_term(Grasp_T, Grasp_T_term),
  % Make sure that there is no put down event after the grasp
  not((
    rdfs_individual_of(Put, knowrob:'PuttingSomethingSomewhere'),
    rdf_has(Put, knowrob:'objectActedOn', ObjectId),
    rdf_has(Put, knowrob:'endTime', Put_T),
    time_term(Put_T, Put_T_term),
    Put_T_term > Grasp_T_term
  )).

% Yields in a list of designator ids that were perceived in the last perception event
% XXX: may yield unwanted results when other task logs are asserted to the KB
saphari_perceived_objects(PerceivedObjectIds) :-
  rdfs_individual_of(Perc0, knowrob:'UIMAPerception'),
  rdf_has(Perc0, knowrob:'startTime', T0),
  time_term(T0, T0_term),
  % Make sure that there is no perception event happening after Perc0
  % we are only interested in the very last perception event
  not((
    rdfs_individual_of(Perc1, knowrob:'UIMAPerception'),
    rdf_has(Perc1, knowrob:'startTime', T1),
    time_term(T1, T1_term),
    T1_term > T0_term
  )),
  findall(ObjId, rdf_has(Perc0, knowrob:'perceptionResult', ObjId), PerceivedObjectIds).

% Find next possible target object for putting it into the basket
% by matching desired classes in empty slots with latest perceived object classes
saphari_next_object(SlotId, (SlotTranslation, SlotRotation), ObjectClass, DesigId) :-
  saphari_empty_slots(Slots),
  saphari_perceived_objects(Perceptions),
  member((SlotId,ObjectClass,(SlotTranslation, SlotRotation)), Slots),
  member(DesigId, Perceptions),
  saphari_object_properties(DesigId, ObjectClass, _),
  saphari_object_on_table(DesigId).

saphari_grasping_point(ObjectId, GraspingPose) :-
  saphari_object_properties(ObjectId, ObjectClass, _),
  saphari_object_mesh(ObjectClass, ObjectMesh),
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
  saphari_perceived_objects(PerceivedObjectIds),
  findall((ObjectId,ObjectClass,PoseStamped), (
    member(ObjectId, PerceivedObjectIds),
    saphari_object_on_table(ObjectId),
    saphari_object_properties(ObjectId, ObjectClass, PoseStamped)
  ), ObjectInstanceDescriptions).

% ObjectInstanceDescription[] objects_on_table
saphari_objects_in_gripper(ObjectInstanceDescriptions) :-
  saphari_perceived_objects(PerceivedObjectIds),
  findall((ObjectId,ObjectClass,PoseStamped), (
    member(ObjectId, PerceivedObjectIds),
    saphari_object_in_gripper(ObjectId),
    saphari_object_properties(ObjectId, ObjectClass, PoseStamped)
  ), ObjectInstanceDescriptions).

  
  
  
%saphari_latest_object_detections(Classes, dt(TimeRange), Detections) :-
%  get_timepoint(T1), !,
%  time_term(T1, T1_Term),
%  T0_Term is T1_Term - TimeRange,
%  atom_concat('http://knowrob.org/kb/knowrob.owl#timepoint_', T0_Term, T0),
%  saphari_latest_object_detections(Classes, interval(T0,T1), Detections).

%saphari_latest_object_detections(Classes, interval(Start,End), ValidDetections) :-
%  saphari_latest_object_detections(Classes, Detections),
%  findall( ValidDetection, (
%    member(Detection, Detections),
%    owl_has(Detection, knowrob:'startTime', StartTime),
%    (( time_later_then(StartTime,Start),
%       time_earlier_then(StartTime,End) )
%    -> ValidDetection = Detection
%    ;  ValidDetection = none
%    )
%  ), ValidDetections).

%saphari_latest_object_detections(Classes, Detections) :-
%  findall(Detection, (
%    member(Class, Classes),
%    saphari_latest_object_detection(Class, Detection)
%  ), Detections).

%saphari_perception_designator(Class, Obj, Start, End) :-
%  task_type(Perc,knowrob:'UIMAPerception'),
%  rdf_has(Perc, knowrob:'perceptionResult', Obj),
%  mng_designator(Obj, ObjJava),
%  once(saphari_object_class(Obj, ObjJava, Class)),
%  rdf_has(Perc, knowrob:'startTime', Start),
%  rdf_has(Perc, knowrob:'endTime', End).

%saphari_latest_object_detection(Class, Obj) :-
%  get_timepoint(Now), !,
%  saphari_latest_object_detection(Class, Obj, Now).

%saphari_latest_object_detection(Class, Obj, Now) :-
%  saphari_perception_designator(Class, Obj, _, End),
%  time_earlier_then(End, Now),
%  % Make sure there is no later event
%  not((
%    saphari_perception_designator(Class, Obj2, _, End2),
%    not(Obj = Obj2),
%    time_earlier_then(End2, Now),
%    time_later_then(End2, End)
%  )).
