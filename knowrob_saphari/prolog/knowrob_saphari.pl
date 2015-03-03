
/** <module> knowrob_saphari

  Copyright (C) 2015 by Daniel Beßler

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Daniel Beßler
@license GPL
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
      highlight_intrusions/4
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('srdl2')).

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

agent_tf_frame(Link, Prefix, TfFrame) :-
  owl_has(Link, srdl2comp:'urdfName', literal(UrdfName)),
  atom_concat(Prefix, UrdfName, Buffer),
  atom_concat(Buffer, '_', TfFrame).

agent_marker(Link, Prefix, Identifier, MarkerId) :-
  agent_tf_frame(Link, Prefix, TfFrame),
  atom_concat(TfFrame, Identifier, MarkerId).

agent_connection_marker(Link0, Link1, Prefix, Identifier, MarkerId) :-
  agent_tf_frame(Link0, Prefix, TfFrame0),
  agent_tf_frame(Link1, Prefix, TfFrame1),
  atom_concat(TfFrame0, TfFrame1, TfFrame),
  atom_concat(TfFrame, Identifier, MarkerId).

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
  highlight_object(MarkerId, 255, 0, 0, 255).

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
  add_stickman_visualization(
      HumanIdentifier, openni_human:'iai_human_robot1',
      Timeppoint, '', HumanPrefix
  ),
  highlight_intrusions(HumanIdentifier, 
      openni_human:'iai_human_robot1', HumanPrefix,
      Timeppoint
  ).

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

saphari_visualize_humans(Timepoint) :-
  time_term(Timepoint, Time),
  MinTimepoint is Time - 0.5,
  
  mng_designator_distinct_values('designator.USER-ID', UserIds),
  forall(member(UserIdJava, UserIds), ((
    ((
      mng_latest_designator(Time, [
        ['__recorded', '>=', date(MinTimepoint)],
        ['designator.USER-ID', '=', UserIdJava]
      ], _)
    )
    -> (
      human_tf_prefix(UserIdJava, Prefix),
      saphari_visualize_human(Prefix, Timepoint)
    ) ; (
      human_tf_prefix(UserIdJava, Prefix),
      remove_agent_visualization(Prefix, openni_human:'iai_human_robot1')
    ))
  ) ; true)).

saphari_visualize_agents(Timepoint) :-
  add_agent_visualization('BOXY', boxy:'boxy_robot1', Timepoint, '', ''),
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
  
  update_object_with_children(Map, Timepoint).

saphari_visualize_experiment(Timepoint) :-
  once(experiment(Experiment, Timepoint)),
  
  saphari_visualize_map(Experiment, Timepoint),
  saphari_visualize_agents(Timepoint), !.
