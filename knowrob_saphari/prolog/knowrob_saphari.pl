
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
            highlight_intrusions(r,r,r,r),
            agent_marker(r,r,r,r),
            agent_connection_marker(r,r,r,r,r).

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
  highlight_object(MarkerId, 250, 0, 0, 250).

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

