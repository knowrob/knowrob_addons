
/** <module> knowrob_robohow

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


:- module(knowrob_robohow,
    [
      designator_grasped_pose/4,
      designator_estimate_pose/4,
      visualize_preparing_experiment/1,
      visualize_rolling_experiment/1,
      visualize_forth_experiment/1,
      visualize_forth_objects/1
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
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(forth_human, 'http://knowrob.org/kb/forth_human.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
%:- rdf_meta saphari_visualize_human(r,r,r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Designator pose estimation based on grasp/put actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

designator_grasped_pose(T, Action, Position, Rotation) :-
  rdf_has(Action, knowrob:'bodyPartsUsed', BodyPart),
  % TODO(daniel): What if right and left hand are used?
  rdf_has(BodyPart, srdl2comp:'urdfName', literal(URDF)),
  % TODO(daniel): howto make this more accurate? Might yield in artifacts to just use the body part TF frame
  mng_lookup_transform('/map', URDF, T, Transform),
  matrix_rotation(Transform, Rotation),
  matrix_translation(Transform, Position).

designator_estimate_pose(ObjId, T, Position, Rotation) :-
  % TODO(daniel): support that objects can be perceived again after putting them down!
  % Check if there was a put down action before
  rdfs_individual_of(Put, knowrob:'PuttingSomethingSomewhere'),
  rdf_has(Put, knowrob:'objectActedOn', ObjId),
  rdf_has(Put, knowrob:'endTime', Put_T),
  time_earlier_then(Put_T, T),
  % And that there was no grasp action between Put_T and T
  not((
    rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
    rdf_has(Grasp, knowrob:'objectActedOn', ObjId),
    rdf_has(Grasp, knowrob:'startTime', Grasp_T),
    time_earlier_then(Put_T, Grasp_T),
    time_earlier_then(Grasp_T, T)
  )),
  % Then the object was put down at timepoint Put_T
  % We just use the pose of the body part that was holding the object
  % at the time when the object was put down
  designator_grasped_pose(Put_T, Put, Position, _),
  designator_perceived_pose(ObjId, T, _, Rotation).

designator_estimate_pose(ObjId, T, Position, Rotation) :-
  % Here we only check if object was grasped before T,
  % if so the human still holds the object
  rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
  rdf_has(Grasp, knowrob:'objectActedOn', ObjId),
  rdf_has(Grasp, knowrob:'endTime', Grasp_T),
  time_earlier_then(Grasp_T, T),
  designator_grasped_pose(T, Grasp, _, Rotation),
  designator_perceived_pose(ObjId, T, _, Rotation).

designator_estimate_pose(ObjId, T, Position, Rotation) :-
  % Finally try to find pose in perception events
  % TODO(daniel): use latest perception of object!
  designator_perceived_pose(ObjId, T, Position, Rotation).

designator_perceived_pose(ObjId, T, Position, Rotation) :-
  rdfs_individual_of(Perc, knowrob:'UIMAPerception'),
  rdf_has(Perc, knowrob:'perceptionResult', ObjId),
  rdf_has(Perc, knowrob:'startTime', Perc_T),
  time_earlier_then(Perc_T, T),
  mng_designator_location(ObjId, Transform),
  matrix_rotation(Transform, Rotation),
  matrix_translation(Transform, Position).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

forth_object('http://knowrob.org/kb/labels.owl#tray_JKdma8aduNdkOM',
             'package://kitchen/cooking-vessels/tray.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#spoon_Jdna8auH73bCMC',
             'package://unsorted/robohow/spoon3.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#smallRedCup1_2ecD3otVRyYGYQ',
             'package://kitchen/hand-tools/red_cup.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#smallRedCup2_feDa5geCRGasVB',
             'package://kitchen/hand-tools/red_cup.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#cheese_Iuad8anDKa27op',
             'package://unsorted/robohow/bowl_cheese.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#onion_Jam39adKAme1Aa',
             '',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#tomatoSauce_JameUd81KmdE18',
             'package://unsorted/robohow/bowl_sauce.dae',
             [1.0,1.0,1.0]).
%forth_object('http://knowrob.org/kb/labels.owl#bacon_OAJe81c71DmaEg',
%             'package://kitchen/food-drinks/pizza-credentials/bacon_cube.dae',
%             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#yellowBowl_mdJa91KdAoemAN',
             'package://kitchen/cooking-vessels/yellow_bowl.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#redBowl_Jame81dDNMAkeC',
             'package://kitchen/cooking-vessels/red_bowl.dae',
             [1.0,1.0,1.0]).
forth_object('http://knowrob.org/kb/labels.owl#pizza_AleMDa28D1Kmvc',
             'package://kitchen/food-drinks/pizza-credentials/pizza.dae',
             [1.0,1.0,1.0]).

visualize_forth_objects(T) :-
  forall( forth_object(ObjId, MeshPath, Scale), (
    (
      once(designator_estimate_pose(ObjId, T, Position, Rot)),
      add_mesh(ObjId, MeshPath, Position, Rot, Scale)
    ) ; true
  )).

visualize_forth_experiment(T) :-
  add_stickman_visualization('forth', forth_human:'forth_human_robot1', T, '', ''),
  visualize_forth_objects(T).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

%add_mesh(MarkerId, MeshPath, Position, Rotation)

visualize_preparing_experiment(T) :-
  false.

visualize_rolling_experiment(T) :-
  false.