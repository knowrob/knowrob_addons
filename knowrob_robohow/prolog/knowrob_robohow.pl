
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
@author Benjamin Brieber
@license GPL
*/


:- module(knowrob_robohow,
    [
      designator_grasped_pose/4,
      designator_estimate_pose/5,
      visualize_preparing_experiment/1,
      visualize_rolling_experiment/1,
      visualize_forth_experiment/1,
      visualize_forth_objects/1,
      show_action_trajectory/1,
      get_image_perception/2,
      get_dynamics_image_perception/2,
      get_sherlock_image_perception/2,
      get_reach_action/2,
      get_roll_action/2,
      get_retract_action/2
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('srdl2')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(forth_human, 'http://knowrob.org/kb/forth_human.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(boxy2, 'http://knowrob.org/kb/BoxyWithRoller.owl#', [keep(true)]).

:-  rdf_meta
  designator_grasped_pose(r,r,r,r),
  designator_estimate_pose(r,r,r,r),
  visualize_preparing_experiment(r),
  visualize_rolling_experiment(r),
  visualize_forth_experiment(r),
  visualize_forth_objects(r),
  show_action_trajectory(r),
  get_dynamics_image_perception(r,r),
  get_sherlock_image_perception(r,r),
  get_reach_action(r,r),
  get_roll_action(r,r),
  get_retract_action(r,r).
  
  
% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
%:- rdf_meta saphari_visualize_human(r,r,r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Designator pose estimation based on grasp/put actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

designator_grasped_pose(T, Action, [X,Y,Z], Rotation) :-
  rdf_has(Action, knowrob:'bodyPartsUsed', BodyPart),
  % TODO(daniel): What if right and left hand are used?
  rdf_has(BodyPart, srdl2comp:'urdfName', literal(URDF)),
  % TODO(daniel): howto make this more accurate? Might yield in artifacts to just use the body part TF frame
  mng_lookup_transform('/map', URDF, T, Transform),
  matrix_translation(Transform, [X,Y,Z]),
  %matrix_rotation(Transform, Rotation),
  %matrix_translation(Transform, Position).
  
  % Use parent link to find rotation
  rdf_has(ParentJoint, srdl2comp:'succeedingLink', BodyPart),
  rdf_has(ParentLink, srdl2comp:'succeedingJoint', ParentJoint),
  rdf_has(ParentLink, srdl2comp:'urdfName', literal(URDFParent)),
  mng_lookup_transform('/map', URDFParent, T, TransformParent),
  matrix_translation(TransformParent, [X_Parent,Y_Parent,Z_Parent]),
  
  Dir_X is X - X_Parent,
  Dir_Y is Y - Y_Parent,
  Dir_Z is Z - Z_Parent,
  jpl_list_to_array([Dir_X, Dir_Y, Dir_Z], DirArr),
  jpl_call('org.knowrob.utils.MathUtil', 'orientationToQuaternion', [DirArr], QuaternionArr),
  jpl_array_to_list(QuaternionArr, Rotation).


designator_estimate_pose(ObjId, T, movable, Position, Rotation) :-
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
  % And that there was no perceive action between Put_T and T
  not((
    rdfs_individual_of(Perc, knowrob:'UIMAPerception'),
    rdf_has(Perc, knowrob:'perceptionResult', ObjId),
    rdf_has(Perc, knowrob:'startTime', Perc_T),
    time_earlier_then(Put_T, Perc_T),
    time_earlier_then(Perc_T, T)
  )),
  % Then the object was put down at timepoint Put_T
  % We just use the pose of the body part that was holding the object
  % at the time when the object was put down
  designator_grasped_pose(Put_T, Put, Position, Rotation).
  %designator_perceived_pose(ObjId, T, _, Rotation).

designator_estimate_pose(ObjId, T, movable, Position, Rotation) :-
  % Here we only check if object was grasped before T,
  % if so the human still holds the object
  rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
  rdf_has(Grasp, knowrob:'objectActedOn', ObjId),
  rdf_has(Grasp, knowrob:'endTime', Grasp_T),
  time_earlier_then(Grasp_T, T),
  % And that there was no perceive action between Grasp_T and T
  not((
    rdfs_individual_of(Perc, knowrob:'UIMAPerception'),
    rdf_has(Perc, knowrob:'perceptionResult', ObjId),
    rdf_has(Perc, knowrob:'startTime', Perc_T),
    time_earlier_then(Grasp_T, Perc_T),
    time_earlier_then(Perc_T, T)
  )),
  designator_grasped_pose(T, Grasp, Position, Rotation).
  %designator_perceived_pose(ObjId, T, _, Rotation).

designator_estimate_pose(ObjId, T, _, Position, Rotation) :-
  % Finally try to find pose in perception events
  designator_perceived_pose(ObjId, T, Position, Rotation).

designator_perceived_pose(ObjId, T, Position, Rotation) :-
  rdfs_individual_of(Perc, knowrob:'UIMAPerception'),
  rdf_has(Perc, knowrob:'perceptionResult', ObjId),
  rdf_has(Perc, knowrob:'startTime', Perc_T),
  time_earlier_then(Perc_T, T),
  % And that there was no perceive action before T and later then Perc_T
  % -> Use latest perception of object
  not((
    rdfs_individual_of(Perc2, knowrob:'UIMAPerception'),
    rdf_has(Perc2, knowrob:'objectActedOn', ObjId),
    rdf_has(Perc2, knowrob:'startTime', Perc2_T),
    time_earlier_then(Perc2_T, T),
    time_earlier_then(Perc_T, Perc2_T)
  )),
  mng_designator_location(ObjId, Transform, T),
  matrix_rotation(Transform, Rotation),
  matrix_translation(Transform, Position).

  
get_reach_action(ActionID, ReachID):-
  rdf_has(ActionID, knowrob:'subAction', _WithDesig),
  findall(_Sub,rdf_has(_WithDesig, knowrob:'subAction', _Sub),_Subs),
  nth0(0, _Subs, ReachID).
  
get_roll_action(ActionID, RollID):-
  rdf_has(ActionID, knowrob:'subAction', _WithDesig),
  findall(_Sub,rdf_has(_WithDesig, knowrob:'subAction', _Sub),_Subs),
  nth0(1, _Subs, RollID).
  
get_retract_action(ActionID, RetractID):-
  rdf_has(ActionID, knowrob:'subAction', _WithDesig),
  findall(_Sub,rdf_has(_WithDesig, knowrob:'subAction', _Sub),_Subs),
  nth0(2, _Subs, RetractID).
  
show_action_trajectory(ActionID):-
  clear_trajectories,
  task_start(ActionID,StAction),
  task_end(ActionID,EAction),
  add_trajectory('right_arm_adapter_kms40_fwk050_frame_out', StAction, EAction,0.2,2),
  add_agent_visualization('BOXY', boxy:'boxy_robot2', StAction, '', '').
  
  
  
  
get_sherlock_image_perception(Parent,Perceive):-
  rdf_has(Parent, knowrob:'subAction', Perceive),
  rdf_has(Perceive,knowrob:'capturedImage',Image).
  rdf_has(Image,knowrob:'rosTopic',literal('/RoboSherlock_dough_detection/output_image')).

get_sherlock_image_perception(Parent,Perceive):-
  rdf_has(Parent, knowrob:'subAction', Sub),
  get_sherlock_image_perception(Sub,Perceive).
  
  
  
get_dynamics_image_perception(Parent,Perceive):-
  rdf_has(Parent, knowrob:'subAction', Perceive),
  rdf_has(Perceive,knowrob:'capturedImage',Image),
  rdf_has(Image,knowrob:'rosTopic',literal('/motion_planner/dynamics_image')).

get_dynamics_image_perception(Parent,Perceive):-
  rdf_has(Parent, knowrob:'subAction', Sub),
  get_dynamics_image_perception(Sub,Perceive).
  
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

forth_object('http://knowrob.org/kb/labels.owl#tray_JKdma8aduNdkOM',
             'package://kitchen/cooking-vessels/tray.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#spoon_Jdna8auH73bCMC',
             'package://unsorted/robohow/spoon3.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#smallRedCup1_2ecD3otVRyYGYQ',
             'package://kitchen/hand-tools/red_cup.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#smallRedCup2_feDa5geCRGasVB',
             'package://kitchen/hand-tools/red_cup.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#cheese_Iuad8anDKa27op',
             'package://unsorted/robohow/bowl_cheese.dae',
             static).
%forth_object('http://knowrob.org/kb/labels.owl#onion_Jam39adKAme1Aa',
%             '',
%             static).
forth_object('http://knowrob.org/kb/labels.owl#tomatoSauce_JameUd81KmdE18',
             'package://unsorted/robohow/bowl_sauce.dae',
             static).
%forth_object('http://knowrob.org/kb/labels.owl#bacon_OAJe81c71DmaEg',
%             'package://kitchen/food-drinks/pizza-credentials/bacon_cube.dae',
%             static).
forth_object('http://knowrob.org/kb/labels.owl#yellowBowl_mdJa91KdAoemAN',
             'package://kitchen/cooking-vessels/yellow_bowl.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#redBowl_Jame81dDNMAkeC',
             'package://kitchen/cooking-vessels/red_bowl.dae',
             movable).
forth_object('http://knowrob.org/kb/labels.owl#pizza_AleMDa28D1Kmvc',
             'package://kitchen/food-drinks/pizza-credentials/pizza.dae',
             movable).

visualize_forth_objects(T) :-
  forall( forth_object(ObjId, MeshPath, Mode), (
    (
      once(designator_estimate_pose(ObjId, T, Mode, Position, Rot)),
      add_mesh(ObjId, MeshPath, Position, Rot)
    ) ; true
  )).

visualize_forth_experiment(T) :-
  % Remove because some links may be missing
  remove_agent_visualization('forth', forth_human:'forth_human_robot1'),
  add_stickman_visualization('forth', forth_human:'forth_human_robot1', T, '', ''),
  visualize_forth_objects(T).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

visualize_rolling_experiment(T) :-
  add_agent_visualization('BOXY', boxy2:'boxy_robot2', T, '', ''),
  mng_latest_designator_with_values(T,
        ['designator.NAME'], ['is'], ['PERCEIVE-OBJECT'],
        Desig),
  add_designator_contour_mesh('DOUGH', Desig, [0.0,0.0,0.0], ['DOUGH', 'CONTOUR']).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

%add_mesh(MarkerId, MeshPath, Position, Rotation)

visualize_preparing_experiment(T) :-
  false.
