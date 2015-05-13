/** <module> knowrob_robohow

  Copyright (C) 2015 Daniel Beßler, Benjamin Brieber, Asil Kaan Bozcuoglu
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

  @author Daniel Beßler
  @author Benjamin Brieber
  @author Asil Kaan Bozcuoglu
  @license BSD
*/

:- module(knowrob_robohow,
    [
      designator_grasped_pose/5,
      designator_estimate_pose/5,
      visualize_pnp_experiment/1,
      visualize_rolling_experiment/1,
      visualize_forth_experiment/1,
      visualize_forth_objects/1,
      show_action_trajectory/1,
      get_dynamics_image_perception/2,
      get_sherlock_image_perception/2,
      get_reach_action/2,
      get_roll_action/2,
      get_retract_action/2,
      experiment/1,
      experiment_start/2,
      experiment_end/2,
      forth_task_start/2,
      forth_task_end/2
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
:- rdf_db:rdf_register_ns(pr2, 'http://knowrob.org/kb/PR2.owl#', [keep(true)]).

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

human_hand_pose(BodyPart, T, [X,Y,Z], Rotation) :-
  writeln(T),
  % Find parent joint
  rdf_has(ParentJoint, srdl2comp:'succeedingLink', BodyPart),
  rdf_has(ParentLink, srdl2comp:'succeedingJoint', ParentJoint),
  % Read TF frame names
  rdf_has(BodyPart, srdl2comp:'urdfName', literal(URDF)),
  rdf_has(ParentLink, srdl2comp:'urdfName', literal(URDFParent)),
  % Lookup TF frames
  mng_lookup_transform('/map', URDF, T, Transform),
  mng_lookup_transform('/map', URDFParent, T, TransformParent),
  % Find the translation of both joints
  matrix_translation(Transform, [X,Y,Z]),
  matrix_translation(TransformParent, [X_Parent,Y_Parent,Z_Parent]),
  % Estimate rotation based on joint positions
  Dir_X is X - X_Parent,
  Dir_Y is Y - Y_Parent,
  Dir_Z is Z - Z_Parent,
  jpl_list_to_array([Dir_X, Dir_Y, Dir_Z], DirArr),
  jpl_call('org.knowrob.utils.MathUtil', 'orientationToQuaternion', [DirArr], QuaternionArr),
  jpl_array_to_list(QuaternionArr, Rotation).
  

designator_grasped_pose(Grasp, ObjId, T, Position, Rotation) :-
  % TODO(daniel): What if right and left hand are used?
  %     - Translation: use center of both joints
  %     - Rotation: ....
  % TODO(daniel): Object might not be grasped at center of object
  %     - Use knowledge about graspable object parts
  %     - Use planner knowledge where the grasp was done
  once(rdf_has(Action, knowrob:'bodyPartsUsed', BodyPart)),
  % The time when the object was grasped
  rdf_has(Grasp, knowrob:'endTime', Grasp_T),
  % Find the pose of the object before the grasp
  designator_perceived_pose(ObjId, Grasp_T, _, Ro_0),
  % Find the pose of the hand before the grasp
  human_hand_pose(BodyPart, Grasp_T, _, Rh_0),
  % Find the pose of the hand for current timestamp
  human_hand_pose(BodyPart, T, Ph_T, Rh_T),
  
  % Just apply the position of the hand for now
  Position = Ph_T,
  
  % Find rotation difference of hand between Grasp_T and T
  jpl_list_to_array(Rh_0, Rh_0_Arr),
  jpl_list_to_array(Rh_T, Rh_T_Arr),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionDifference', [Rh_0_Arr, Rh_T_Arr], Rh_Diff_Arr),
  
  % Apply quaternion difference to Ro_0
  jpl_list_to_array(Ro_0, Ro_0_Arr),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionMultiply', [Ro_0_Arr, Rh_Diff_Arr], Rot_Arr),
  jpl_array_to_list(Rot_Arr, Rotation).


%designator_grasped_pose(T, Action, [X,Y,Z], Rotation) :-
%  rdf_has(Action, knowrob:'bodyPartsUsed', BodyPart),
%  % TODO(daniel): What if right and left hand are used?
%  rdf_has(BodyPart, srdl2comp:'urdfName', literal(URDF)),
%  % TODO(daniel): howto make this more accurate? Might yield in artifacts to just use the body part TF frame
%  mng_lookup_transform('/map', URDF, T, Transform),
%  matrix_translation(Transform, [X,Y,Z]),
%  %matrix_rotation(Transform, Rotation),
%  %matrix_translation(Transform, Position).
%  
%  % Use parent link to find rotation
%  rdf_has(ParentJoint, srdl2comp:'succeedingLink', BodyPart),
%  rdf_has(ParentLink, srdl2comp:'succeedingJoint', ParentJoint),
%  rdf_has(ParentLink, srdl2comp:'urdfName', literal(URDFParent)),
%  mng_lookup_transform('/map', URDFParent, T, TransformParent),
%  matrix_translation(TransformParent, [X_Parent,Y_Parent,Z_Parent]),
%  
%  Dir_X is X - X_Parent,
%  Dir_Y is Y - Y_Parent,
%  Dir_Z is Z - Z_Parent,
%  jpl_list_to_array([Dir_X, Dir_Y, Dir_Z], DirArr),
%  jpl_call('org.knowrob.utils.MathUtil', 'orientationToQuaternion', [DirArr], QuaternionArr),
%  jpl_array_to_list(QuaternionArr, Rotation).

designator_estimate_pose(ObjId, T, movable, Position, Rotation) :-
  % Check if there was a put down action before
  rdfs_individual_of(Put, knowrob:'PuttingSomethingSomewhere'),
  rdf_has(Put, knowrob:'objectActedOn', ObjId),
  rdf_has(Put, knowrob:'endTime', Put_T),
  time_earlier_then(Put_T, T),
  % And that there was no grasp action between Put_T and T (i.e., object was not grasped again after putdown action)
  not((
    rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
    rdf_has(Grasp, knowrob:'objectActedOn', ObjId),
    rdf_has(Grasp, knowrob:'startTime', Grasp_T),
    time_earlier_then(Put_T, Grasp_T),
    time_earlier_then(Grasp_T, T)
  )),
  % And that there was no perceive action between Put_T and T (i.e., object was not perceived after putdown action)
  not((
    rdfs_individual_of(Perc, knowrob:'UIMAPerception'),
    rdf_has(Perc, knowrob:'perceptionResult', ObjId),
    rdf_has(Perc, knowrob:'startTime', Perc_T),
    time_earlier_then(Put_T, Perc_T),
    time_earlier_then(Perc_T, T)
  )),
  writeln('PUTDOWN POSE'),
  % Find latest grasp that occurred before the PutDown happened
  designator_latest_grasp(ObjId, Put_T, Grasp),
  % Then the object was put down at timepoint Put_T.
  designator_grasped_pose(Grasp, ObjId, Put_T, Position, Rotation).
  %designator_grasped_pose(Put_T, Put, Position, Rotation).

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
  writeln('HOLDING POSE'),
  designator_grasped_pose(Grasp, ObjId, T, Position, Rotation).
  %designator_grasped_pose(T, Grasp, Position, Rotation).

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

designator_latest_grasp(ObjId, T, Grasp) :-
  rdfs_individual_of(Grasp, knowrob:'GraspingSomething'),
  rdf_has(Grasp, knowrob:'objectActedOn', ObjId),
  rdf_has(Grasp, knowrob:'endTime', Grasp_T),
  time_earlier_then(Grasp_T, T),
  % Make sure there is no later grasp before T
  not((
    rdfs_individual_of(Grasp_2, knowrob:'GraspingSomething'),
    rdf_has(Grasp_2, knowrob:'objectActedOn', ObjId),
    rdf_has(Grasp_2, knowrob:'startTime', Grasp_T_2),
    time_earlier_then(Grasp_T_2, T),
    time_earlier_then(Grasp_T, Grasp_T_2)
  )).

  
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Rolling experiment helper methods
%%%%%%%%%%%%%%%%%%%%%%%
  
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
  add_agent_visualization('BOXY', boxy2:'boxy_robot2', StAction, '', '').
  
  
  
  
get_sherlock_image_perception(Parent,Perceive):-
  rdf_has(Parent, knowrob:'subAction', Perceive),
  rdf_has(Perceive,knowrob:'capturedImage',Image),
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


visualize_rolling_experiment(T) :-
  add_agent_visualization('BOXY', boxy2:'boxy_robot2', T, '', ''),
  mng_latest_designator_with_values(T, ['designator.DOUGH'], ['exist'], ['true'], Desig),
  add_designator_contour_mesh('DOUGH', Desig, [0.6,0.6,0.2], ['DOUGH', 'CONTOUR']).
  
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

experiment(E) :-
  owl_individual_of(E, knowrob:'RobotExperiment').

experiment_start(T,S) :-
  experiment(T),
  rdf_has(T, knowrob:'startTime', S).

experiment_end(T,E) :-
  experiment(T),
  rdf_has(T, knowrob:'endTime', E).

forth_task_start(T,S) :-
  rdf_has(T, knowrob:'startTime', S).

forth_task_end(T,E) :-
  rdf_has(T, knowrob:'endTime', E).

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
%             'package://unsorted/robohow/cup_onions.dae',
%             static).
forth_object('http://knowrob.org/kb/labels.owl#tomatoSauce_JameUd81KmdE18',
             'package://unsorted/robohow/bowl_sauce.dae',
             static).
%forth_object('http://knowrob.org/kb/labels.owl#bacon_OAJe81c71DmaEg',
%             'package://unsorted/robohow/cup_bacon.dae',
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
  %remove_agent_visualization('forth', forth_human:'forth_human_robot1'),
  add_stickman_visualization('forth', forth_human:'forth_human_robot1', T, '', ''),
  visualize_forth_objects(T).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

pnp_object('SPOON',
           'package://unsorted/robohow/spoon3.dae',
            movable).
pnp_object('TRAY',
           'package://kitchen/cooking-vessels/tray.dae',
           movable).
pnp_object('TOMATO-SAUCE',
           'package://kitchen/food-drinks/ketchup/ketchup.dae',
           movable).

pnp_object_pose(ObjType, T, Position, Rotation) :-
  task_type(Perc,knowrob:'UIMAPerception'),
  % Is perceive earlier then T?
  task_end(Perc,_E),
  time_earlier_then(_E, T),
  % Is there a designator?
  rdf_has(Perc, knowrob:'nextAction', PostAction),
  designator_between(Obj, Perc, PostAction),
  % Has the designator the correct type?
  mng_designator(Obj, ObjJava),
  mng_designator_props(Obj, ObjJava, ['TYPE'], ObjType),
  
  mng_designator_location(Obj, Transform, T),
  matrix_rotation(Transform, Rotation),
  matrix_translation(Transform, Position).

visualize_pnp_objects(T) :-
  forall( pnp_object(ObjType, MeshPath, _), (
    (
      once(pnp_object_pose(ObjType, T, Position, Rot)),
      add_mesh(ObjType, MeshPath, Position, Rot)
    ) ; (
      remove_object(ObjType)
    )
  )).
  
visualize_pnp_speech_bubble(Agent, T) :-
  (
    rdfs_individual_of(Ev, knowrob:'SpeechAct'),
    rdf_has(Ev, knowrob:'sender', Agent),
    rdf_has(Ev, knowrob:'startTime', T0),
    rdf_has(Ev, knowrob:'endTime', T1),
    time_earlier_then(T0, T),
    time_earlier_then(T, T1),
    % Speech bubble visible
    rdf_has(Ev, knowrob:'content', literal(type(_,Text))),
    visualize_pnp_speech_bubble(Sender, Text, T)
  ) ; (
    % no speech bubble visible
    add_speech_bubble(Agent, '', [0,0,0])
  ).

visualize_pnp_speech_bubble('http://knowrob.org/kb/PR2.owl#PR2', Text, T) :-
  mng_lookup_transform('/map', '/head_pan_link', T, Transform),
  matrix_translation(Transform, [X,Y,Z]),
  Z_Offset is Z + 0.2,
  add_speech_bubble('http://knowrob.org/kb/PR2.owl#PR2', Text, [X,Y,Z_Offset]).

visualize_pnp_speech_bubble('http://knowrob.org/kb/Boxy.owl#boxy_robot1', Text, T) :-
  true.
  %mng_lookup_transform('/map', '/boxy_head_mount_kinect2_rgb_optical_frame', T, Transform),
  %matrix_translation(Transform, [X,Y,Z]),
  %Z_Offset is Z + 0.2,
  %add_speech_bubble('http://knowrob.org/kb/Boxy.owl#boxy_robot1', Text, [2,2,2]).
  

visualize_pnp_experiment(T) :-
  update_object_with_children('http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j', T),
  add_agent_visualization('PR2', pr2:'PR2Robot1', T, '', ''),
  visualize_pnp_objects(T),
  visualize_pnp_speech_bubble('http://knowrob.org/kb/PR2.owl#PR2', T),
  visualize_pnp_speech_bubble('http://knowrob.org/kb/PR2.owl#boxy_robot1', T), !.
