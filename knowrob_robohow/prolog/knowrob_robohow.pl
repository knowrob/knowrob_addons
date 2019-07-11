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
      visualize_rolling_experiment/1,
      show_action_trajectory/1,
      get_dynamics_image_perception/2,
      get_sherlock_image_perception/2,
      get_reach_action/2,
      get_roll_action/2,
      get_retract_action/2
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/comp_temporal')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/srdl2')).
:- use_module(library('knowrob_meshes')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(forth_human, 'http://knowrob.org/kb/forth_human.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(boxy2, 'http://knowrob.org/kb/BoxyWithRoller.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pr2, 'http://knowrob.org/kb/PR2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

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
  matrix(Transform, [X,Y,Z], _),
  matrix(TransformParent, [X_Parent,Y_Parent,Z_Parent], _),
  % Estimate rotation based on joint positions
  Dir_X is X - X_Parent,
  Dir_Y is Y - Y_Parent,
  Dir_Z is Z - Z_Parent,
  jpl_list_to_array([Dir_X, Dir_Y, Dir_Z], DirArr),
  fail,
  % FIXME
  %jpl_call('org.knowrob.utils.MathUtil', 'orientationToQuaternion', [DirArr], QuaternionArr),
  jpl_array_to_list(QuaternionArr, Rotation).
  

designator_grasped_pose(Grasp, ObjId, T, Position, Rotation) :-
  % TODO(daniel): What if right and left hand are used?
  %     - Translation: use center of both joints
  %     - Rotation: ....
  % TODO(daniel): Object might not be grasped at center of object
  %     - Use knowledge about graspable object parts
  %     - Use planner knowledge where the grasp was done
  once(rdf_has(Grasp, knowrob:'bodyPartsUsed', BodyPart)),
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
  quaternion_diff(Rh_0_Arr, Rh_T_Arr, Rh_Diff_Arr),
  
  % Apply quaternion difference to Ro_0
  jpl_list_to_array(Ro_0, Ro_0_Arr),
  quaternion_multiply(Ro_0_Arr, Rh_Diff_Arr, Rot_Arr),
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
  matrix(Transform, Position, Rotation).

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
  rdf_has(ActionID, knowrob:'subAction', WithDesig),
  findall(Sub,rdf_has(WithDesig, knowrob:'subAction', Sub),Subs),
  nth0(0, Subs, ReachID).
  
get_roll_action(ActionID, RollID):-
  rdf_has(ActionID, knowrob:'subAction', WithDesig),
  findall(Sub,rdf_has(WithDesig, knowrob:'subAction', Sub),Subs),
  nth0(1, Subs, RollID).
  
get_retract_action(ActionID, RetractID):-
  rdf_has(ActionID, knowrob:'subAction', WithDesig),
  findall(Sub,rdf_has(WithDesig, knowrob:'subAction', Sub),Subs),
  nth0(2, Subs, RetractID).
  
show_action_trajectory(ActionID):-
  clear_trajectories,
  task_start(ActionID,StAction),
  task_end(ActionID,EAction),
  marker_update(trajectory('right_arm_adapter_kms40_fwk050_frame_out'), interval(StAction, EAction, dt(0.2))),
  marker_update(agent(boxy2:'boxy_robot2'), StAction).


  
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


% FIXME: is never called because
%	- knowrob_marker implements more general case object(Obj)
%         -> this must fail because no pose can be obtained
%	- there is a generic case that just iterates all childs
%	  -> fail if no children?
%% special vis hook for the dough
knowrob_marker:marker_new(MarkerName,
    contour('http://knowrob.org/kb/IAI-kitchen.owl#Dough_fngh257tgh'),
    MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName,
      contour('http://knowrob.org/kb/IAI-kitchen.owl#Dough_fngh257tgh'),
      MarkerObject, Parent),
  marker_color(MarkerObject, [0.6,0.6,0.2,1.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

knowrob_marker:marker_update(
    contour('http://knowrob.org/kb/IAI-kitchen.owl#Dough_fngh257tgh'), 
    MarkerObject, Instant) :-
  % FIXME: hide dough if no desig could be found
  %	- could be done one level above (i.e., hide markers for which marker_update failed)
  mng_query_latest('logged_designators', one(DBObj),
    '__recorded', Instant, [['designator.DOUGH', 'exist', 'true']]),
  mng_designator(DBObj, Desig),
  contour_mesh_extends(Desig, ['DOUGH', 'CONTOUR'],
    [[Min_x,Min_y,Min_z], [Max_x,Max_y,Max_z]]),
  % compute scaling
  Scale_x is Max_x - Min_x,
  Scale_y is Max_y - Min_y,
  Scale_z is Max_z - Min_z,
  marker_scale(MarkerObject, [Scale_x,Scale_y,Scale_z]),
  % compute translation
  Offset_x is 0.5*(Max_x + Min_x),
  Offset_y is 0.5*(Max_y + Min_y),
  Offset_z is 0.5*(Max_z + Min_z),
  marker_translation(MarkerObject, [Offset_x,Offset_y,Offset_z]).


visualize_rolling_experiment(T) :-
  marker_update(agent(boxy2:'boxy_robot2'), T),
  mng_query_latest('logged_designators', one(DBObj),
    '__recorded', T, [['designator.DOUGH', 'exist', 'true']]),
  mng_designator(DBObj, Desig),
  % TODO: Interface for knowrob_meshes !!!
  add_designator_contour_mesh('DOUGH', Desig, [0.6,0.6,0.2], ['DOUGH', 'CONTOUR']).
  
  %add_agent_visualization('BOXY', boxy2:'boxy_robot2', T, '', ''),
  %mng_query_latest('logged_designators', one(DBObj),
  %  '__recorded', T, [['designator.DOUGH', 'exist', 'true']]),
  %mng_designator(DBObj, Desig),
  %add_designator_contour_mesh('DOUGH', Desig, [0.6,0.6,0.2], ['DOUGH', 'CONTOUR']).
