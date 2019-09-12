/** <module> knowrob_adapt_environment

  Copyright (C) 2017 Asil Kaan Bozcuoglu
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

  @author Asil Kaan Bozcuoglu
  @license BSD
*/

:- module(knowrob_adapt_environment,
    [
        estimate_action_by_comparing/4,
        apply_rule_for_adapt/3,
        project_link_for_arch_traj/5,
        project_arch_trajectory_samples/6,
        sample_trajectory/4,
        sample_trajectory/5,
        visualize_projected_traj/1,
        visualize_traj_samples/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/transforms')).

apply_rule_for_adapt(SourceAction, TargetAction, RuleOut) :-
    rdf_instance_from_class(knowrob:'AdaptingEpisodicMemoryData', RuleOut),
    apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut),
    apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut),
    apply_rule_for_radius(SourceAction, TargetAction, RuleOut).

radius_measure_action(Action, Radius) :-
    (rdf_has(Action, knowrob:'openingTrajectory', Trj); rdf_has(Action, knowrob:'closingTrajectory', Trj)),
    rdf_has(Trj, knowrob:'radius', literal(type(_, RadiusA))),
    atom_number(RadiusA, Radius).

apply_rule_for_radius(SourceAction, TargetAction, RuleOut) :-
    radius_measure_action(SourceAction, SourceRadius),
    radius_measure_action(TargetAction, TargetRadius),
    SourceRadius < TargetRadius,
    Increase is TargetRadius - SourceRadius,
    rdf_assert(RuleOut, rdf:type, knowrob:'IncreaseRadiusOfTrajectory'),
    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Increase))).

apply_rule_for_radius(SourceAction, TargetAction, RuleOut) :-
    radius_measure_action(SourceAction, SourceRadius),
    radius_measure_action(TargetAction, TargetRadius),
    SourceRadius >= TargetRadius,
    Decrease is SourceRadius - TargetRadius,
    rdf_assert(RuleOut, rdf:type, knowrob:'DecreaseRadiusOfTrajectory'),
    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Decrease))).

%apply_rule_for_radius(SourceDoor, TargetDoor, RuleOut) :-
%    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
%    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
%    rdf_has(SourceDoor, knowrob:'widthOfObject', literal(type(_, SourceWidth))),
%    atom_number(SourceWidth, SourceWidthN),
%    rdf_has(TargetDoor, knowrob:'widthOfObject', literal(type(_, TargetWidth))),
%    atom_number(TargetWidth, TargetWidthN),
%    SourceWidthN < TargetWidthN,
%    Increase is TargetWidthN - SourceWidthN,
%    rdf_assert(RuleOut, rdf:type, knowrob:'IncreaseRadiusOfTrajectory'),
%    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Increase))).

%apply_rule_for_radius(SourceDoor, TargetDoor, RuleOut) :-
%    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
%    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
%    rdf_has(SourceDoor, knowrob:'widthOfObject', literal(type(_, SourceWidth))),
%    atom_number(SourceWidth, SourceWidthN),
%    rdf_has(TargetDoor, knowrob:'widthOfObject', literal(type(_, TargetWidth))),
%    atom_number(TargetWidth, TargetWidthN),
%    SourceWidthN >= TargetWidthN,
%    Decrease is SourceWidthN - TargetWidthN,
%    rdf_assert(RuleOut, rdf:type, knowrob:'DecreaseRadiusOfTrajectory'),
%    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Decrease))).

apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeGripperPerpendicular'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeGripperParallel'),
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateEndEffector'),
    rdf_assert(RuleOut, knowrob:'turnDegreeEndEffector', literal(type(xsd:'float', 90))).

apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeGripperParallel'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeGripperPerpendicular'),
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateEndEffector'),
    rdf_assert(RuleOut, knowrob:'turnDegreeEndEffector', literal(type(xsd:'float', 90))).

apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeGripperPerpendicular'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeGripperPerpendicular'),
    rdf_assert(RuleOut, rdf:type, knowrob:'NullChangeEndEffector').

apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeGripperParallel'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeGripperParallel'),
    rdf_assert(RuleOut, rdf:type, knowrob:'NullChangeEndEffector').

apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeDoorCCW'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeDoorCW'),
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateTrajectoryInX'),
    rdf_assert(RuleOut, knowrob:'turnDegreeTrajectory', literal(type(xsd:'float', 180))).

apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeDoorCW'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeDoorCCW'),
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateTrajectoryInX'),
    rdf_assert(RuleOut, knowrob:'turnDegreeTrajectory', literal(type(xsd:'float', 180))).

apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeDoorCW'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeDoorCW'),
    rdf_assert(RuleOut, rdf:type, knowrob:'NullChangeTrajectory').

apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeDoorCCW'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeDoorCCW'),
    rdf_assert(RuleOut, rdf:type, knowrob:'NullChangeTrajectory').

check_joint_type(Door, Tsk) :-
    rdf_has(Door, knowrob:'doorHingedWith', Hinge),
    ((rdf_has(Hinge, knowrob:'openingDirection', literal(type(_, 'CCW'))),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeDoorCCW'));
     (rdf_has(Hinge, knowrob:'openingDirection', literal(type(_, 'CW'))),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeDoorCW'))).

check_handle_type(Handle, Tsk) :-
    rdf_has(Handle, knowrob:'widthOfObject', literal(type(_, W))),
    atom_number(W, WN),
    rdf_has(Handle, knowrob:'heightOfObject', literal(type(_, H))),
    atom_number(H, HN),
    (((WN >= HN),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeGripperParallel'));
     ((HN > WN),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeGripperPerpendicular'))).

check_trajectory_props(Tsk, Door) :-
    owl_individual_of(Tsk, Cls),
    rdf_instance_from_class(knowrob:'ArmTrajectory', TrjInst),
    ((owl_class_properties(Cls, knowrob:'openingTrajectory', Trj), rdf_assert(Tsk, knowrob:'openingTrajectory', TrjInst));
     (owl_class_properties(Cls, knowrob:'closingTrajectory', Trj), rdf_assert(Tsk, knowrob:'closingTrajectory', TrjInst))),
    rdf_assert(TrjInst, rdf:type, Trj),
    owl_class_properties(Trj, knowrob:'center', HJoint),
    owl_individual_of(HJInst, HJoint),
    rdf_has(Door, _Y, HJInst),
    rdf_assert(TrjInst, knowrob:'center', HJInst),
    rdf_has(Door, knowrob:'widthOfObject', R),
    rdf_assert(TrjInst, knowrob:'radius', R).

find_handle_of_doors(Door, Handle) :-
    owl_individual_of(Door, knowrob:'IAIFridgeDoor'),
    (rdf_has(Door, 'http://knowrob.org/kb/srdl2-comp.owl#succeedingJoint', Handle);
    (rdf_has(Frd, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent', Door),
     rdf_has(Frd, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent', Handle),
     owl_individual_of(Handle, knowrob:'IAIFridgeDoorHandle'))).

estimate_action_by_comparing(EpisodicMemoryTask, SourceDoor, TargetDoor, TargetAction) :-
    entity(EpisodicMemoryTask, [an, action, ['task_context', 'ReachAndOpenFridgeDoor']]), 
    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
    find_handle_of_doors(SourceDoor, SourceHandle),
    find_handle_of_doors(TargetDoor, TargetHandle),
    rdf_instance_from_class(knowrob:'OpeningAFridgeDoorGeneric', TargetAction),
    check_joint_type(SourceDoor, EpisodicMemoryTask),
    check_joint_type(TargetDoor, TargetAction),
    check_handle_type(SourceHandle, EpisodicMemoryTask),
    check_handle_type(TargetHandle, TargetAction),
    check_trajectory_props(EpisodicMemoryTask, SourceDoor), 
    check_trajectory_props(TargetAction, TargetDoor),
    rdf_assert(TargetAction, rdf:type, knowrob:'IntentionalMentalEvent').

estimate_action_by_comparing(EpisodicMemoryTask, SourceDoor, TargetDoor, TargetAction) :-
    entity(EpisodicMemoryTask, [an, action, ['task_context', 'CloseFridgeDoor']]),
    find_handle_of_doors(SourceDoor, SourceHandle),
    find_handle_of_doors(TargetDoor, TargetHandle),
    rdf_instance_from_class(knowrob:'ClosingAFridgeDoorGeneric', TargetAction),
    check_joint_type(SourceDoor, EpisodicMemoryTask),
    check_joint_type(TargetDoor, TargetAction),
    check_handle_type(SourceHandle, EpisodicMemoryTask),
    check_handle_type(TargetHandle, TargetAction),
    check_trajectory_props(EpisodicMemoryTask, SourceDoor), 
    check_trajectory_props(TargetAction, TargetDoor),
    rdf_assert(TargetAction, rdf:type, knowrob:'IntentionalMentalEvent').

sample_trajectory(Task, Link, Samples, StepSize) :-
   occurs(Task, [Begin,End]),
   sample_trajectory(Begin, End, Link, Samples, StepSize).

sample_trajectory(Start, End, Link, Samples, StepSize) :-
   End >= Start,
   NewStart is Start + StepSize, 
   sample_trajectory(NewStart, End, Link, RestSamples, StepSize),
   mng_lookup_transform('/map', Link, Start, Pose),
   matrix(Pose, Pos, Rot),
   append([[Pos, Rot]], RestSamples, Samples).

sample_trajectory(Start, End, _Link, Samples, _StepSize) :-
   End < Start, Samples = [], true.

project_link_for_arch_traj(Time, Link, Door, Rule, ProjectedPose) :-
    mng_lookup_transform('/map', Link, Time, Pose),
    rdf_has(Door, knowrob:'doorHingedWith', Joint),
    rdf_has(Door, knowrob:'widthOfObject', literal(type(_, Width))),
    atom_number(Width, Radius),
    ((owl_individual_of(Rule, knowrob:'DecreaseRadiusOfTrajectory'), rdf_has(Rule, knowrob:'radius', literal(type(_, Change))),
        NewRadius is Radius - Change);
    (owl_individual_of(Rule, knowrob:'IncreaseRadiusOfTrajectory'), rdf_has(Rule, knowrob:'radius', literal(type(_, Change))),
        NewRadius is Radius + Change)),
    Ratio is NewRadius / Radius,
    object_pose_at_time(Joint, Time, mat(JointPose)),
    matrix(Pose, [X,Y,Z], Rot),
    matrix(JointPose, [X_J, Y_J, _JZ], _),
    Delta_X is X - X_J,
    Delta_Y is Y - Y_J,
    Diff_X is Delta_X * Ratio,
    Diff_Y is Delta_Y * Ratio,
    Projected_X is Diff_X + X_J,
    Projected_Y is Diff_Y + Y_J,
    ProjectedPosition = [Projected_X, Projected_Y, Z],
    ProjectedPose = [ProjectedPosition, Rot].

project_arch_trajectory_samples(Task, Link, Door, Rule, ProjectedPose, StepSize) :-
   occurs(Task, [Begin,End]),
   project_arch_trajectory_samples(Begin, End, Link, Door, Rule, ProjectedPose, StepSize).

project_arch_trajectory_samples(Start, End, Link, Door, Rule, ProjectedPose, StepSize) :-
   End >= Start,
   NewStart is Start + StepSize, 
   project_arch_trajectory_samples(NewStart, End, Link, Door, Rule, RestPoseLists, StepSize),
   project_link_for_arch_traj(Start, Link, Door, Rule, Curr),
   append([Curr], RestPoseLists, ProjectedPose).

project_arch_trajectory_samples(Start, End, _Link, _Door, _Rule, ProjectedPose, _StepSize) :-
   Start > End, ProjectedPose =[],  true.

visualize_projected_traj(Traj):-
   forall(member(Sample, Traj),
          (
            nth0(0, Sample, Position),
            nth0(1, Sample, Rot),
            rdf_instance_from_class('Traj', Id),
            show(cube(Id), [ pose(Position,Rot),scale([0.02,0.02,0.02]),color([1.0,1.0,0.0])])
          )).
visualize_traj_samples(Traj):-
    visualize_projected_traj(Traj).
