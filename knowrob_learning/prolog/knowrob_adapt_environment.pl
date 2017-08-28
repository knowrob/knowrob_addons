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
        %generic_adapt/3,
        estimate_action_by_comparing/4,
        apply_rule_for_adapt/3
        %rule_dimensions_hinge_joint/5
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).

apply_rule_for_adapt(SourceAction, TargetAction, RuleOut) :-
    rdf_instance_from_class(knowrob:'AdaptingEpisodicMemoryData', RuleOut),
    apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut),
    apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut),
    apply_rule_for_radius(SourceAction, TargetAction, RuleOut).

radius_measure_action(Action, Radius) :-
    (rdf_has(Action, knowrob:'openingTrajectory', Trj); rdf_has(SourceAction, knowrob:'closingTrajectory', Trj)),
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
    ((class_properties(Cls, knowrob:'openingTrajectory', Trj), rdf_assert(Tsk, knowrob:'openingTrajectory', TrjInst));
     (class_properties(Cls, knowrob:'closingTrajectory', Trj), rdf_assert(Tsk, knowrob:'closingTrajectory', TrjInst))),
    rdf_assert(TrjInst, rdf:type, Trj),
    class_properties(Trj, knowrob:'center', HJoint),
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
    rdf_instance_from_class(knowrob:'OpeningAFridgeDoorGeneric', TargetAction),
    check_joint_type(SourceDoor, EpisodicMemoryTask),
    check_joint_type(TargetDoor, TargetAction),
    check_handle_type(SourceHandle, EpisodicMemoryTask),
    check_handle_type(TargetHandle, TargetAction),
    check_trajectory_props(EpisodicMemoryTask, SourceDoor), 
    check_trajectory_props(TargetAction, TargetDoor),
    rdf_assert(TargetAction, rdf:type, knowrob:'IntentionalMentalEvent').
