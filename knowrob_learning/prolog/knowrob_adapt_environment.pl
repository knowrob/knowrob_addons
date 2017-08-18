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
        apply_rule_for_adapt/5
        %rule_dimensions_hinge_joint/5
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).

apply_rule_for_adapt(SourceAction, TargetAction, SourceDoor, TargetDoor, RuleOut) :-
    rdf_instance_from_class(knowrob:'AdaptingEpisodicMemoryData', RuleOut),
    apply_rule_for_adapt_end_effector(SourceAction, TargetAction, RuleOut),
    apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut),
    apply_rule_for_radius(SourceDoor, TargetDoor, RuleOut).

apply_rule_for_radius(SourceDoor, TargetDoor, RuleOut) :-
    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
    rdf_has(SourceDoor, knowrob:'widthOfObject', literal(type(_, SourceWidth))),
    rdf_has(TargetDoor, knowrob:'widthOfObject', literal(type(_, TargetWidth))),
    SourceWidth < TargetWidth,
    Increase is TargetWidth - SourceWidth,
    rdf_assert(RuleOut, rdf:type, knowrob:'IncreaseRadiusOfTrajectory'),
    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Increase))).

apply_rule_for_radius(SourceDoor, TargetDoor, RuleOut) :-
    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
    rdf_has(SourceDoor, knowrob:'widthOfObject', literal(type(_, SourceWidth))),
    rdf_has(TargetDoor, knowrob:'widthOfObject', literal(type(_, TargetWidth))),
    SourceWidth >= TargetWidth,
    Decrease is SourceWidth - TargetWidth,
    rdf_assert(RuleOut, rdf:type, knowrob:'DecreaseRadiusOfTrajectory'),
    rdf_assert(RuleOut, knowrob:'radius', literal(type(xsd:'float', Decrease))).

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
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateTrajectoryInY'),
    rdf_assert(RuleOut, knowrob:'turnDegreeTrajectory', literal(type(xsd:'float', 180))).

apply_rule_for_adapt_trajectory(SourceAction, TargetAction, RuleOut) :-
    owl_individual_of(SourceAction, knowrob:'OpeningAFridgeDoorCW'),
    owl_individual_of(TargetAction, knowrob:'OpeningAFridgeDoorCCW'),
    rdf_assert(RuleOut, rdf:type, knowrob:'RotateTrajectoryInY'),
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

check_handle_type(Door, Tsk) :-
    rdf_has(Door, srdl2comp:'succeedingJoint', Handle),
    rdf_has(Handle, knowrob:'widthOfObject', literal(type(_, W))),
    rdf_has(Handle, knowrob:'heightOfObject', literal(type(_, H)))
    (((W >= H),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeGripperParallel'));
     ((H > W),
      rdf_assert(Tsk, rdf:type, knowrob:'OpeningAFridgeGripperPerpendicular'))).
 

estimate_action_by_comparing(EpisodicMemoryTask, SourceDoor, TargetDoor, TargetAction) :-
    owl_individual_of(SourceDoor, knowrob:'IAIFridgeDoor'),
    rdf_has(SourceDoor, knowrob:'describedInMap', SourceMap),
    owl_individual_of(SourceHandle, knowrob:'IAIFridgeDoorHandle'),
    rdf_has(SourceHandle, knowrob:'describedInMap', SourceMap),
    owl_individual_of(TargetDoor, knowrob:'IAIFridgeDoor'),
    rdf_has(TargetDoor, knowrob:'describedInMap', TargetMap),
    owl_individual_of(TargetHandle, knowrob:'IAIFridgeDoorHandle'),
    rdf_has(TargetHandle, knowrob:'describedInMap', TargetMap),
    rdf_instance_from_class(knowrob:'OpeningAFridgeDoorGeneric', TargetAction),
    check_joint_type(SourceDoor, EpisodicMemoryTask),
    check_joint_type(TargetDoor, TargetAction),
    check_handle_type(SourceDoor, EpisodicMemoryTask),
    check_joint_type(TargetDoor, TargetAction).
