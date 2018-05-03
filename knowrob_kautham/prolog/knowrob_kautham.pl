/** <module> Predicates for interfacing with the Kautham planner and geometric reasoner

  Copyright (C) 2017 Daniel Beßler
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
@license BSD
*/
:- module(knowrob_kautham,
    [
        battat_initialize_kautham_sim/0,
        comp_affordanceocclusion/3,
        kautham_init_planning_scene/2,
        kautham_grab_part/4,
        kautham_put_part/5,
        perform_assembly_action/2,
        perform_put_away/2
    ]).


:- use_foreign_library('libkauthamwrapper.so').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob_assembly')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_kautham, 'http://knowrob.org/kb/knowrob_kautham.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).

:-  rdf_meta
  comp_affordanceocclusion(?, r, r),
  kautham_init_planning_scene(r, r),
  kautham_grab_part(r, r, r, ?),
  kautham_put_part(r, r, r, r, ?).

battat_initialize_kautham_sim :- 
  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_airplane_test.owl', belief_state).
%%  owl_parser:owl_parse('package://knowrob_kautham/owl/battat_airplane_simulation.owl').

free_grasping_affordance(MobilePart, GraspingAffordance) :-
  owl_has(MobilePart, knowrob_assembly:'hasAffordance', GraspingAffordance),
  owl_individual_of(GraspingAffordance, knowrob_assembly:'GraspingAffordance'),
  \+ rdf_has(_, knowrob_assembly:'blocksAffordance', GraspingAffordance),
  \+ holds(_, knowrob_assembly:'blocksAffordance', GraspingAffordance).

grasppose(MobilePart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]) :-
  belief_at_global(MobilePart, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  transform_multiply(["map", "mobilepart", [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]], ["mobilepart", "gripper", [GTx, GTy, GTz], [GRx, GRy, GRz, GRw]], [_, _, [PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]).

arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], "left") :-
  kautham_call_yumi_ik("left", [PTx, PTy, PTz, PRx, PRy, PRz, PRw], Conf),
  \+ =([], Conf).

arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], "right") :-
  kautham_call_yumi_ik("right", [PTx, PTy, PTz, PRx, PRy, PRz, PRw], Conf),
  \+ =([], Conf).

get_connection_transform(Connection, [CTx, CTy, CTz, CRx, CRy, CRz, CRw], ReferenceObj) :-
  once(owl_has(Connection, knowrob_assembly:'usesTransform', ConnectionTransform)),
  assemblage_connection_reference(Connection, ConnectionTransform, ReferenceObj), 
  rdf_has(ConnectionTransform, knowrob:'translation', literal(type(_, ConnectionTranslation))),
  rdf_has(ConnectionTransform, knowrob:'quaternion', literal(type(_, ConnectionRotation))),
  rdf_vector_prolog(ConnectionTranslation, [CTx, CTy, CTz]),
  rdf_vector_prolog(ConnectionRotation, [CRx, CRy, CRz, CRw]).

put_away_manips(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result) :-
  kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, GrabResult),
  (=("ok", GrabResult) -> 
    kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(GrabResult, Result)).

get_pose_and_dist([Tx, Ty, Tz, Rx, Ry, Rz, Rw, D]) :-
  owl_instance_of(Part, knowrob_assembly:'AtomicPart'),
  belief_at_global(Part, [_, _, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]),
  %%%%%% Add dist to the rep.
  =(D, 0.15).

get_poses_and_dists(PartPosesAndDists) :-
  findall(Pd, get_pose_and_dist(Pd), PartPosesAndDists).

perform_put_away(ActionDescription, Result) :-
  %% =(ActionDescription, ["an", "action", ["type", "putting_part_away"], ["mobile-part", MobilePart]]),
  rdf_has(ActionDescription, knowrob_assembly:'movePart', MobilePart),
  free_grasping_affordance(MobilePart, GraspingAffordance),
  rdf_has(GraspingAffordance, knowrob_assembly:'graspAt', GraspSpecification),
  grasppose(MobilePart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]),
  arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], ArmName),
  %%%%%% Define table surface limits [fwd back left right height] and object height;
  =(TableLimits, [0.6, 0, -0.5, 0.5, 0]),
  =(PartHeight, 0.1),
  get_poses_and_dists(PartPosesAndDists),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  kautham_find_away_pose(ArmName, TableLimits, PartHeight, [GTx, GTy, GTz, GRx, GRy, GRz, GRw], [PRx, PRy, PRz, PRw], PartPosesAndDists, PartGlobalTargetPose, FindResult),
  ((=("ok", FindResult) ->
    put_away_manips(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(FindResult, Result))),
  =("ok", Result).

perform_assembly_action(ActionDescription, Result) :-
  %% =(ActionDescription, ["an", "action", ["type", "connecting"], ["connection", Connection], ["fixed-part", _], ["mobile-part", MobilePart]]),
  rdf_has(ActionDescription, knowrob_assembly:'mobilePart', MobilePart),
  rdf_has(ActionDescription, knowrob_assembly:'assembledConnection', Connection),
  assemblage_possible_grasp(MobilePart, Connection, [GraspPart, GraspingAffordance, GraspSpecification]),
  grasppose(GraspPart, GraspSpecification, [PTx, PTy, PTz, PRx, PRy, PRz, PRw]),
  arm_available([PTx, PTy, PTz, PRx, PRy, PRz, PRw], ArmName),
  get_connection_transform(Connection, [CTx, CTy, CTz, CRx, CRy, CRz, CRw], ReferenceObj),
  belief_at_global(ReferenceObj, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
  transform_multiply(["map", "fixedpart", [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]], ["fixedpart", "mobilepart", [CTx, CTy, CTz], [CRx, CRy, CRz, CRw]], [_, _, [PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]),
  =(PartGlobalTargetPose, [[PTx, PTy, PTz], [PRx, PRy, PRz, PRw]]),
  kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, GrabResult),
  ((=("ok", GrabResult) -> 
    kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result);
    =(GrabResult, Result))),
  =("ok", Result).

get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]) :-
  rdf_has(GraspSpecification, knowrob_paramserver:'hasGraspTransform', GraspTransform),
  rdf_has(GraspTransform, knowrob:'translation', literal(type(_, GraspTranslation))),
  rdf_has(GraspTransform, knowrob:'quaternion', literal(type(_, GraspRotation))),
  rdf_vector_prolog(GraspTranslation, [GTx, GTy, GTz]),
  rdf_vector_prolog(GraspRotation, [GRx, GRy, GRz, GRw]).

%% comp_affordanceocclusion(?Part, +Affordance, +ArmName)
%
comp_affordanceocclusion(Part, GraspingAffordance, ArmName) :-
% Retrieve transform associated to GraspingAffordance
  rdf_has(GraspingAffordance, knowrob_assembly:'graspAt', GraspSpecification),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
% Retrieve TargetPart (associated to GraspingAffordance)
  rdf_has(TargetPart, knowrob_assembly:'hasAffordance', GraspingAffordance),
% Retrieve pose associated to TargetPart
  belief_at_global(TargetPart, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
% Call helper program: give transform to TargetPart, return a list of colliding bodies (represented as indices in planning scene)
  kautham_blocking_objects([OTx, OTy, OTz, ORx, ORy, ORz, ORw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], CollidingObjectIndices, ArmName),
% retrieve Part so that it has planningSceneIndex in the list
  member(Index, CollidingObjectIndices),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, Index))).

part_data(Part, Mesh, Pose) :-
  object_mesh_path(Part, Mesh),
  belief_at_global(Part, [_, _, [Tx, Ty, Tz], [Rx, Ry, Rz, Rw]]),
  =(Pose, [Tx, Ty, Tz, Rx, Ry, Rz, Rw]).

assert_part_index([Part, Index]) :-
  rdf_assert(Part, knowrob_kautham:'planningSceneIndex', literal(type('http://www.w3.org/2001/XMLSchema#integer', Index))).

kautham_init_planning_scene(ModelFolder, SceneMap) :-
  kautham_init_planning_scene_internal(ModelFolder, SceneMap),
  findall(Part, (owl_individual_of(Part, knowrob_assembly:'AtomicPart')), PartList),
  list_to_set(PartList, Parts),
  findall([Part, Mesh, Pose], (member(Part, Parts), part_data(Part, Mesh, Pose)), PartData),
  kautham_add_obstacles_internal(PartData, Indices),
  maplist(assert_part_index, Indices).

kautham_grab_part(GraspingAffordance, GraspSpecification, ArmName, Result) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', GraspingAffordance),
  belief_at_global(Part, [_, _, [TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, ObjectIndex))),
  kautham_grab_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex, ArmName, Result).

kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, ArmName, Result) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', GraspingAffordance),
  =(PartGlobalTargetPose, [[TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_kautham:'planningSceneIndex', literal(type(_, ObjectIndex))),
  kautham_put_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex, ArmName, Result).

