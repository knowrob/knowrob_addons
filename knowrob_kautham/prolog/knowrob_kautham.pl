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
        comp_affordanceocclusion/2,
        kautham_init_planning_scene/2,
        kautham_grab_part/3,
        kautham_put_part/4,
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
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).

:-  rdf_meta
  comp_affordanceocclusion(?, r),
  kautham_init_planning_scene(r, r),
  kautham_grab_part(r, r, r),
  kautham_put_part(r, r, r, r).

get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]) :-
  rdf_has(GraspSpecification, knowrob_paramserver:'hasGraspTransform', GraspTransform),
  rdf_has(GraspTransform, knowrob:'translation', literal(type(_, GraspTranslation))),
  rdf_has(GraspTransform, knowrob:'quaternion', literal(type(_, GraspRotation))),
  rdf_vector_prolog(GraspTranslation, [GTx, GTy, GTz]),
  rdf_vector_prolog(GraspRotation, [GRx, GRy, GRz, GRw]).

%% comp_affordanceocclusion(?Part, +Affordance)
%
comp_affordanceocclusion(Part, GraspingAffordance) :-
% Retrieve transform associated to GraspingAffordance
  rdf_has(GraspingAffordance, knowrob_assembly:'graspAt', GraspSpecification),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
% Retrieve TargetPart (associated to GraspingAffordance)
  rdf_has(TargetPart, knowrob_assembly:'hasAffordance', GraspingAffordance),
% Retrieve pose associated to TargetPart
  belief_at(TargetPart, [_, _, [OTx, OTy, OTz], [ORx, ORy, ORz, ORw]]),
% Call helper program: give transform to TargetPart, return a list of colliding bodies (represented as indices in planning scene)
  kautham_blocking_objects([OTx, OTy, OTz, ORx, ORy, ORz, ORw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], CollidingObjectIndices),
% retrieve Part so that it has planningSceneIndex in the list
  member(Index, CollidingObjectIndices),
  rdf_has(Part, knowrob_assembly:'planningSceneIndex', literal(type(_, Index))).

kautham_init_planning_scene(ModelFolder, SceneMap) :-
  kautham_init_planning_scene_internal(ModelFolder, SceneMap).

kautham_grab_part(GraspingAffordance, GraspSpecification, Result) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', GraspingAffordance),
  belief_at(Part, [_, _, [TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_assembly:'planningSceneIndex', literal(type(_, ObjectIndex)))
  kautham_grab_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex).

kautham_put_part(GraspingAffordance, GraspSpecification, PartGlobalTargetPose, Result) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', GraspingAffordance),
  equal(PartGlobalTargetPose, [[TTx, TTy, TTz], [TRx, TRy, TRz, TRw]]),
  get_grasp_transform(GraspSpecification, [GTx, GTy, GTz, GRx, GRy, GRz, GRw]),
  rdf_has(Part, knowrob_assembly:'planningSceneIndex', literal(type(_, ObjectIndex)))
  kautham_put_part_internal([TTx, TTy, TTz, TRx, TRy, TRz, TRw], [GTx, GTy, GTz, GRx, GRy, GRz, GRw], ObjectIndex).

