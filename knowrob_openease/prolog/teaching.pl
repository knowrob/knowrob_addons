/** <module> openease_video

  Copyright (C) 2016 Asil Kaan Bozcuoglu
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

:- module(teaching,
    [
        link_pose_with_respect_to_preceding/3,
        link_pose_with_respect_to_other/4,
        distance_from_pose_matrix/2
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
.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


link_pose_with_respect_to_preceding(SourceLink, TargetLink, Transform) :-
  succeeding_link(SourceLink, TargetLink),
  owl_individual_of(Experiment, knowrob:'RobotExperiment'),
  rdf_has(Experiment, knowrob:'endTime', Time),
  link_pose_with_respect_to_other(SourceLink, TargetLink, Time, Transform).

link_pose_with_respect_to_other(SourceLink, TargetLink, Time, Transform) :- 
  rdf_has(SourceLink, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(SourceUrdf)),
  rdf_has(TargetLink, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(TargetUrdf)),
  mng_lookup_transform(TargetUrdf, SourceUrdf, Time, Transform).

distance_from_pose_matrix(Pose, Distance) :-
  matrix_translation(Pose, [X, Y, Z]),
  XSq is X * X,
  YSq is Y * Y,
  ZSq is Z * Z,
  DistanceXYSq is XSq + YSq,
  DistanceSq is DistanceXYSq + ZSq,
  Distance is sqrt(DistanceSq).
