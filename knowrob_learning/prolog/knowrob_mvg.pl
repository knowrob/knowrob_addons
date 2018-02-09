/** <module> knowrob_mvg

  Copyright (C) 2016-17 Asil Kaan Bozcuoglu

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

:- module(knowrob_mvg,
    [
      generate_mixed_gaussian/1,
      generate_mixed_gaussian/2,
      mixed_gaussian_with_failure/6,
      get_likely_location/6,
      get_likely_pose/7,
      generate_multivar_gaussian/2,
      generate_heat_maps/0,
      generate_feature_files/2,
      get_divided_subtasks_with_goal/4
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('knowrob/transforms')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).


generate_multivar_gaussian(InFile, OutFile) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'createMultiVarGaussians', [InFile, OutFile], _X).

generate_mixed_gaussian(InFile, OutFile) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'analyzeCluster', [InFile, OutFile], _X).  

mixed_gaussian_with_failure(PosFile, PosCluster, NegFile, NegCluster, OutFile, Pose) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'analyzeTrials', [PosFile, NegFile, OutFile, PosCluster, NegCluster], X),
  jpl_array_to_list(X, Pose). 

get_likely_location(PosFile, PosCluster, NegFile, NegCluster, Mean, Covariance) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'likelyLocationClosest', [PosFile, NegFile, PosCluster, NegCluster], X),
  jpl_array_to_list(X, [MX, MY, C1, C2, C3, C4]),
  Mean = [MX, MY],
  Covariance = [C1, C2, C3, C4].  

get_likely_pose(PosFile, PosCluster, NegFile, NegCluster, CurrentPose, Pose, Covariance) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  %jpl_list_to_array(CurrentPose, CurrentPoseArr),
  jpl_call(GausInterface, 'likelyLocationClosest', [PosFile, NegFile, PosCluster, NegCluster], X),
  jpl_array_to_list(X, [MX, MY, C1, C2, C3, C4]),
  matrix(CurrentPose, [CP_X,CP_Y,_], [0,0,0,1]),
  matrix_translate(CurrentPose, [MX-CP_X , MY-CP_Y,0], Pose),
  Covariance = [C1, C2, C3, C4].

generate_mixed_gaussian(OutFile) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'createMixedGaussians', [OutFile], _X).

generate_heat_maps :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'createHeatMaps', [], _X).

generate_feature_files(Features, FileName) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'writeSimpleFeatures', [Features, FileName], _X).

generate_feature_files(FloatFeatures, StringFeatures) :-
  jpl_new('org.knowrob.gaussian.MixedGaussianInterface', [], GausInterface),
  jpl_call(GausInterface, 'writeFeature', [FloatFeatures, StringFeatures], _X).

get_divided_subtasks_with_goal(Parent, Goal, SuccInst, NegInsts) :-
  findall(ST, (subtask(Parent, ST),
  entity(ST, [an, action, ['task_context', Goal]])), AllInstances),
  list_to_set(AllInstances, AllSet),
  member(SuccInst, AllSet),
  subtract(AllSet, [SuccInst], NegInsts),
  occurs(SuccInst, [SuccBegin, _]),
  forall(member(A,NegInsts), (occurs(A, [Begin, _]), SuccBegin > Begin)). 
