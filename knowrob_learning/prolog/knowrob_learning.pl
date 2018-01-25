/** <module> knowrob_learning

  Copyright (C) 2016 Asil Kaan Bozcuoglu
  Based on classifier.pl implemented by Moritz Tenorth and Jakob Angel (2010)
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

:- module(knowrob_learning,
    [
      select_features/4,
      train_classifier/4,
      classify_data/3,
      save_classifier/2,
      save_selector/2,
      get_negative_instances_of_tasktype/2,
      get_positive_instances_of_tasktype/2,
      featurize_ar_places/3,
      featurize_ar_place/3,
%      featurize_gaussian_places/3,
%      featurize_gaussian_place/3,
      featurize_gaussian_places/3,
      featurize_gaussian_place/3,
      color_directed_trajectory/4
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('knowrob/utility/jpl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).



% select_features(-FilteredInstances, +Algorithm, +Options, +Trainingsdata) is nondet.
%
% Interface to the different feature selection algorithms
%
% Filters out unrelated feature set out of instances
%
% Example: classifier_trained(FilteredInstances, 'weka.selection.relieff', [], [['ClassA', 3, 5] , ['ClassB', 1, 9], ...]).
%
% @param FilteredInstances     The resulting instances as Java Object
% @param Algorithm          Name of the selection algorithm, e.g. 'weka.selection.relieff'
% @param Options        Options that are passed to the selection algorithm
% @param Trainingsdata  Data the filter is to be applied

select_features(FilteredInstances, Algorithm, Options, Trainingsdata) :-
  prepare_data(Trainingsdata, T1),
  check_weka_data(T1),
  lists_to_arrays(T1, Arg),
  list_to_array(Options, OptionsArray),
  jpl_call('org.knowrob.learning.AttributeSelector', 'getASelector', [Algorithm, OptionsArray, Arg], Selector),
  jpl_call(Selector, 'applyFeatureSelection', [], FilteredInstances).
  

% train_classifier(-TrainedClassifier, +Algorithm, +Options, +Instances) is nondet.
%
% Interface to the different classifiers
%
% Filters out unrelated feature set out of instances
%
% Example: classifier_trained(FilteredInstances, 'weka.classifier.svm', [], Instances).
%
% @param TrainedClassifier     The resulting classifier instance as Java Object
% @param Options        Options that are passed to the selection algorithm
% @param Instances      Data that will train the classifier

train_classifier(NullClassifier, TrainedClassifier, Options, Instances) :-
  jpl_call('org.knowrob.learning.KnowrobClassifier', 'setAClassifier', [NullClassifier, Options, Instances], TrainedClassifier).


%% classify_instances(+ Classifier, + Instances, - Classified) is nondet.
%
% 
%
% Classifies a list of instances, using a previously trained Clusterer / Classifier.
%
% Returns a list of the assigned classes in Classified.
%
% Example: classify_instances(C, [[3,b],[9,a]], R).
%          -> R = [classA,classB]
%
% @param Classifier   Classifier instance that is to be used for classification
% @param Instances    Data to be classified
% @param Classified   Resulting class/cluster assignment: [classA,classB]

classify_data(Classifier, Data, Classified) :-
  %prepare_data(Data, I1),
  %check_weka_data(I1),
  %lists_to_arrays(I1, DataArr),
  jpl_call(Classifier, classify, Data, ClassifiedArr),
  arrays_to_lists(ClassifiedArr, Classified).



%% classifier_saved(?File,?Classifier) is nondet.
%
% Interface to the Weka machine learning library
%
% Loads / saves a classifier that has been created via classifier_trained to a specified file.
% Loading: classifier_saved(+File,-Classifier)
% Saving:  classifier_saved(+File,+Classifier)
%
% @param File       File name the classifier is to be saved to/loaded from
% @param Classifier The classifier instance to be saved or loaded

save_classifier(File, Classifier) :-
  (var(Classifier), print('Loading Classifier...'), jpl_call('org.knowrob.learning.Classifier','loadFromFile',[File],Classifier)) ;
  (nonvar(Classifier), print('Saving Classifier...'),jpl_call(Classifier,'saveToFile',[File],_)).


%% selector_saved(?File,?Classifier) is nondet.
%
% Interface to the Weka machine learning library
%
% Loads / saves a classifier that has been created via classifier_trained to a specified file.
% Loading: classifier_saved(+File,-Selector)
% Saving:  classifier_saved(+File,+Selector)
%
% @param File       File name the classifier is to be saved to/loaded from
% @param Selector The classifier instance to be saved or loaded

save_selector(File, Selector) :-
  (var(Selector), print('Loading Selector...'), jpl_call('org.knowrob.learning.AttributeSelector','loadFromFile',[File],Selector)) ;
  (nonvar(Selector), print('Saving Selector...'),jpl_call(Selector,'saveToFile',[File],_)).



% prepare_data(+Data, -Result)
% takes (multiple) lists, and
% - replaces every numeric values with the corresponding string.
% - removes literal(type(_,Value))
%
  prepare_data(X,Y) :- number(X), string_to_atom(X,Y), !.
  prepare_data(X,X) :- atom(X), !.
  prepare_data(literal(type(_,X)),Y) :- prepare_data(X,Y), !.
  prepare_data([],[]).
  prepare_data([A1|ARest],[B1|BRest]) :- prepare_data(A1,B1), prepare_data(ARest, BRest).


% check_weka_data(+Data)
% succeeds if Data is a non-empty List of equally long, non-empty lists of atoms.
%
  check_weka_data([F | Rest]) :- length(F, X), >(X,0), atom_list_length(F, X), check_all(Rest, X).
  atom_list_length([],0).
  atom_list_length([H|T],L) :- atom(H), >(L,0), LN is L-1, atom_list_length(T,LN).
  check_all([],_).
  check_all([H|T],L) :- atom_list_length(H,L), check_all(T,L).


get_negative_instances_of_tasktype(TaskClass, TaskList) :-
  findall(TaskID,(
     get_a_instance_of_tasktype(TaskClass, 'false', TaskID)
  ), TaskList).

get_positive_instances_of_tasktype(TaskClass, TaskList) :-
  findall(TaskID,(
     get_a_instance_of_tasktype(TaskClass, 'true', TaskID)
  ), TaskList).

get_a_instance_of_tasktype(TaskClass, IsSuccess, TaskInstance) :-
  owl_individual_of(TaskInstance, TaskClass),
  rdf_has(TaskInstance, knowrob:'subAction', FailureHandlingTask),
  owl_individual_of(FailureHandlingTask, 'http://knowrob.org/kb/knowrob.owl#WithFailureHandling'),
  rdf_has(FailureHandlingTask, knowrob:'taskSuccess', literal(type(_, IsSuccess))).

featurize_ar_places(TaskList, Success, FeatureList) :-
  findall( Features, (member(Task, TaskList),
     featurize_ar_place(Task, Success, Features)), FeatureList).

featurize_ar_place(Tsk, Success, Features) :-
  rdf_has(Tsk, knowrob:'designator', DAction),
  rdf_has(Tsk, knowrob:'startTime', StartTime),
  mng_designator_props(DAction, 'OBJ', DObj),
  %add_object_as_semantic_instance(DObj, Loc, StartTime, SemanticMapInstance), 
  designator_assert(SemanticMapInstance, DAction, DObj, 'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j'), !,
  designator_add_perception(SemanticMapInstance, DObj, _Matrix, StartTime),
  comp_above_of(SemanticMapInstance, TableTop),
  object_pose_at_time(TableTop, StartTime, mat(TableTopTransform)),
  object_pose_at_time(SemanticMapInstance, StartTime, mat(ObjTransform)),
  mng_lookup_transform('/map', '/base_footprint', StartTime, RobotTransform),
  pose_into_relative_coord(ObjTransform, TableTopTransform, ObjTransformTT),
  pose_into_relative_coord(RobotTransform, TableTopTransform, RobotTransformTT),
  object_dimensions(TableTop, TTDepth, TTWidth, _TTHeight),
  TTCenterX is TTDepth / 2,
  TTCenterY is TTWidth / 2,
  matrix_translation(RobotTransformTT, [RobotX, RobotY, _RobotZ]),
  DistanceRobotX is abs(RobotX),
  DistanceRobotY is abs(RobotY),
  matrix_translation(ObjTransformTT, [ObjectX, ObjectY, _ObjectZ]),
  %DistanceObjectX is abs(ObjectX),
  %DistanceObjectY is abs(ObjectY),
  matrix_rotation(ObjTransformTT, [QW, QX, QY, QZ]),
  ((
    DistanceRobotX > TTCenterX,
    DeltaRobotX is DistanceRobotX - TTCenterX,
    DeltaRobotY is DistanceRobotY,
    DeltaObjectX is abs(RobotX - ObjectX) - DeltaRobotX 
  );
  (
    DistanceRobotX =< TTCenterX,
    DeltaRobotX is DistanceRobotY - TTCenterY,
    DeltaRobotY is DistanceRobotX,
    DeltaObjectX is abs(RobotY - ObjectY) - DeltaRobotY
  )),
  DeltaObjectAngle is atan2((2.0*(QY*QZ + QW*QX)), (QW*QW - QX*QX - QY*QY + QZ*QZ)),
  Features = [DeltaRobotX, DeltaRobotY, DeltaObjectX, DeltaObjectAngle, Success].


%featurize_gaussian_places(TaskList, Success, FeatureList) :-
%  findall( Features, (member(Task, TaskList),
%     featurize_gaussian_place(Task, Success, Features)), FeatureList).

%featurize_gaussian_place(Tsk, Success, Features) :-
%  rdf_has(Tsk, knowrob:'designator', DAction),
%  rdf_has(Tsk, knowrob:'startTime', StartTime),
%  mng_designator_props(DAction, 'OBJ', DObj),
%  designator_assert(SemanticMapInstance, DAction, DObj, 'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j'), !,
%  designator_add_perception(SemanticMapInstance, DObj, Matrix, StartTime),
%  object_pose_at_time(SemanticMapInstance, StartTime, mat(ObjTransform)),
%  mng_lookup_transform('/map', '/base_footprint', StartTime, RobotTransform),
%  pose_into_relative_coord(RobotTransform, ObjTransform, RobotTransformTT),
%  matrix_translation(RobotTransformTT, [RobotX, RobotY, RobotZ]),
%  Features = [Success, RobotX, RobotY, 0].

featurize_gaussian_places(TaskList, FloatFeaturesList, StringFeatureList) :-
  findall( [FloatFeatures, StringFeatures], (member(Task, TaskList),
     featurize_gaussian_place_analyze(Task, FloatFeatures, StringFeatures)), FeatureList),
  findall(Ff, ( member(F, FeatureList), nth0(0, F, Ff)), FloatFeaturesList),
  findall(Sf, ( member(F, FeatureList), nth0(1, F, Sf)), StringFeatureList).

featurize_gaussian_place(Tsk, FloatFeatures, StringFeatures) :-
  rdf_has(Tsk, knowrob:'designator', DAction),
  rdf_has(Tsk, knowrob:'startTime', StartTime),
  rdf_has(Tsk, rdf:type, Class),
  rdf_split_url(_, TskType, Class),
  mng_designator_props(DAction, 'OBJ', DObj),
  mng_designator_props(DAction, 'OBJ.TYPE', ObjType),!,
  designator_assert(SemanticMapInstance, DAction, DObj, 'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j'), !,
  designator_add_perception(SemanticMapInstance, DObj, _Matrix, StartTime),
  object_pose_at_time(SemanticMapInstance, StartTime, mat(ObjTransform)),
  mng_lookup_transform('/map', '/base_footprint', StartTime, RobotTransform),
  pose_into_relative_coord(RobotTransform, ObjTransform, RobotTransformTT),
  matrix_translation(RobotTransformTT, [RobotX, RobotY, RobotZ]),
  matrix_rotation(RobotTransformTT, [QW, QX, QY, QZ]),
  DeltaRobotAngle is atan2((2.0*(QY*QZ + QW*QX)), (QW*QW - QX*QX - QY*QY + QZ*QZ)),
  FloatFeatures = [RobotX, RobotY, RobotZ, DeltaRobotAngle],	
  jpl_new( '[Ljava.lang.String;', [TskType, ObjType], StringFeatures).

color_directed_trajectory(Lnk, Start, End, Interval) :-
  Diff is  End - Start,
  DiffScaled is Interval/Diff,
  color_directed_trajectory(Lnk, Start, End, DiffScaled, Interval, [0.0, 1.0, 0.0]).
  
color_directed_trajectory(Lnk, Start, End, Scale, Interval, [R,G,B]) :-
  ChunkEnd is Start + Interval,
  ChunkEnd =< End,
  rdf_instance_from_class(Lnk, TrajId),
  marker(trajectory(Lnk), T, TrajId), 
  marker_color(T, [R,G,B]),
  marker_update(T, interval(Start, ChunkEnd, dt(Interval))),
  R_new is R+Scale, G_new is G-Scale, 
  !, color_directed_trajectory(Lnk, ChunkEnd, End, Scale, Interval, [R_new,G_new,B]).

color_directed_trajectory(_Lnk, Start, End, _Scale, Interval, _C) :-
  ChunkEnd is Start + Interval,
  End < ChunkEnd, true.

