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
      save_selector/2
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('util')).

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
% @param Algorithm          Name of the selection algorithm, e.g. 'weka.classifier.svm'
% @param Options        Options that are passed to the selection algorithm
% @param Instances      Data that will train the classifier

train_classifier(TrainedClassifier, Algorithm, Options, Instances) :-
  list_to_array(Options, OptionsArray),
  jpl_call('org.knowrob.learning.Classifier', 'getAClassifier', [Algorithm, OptionsArray, Instances], TrainedClassifier).


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
  prepare_data(Data, I1),
  check_weka_data(I1),
  lists_to_arrays(I1, DataArr),
  jpl_call(Classifier, classify, [DataArr], ClassifiedArr),
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


