:- module(rs_plan_pipeline,
    [
  compute_annotators/1,
  annotators/1,
  compute_annotator_outputs/2,
  compute_annotator_inputs/2,
  reset_planning/0,
  annotator_outputs/2,
  annotator_inputs/2,
  type_available/1,
  annotator_in_dependency_chain_of/2,
  dependency_chain_as_set_for_annotator/2,
  dependency_chain_ordering/3,
  ordered_dependency_chain_for_annotator/2,
  annotator_missing_inputs/2,
  annotators_satisfying_atleast_one_input/2,
  annotators_satisfying_direct_inputs_of/2,
  get_required_annotators_for_annotator_list/2,
  annotatorlist_requirements_fulfilled/1,
  get_missing_annotators/2,
  can_inputs_be_provided_for_annotator_list/1,
  build_pipeline/2,
  annotators_for_predicate/2,
  annotators_for_predicates/2,
  build_pipeline_from_predicates/2,
  build_pipeline_for_object/2,
  obj_has_predicate/2,
  enumerate_annotator_sets_for_predicate/2,
  enumerate_annotator_sets_for_predicates/2,
  subsequence/2,
  subseq/2,
  predicates_for_object/2,
  build_pipeline_for_subclass_leafs/3,
  build_pipeline_for_subclasses/3,
  leaf_subclasses/2,
  detect_if_individual_present/3,
  build_single_pipeline_from_predicates/2
]).

:- rdf_meta
   build_pipeline_for_object(+,+),
   compute_annotator_inputs(r,r),
   build_pipeline(t,t).

:- use_module(library('jpl')).

% Load RoboSherlock components
:- owl_parse('package://knowrob_robosherlock/owl/rs_components.owl').

% Register Namespaces
:- rdf_db:rdf_register_ns(rs_components, 'http://knowrob.org/kb/rs_components.owl#',     [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pipeline Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

compute_annotators(A) :- 
	owl_subclass_of(A,rs_components:'RoboSherlockComponent'),
        not(A = 'http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#AnnotationComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#DetectionComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#IoComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#PeopleComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#SegmentationComponent'). 

% cache the annotators
:- forall(compute_annotators(A), assert(annotators(A)) ).

% Get outputs of Annotator
compute_annotator_outputs(Annotator,Output) :- 
	% current_robot(R),!,
	annotators(Annotator), 
	owl_class_properties(Annotator,rs_components:'perceptualOutput',Output).
	%  action_feasible_on_robot(Annotator, R).

% Get inputs of Annotator
compute_annotator_inputs(Annotator,Input) :- 
	% current_robot(R),!,
	annotators(Annotator), 
	owl_class_properties(Annotator,rs_components:'perceptualInputRequired',Input).
	%  action_feasible_on_robot(Annotator, R).

% cache outputs/inputs
:- forall(compute_annotator_outputs(A,O), assert(annotator_outputs(A,O)) ).
:- forall(compute_annotator_inputs(A,I), assert(annotator_inputs(A,I)) ).

% If you changed the robot model or something else in this code,
% call this rule to cache the annotators and their I/Os again.
reset_planning:- retractall(annotator_outputs(_,_)),
	retractall(annotator_inputs(_,_)),
	retractall(annotators(_)),
	forall(compute_annotators(A), assert(annotators(A)) ),
	forall(compute_annotator_outputs(A,O), assert(annotator_outputs(A,O)) ),
	forall(compute_annotator_inputs(A,I), assert(annotator_inputs(A,I)) ).

% Get every type that can be put out by any annotator
type_available(Output) :- 
	annotator_outputs(_,Output).

% Check if Annotator A is somewhere in the Depedency chain of D.
% This means for example in RoboSherlock, where the CollectionReader should be at
% the first place in every pipeline:
% annotator_in_dependency_chain_of(CollectionReader,SomeShapeAnnotator) should be true.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, CollectionReader) should be false.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, imagePreprocessor) should be false.
% annotator_in_dependency_chain_of(X, Collectionreader) should be false.
%
% Trivial case: A is in the dependency chain of D, if A provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :- 
	annotator_outputs(A,Input),
	annotator_inputs(D, Input).

% Recursive case: A is in the Dependency chain of D, if A provides a Type
% that X needs, and X provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :- 
	annotator_outputs(A,Input),
	annotator_inputs(X, Input),
	annotator_in_dependency_chain_of(X, D).

% calculate the full dependency chain for a given
% Annotator, include the annotator itself. The chain 
% MUST NOT be in the correct order
dependency_chain_as_set_for_annotator(Annotator,S) :-
	L=[Annotator],
	setof(X,annotator_in_dependency_chain_of(X,Annotator),D),
	append(L,D,S); % Either a set of dependencies can be calculated
	S=[]. % or we return an empty list, when no dependencies are present

% AnnotatorA < AnnotatorB when A is somewhere in the beginning
% of the dependency chain of B
dependency_chain_ordering(R, AnnotatorA, AnnotatorB) :-
	annotator_in_dependency_chain_of(AnnotatorA, AnnotatorB) -> R = '<' ; R = '>'.
% Order the output of dependency_chain_as_set_for_annotator in a manner
% that the evaluation order in L is correct.
ordered_dependency_chain_for_annotator(Annotator,L) :-
	dependency_chain_as_set_for_annotator(Annotator,AnnotatorChainSet),
	predsort(dependency_chain_ordering, AnnotatorChainSet, L).


% Can an input never be satisified?
annotator_missing_inputs(Annotator,Missing) :- 
	findall(Input, (annotator_inputs(Annotator, Input),
	not(type_available(Input)) ), Missing).

% Get a list caled AnnotatorSatisfyingInput, that
% includes all annotators that provide _one_ input of Annotator A.
annotators_satisfying_atleast_one_input(Annotator, AnnotatorSatisfyingInput):-
	annotator_inputs(Annotator, Input),
	setof(X, annotator_outputs(X,Input), AnnotatorSatisfyingInput).

% Get a List of Annotators, that provide the required inputs of
% _ALL_ inputs of A
annotators_satisfying_direct_inputs_of(Annotator, AnnotatorSet):-
	setof(X, annotators_satisfying_atleast_one_input(Annotator, X), L),
	flatten(L, AnnotatorSet);
	AnnotatorSet = []. % Return empty set, when a annotator doesn't need any inputs

get_required_annotators_for_annotator_list(AnnotatorList,RequiredAnnotators):-
	maplist(annotators_satisfying_direct_inputs_of, AnnotatorList, List),
	flatten(List, FlattenedList),
	list_to_set(FlattenedList, RequiredAnnotators).

% Check, if all the required annotators of the annotators are in the given list.
% WARNING: This does NOT work if you pass a list, that has unsatisfiable
% input requirements. This means, that the input of an Annotator
% is not the Result of ANY Annotator in the KnowledgeBase.
annotatorlist_requirements_fulfilled(AnnotatorList):-
	get_required_annotators_for_annotator_list(AnnotatorList, ReqA),!,
	% is ReqA a Subset of AnnotatorList? 
	subset(ReqA, AnnotatorList).

% Take a List of Annotators called L, calculate all the required inputs
% and the Annotators that do provide them.
% Add the Annotators to L.
% Repeat, until the size of L doesn't change.
% add_required_annotators_until_inputs_satisfied(AnnotatorList, ResultList).

% Take a List of Annotators, calculate it's dependencies on other
% Annotators and add them to the ResultList.
get_missing_annotators(AnnotatorList, ResultList):-
	maplist(dependency_chain_as_set_for_annotator,AnnotatorList, L),
	flatten(L, FlattenedList),
	list_to_set(FlattenedList, ResultList).

% Check, if the required inputs of the Anntators in AnnotatorList
% can be provided by any of the Annotators in the System.
% If the Annotator doesn't require any inputs, the method will be true.
can_inputs_be_provided_for_annotator_list(AnnotatorList):-
	% check for all members of AnnotatorList
	forall(member(R,AnnotatorList),
	  % The Annotator doesn't need any inputs
	  (\+ annotator_inputs(R,_) ;
	    % or: EVERY input will be provided by some annotator.
	    forall(annotator_inputs(R,T), annotator_outputs(_,T))
	  )
	).

% TODO: Consistency Checks! Check the dependency graph for the absence of cycles.
% TODO: Test with multiple inputs 

% ListOfAnnotators: A List of Annotators that should be run. The list does not have to include the necessary dependencies for the Annotators nor must be in the correct order.
% EvaluationList: A List of Annotators that form a complete Pipeline. The Annotators should be in the correct evaluation order
build_pipeline(ListOfAnnotators,EvaluationList):-
	% Are there any requested types that can't be calculated by the system?
	can_inputs_be_provided_for_annotator_list(ListOfAnnotators) ->
	  (annotatorlist_requirements_fulfilled(ListOfAnnotators) ->
	    % If requirements are already fulfilled, bring everything in the correct order and return
	    predsort(dependency_chain_ordering, ListOfAnnotators, EvaluationList);
	    % else: get the missing annotators to complete the list and sort it.
	    get_missing_annotators(ListOfAnnotators, FullListOfAnnotators),!,
	    predsort(dependency_chain_ordering, FullListOfAnnotators, EvaluationList)
	  )	
	; write('** WARNING: One or more inputs of the given List of Annotators can not be computed by an Algorithm in the KnowledgeBase **'),
	fail.


% Map a predefined set of predicates to Annotator Outputs
annotators_for_predicate(shape,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationShape' ).
annotators_for_predicate(color,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationSemanticcolor' ).
annotators_for_predicate(size,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGeometry' ).
annotators_for_predicate(location,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationTflocation' ).
annotators_for_predicate(logo,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).
annotators_for_predicate(text,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).
annotators_for_predicate(product,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationGoggles' ).
annotators_for_predicate(class,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationDetection' ).
annotators_for_predicate(detection,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationDetection' ).
annotators_for_predicate(handle,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationHandleannotation' ).
annotators_for_predicate(cylindrical_shape,A) :- 
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationCylindricalshape' ).
annotators_for_predicate(obj-part,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationClusterpart' ).
annotators_for_predicate(inspect,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationClusterpart' ).
annotators_for_predicate(contains,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsdemosAcatSubstance' ).
annotators_for_predicate(volume,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsdemosAcatVolume' ).
annotators_for_predicate(ingredient,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsdemosRobohowPizza' ).
annotators_for_predicate(type,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationDetection' ).
annotators_for_predicate(cad-model,A) :-
	annotator_outputs(A,'http://knowrob.org/kb/rs_components.owl#RsAnnotationPoseannotation' ).


% Detection Clue Annotators. Pick specific Annotators, not based on their inputs/outputs
annotators_for_predicate(blort,A) :- 
	annotators(A), 
	A = 'http://knowrob.org/kb/rs_components.owl#BlortAnnotator'.

annotators_for_predicate(linemod,A) :- 
	annotators(A), 
	A = 'http://knowrob.org/kb/rs_components.owl#LinemodAnnotator'.

annotators_for_predicate(pancakedetector,A) :- 
	annotators(A), 
	A = 'http://knowrob.org/kb/rs_components.owl#PancakeAnnotator'.

annotators_for_predicates(Predicates, A):-
	member(P,Predicates), 
	annotators_for_predicate(P, A).

obj_has_predicate(cylindrical_shape, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, rs_components:'CylindricalShape').

obj_has_predicate(shape, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, rs_components:'Shape'),
	not(O = 'http://knowrob.org/kb/rs_components.owl#CylindricalShape').

obj_has_predicate(color, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, knowrob:'ColoredThing').

obj_has_predicate(size, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, rs_components:'Size').

obj_has_predicate(logo, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, rs_components:'Logo').

obj_has_predicate(text, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, rs_components:'TextOnObject').

obj_has_predicate(handle, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasVisualProperty',O),
	owl_subclass_of(O, knowrob:'Handle').

obj_has_predicate(location, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/knowrob.owl#aboveOf',_).

% Predicates for DetectionClues
obj_has_predicate(blort, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasDetectionClue',O), 
	O = 'http://knowrob.org/kb/rs_components.owl#BlortModel'.

obj_has_predicate(linemod, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasDetectionClue',O),
	O = 'http://knowrob.org/kb/rs_components.owl#LinemodModel'.

obj_has_predicate(pancakedetector, Obj):- 
	owl_class_properties(Obj,'http://knowrob.org/kb/rs_components.owl#hasDetectionClue',O),
	O = 'http://knowrob.org/kb/rs_components.owl#PancakeDetector'.


% Yield every subsequence of possible Annotators for a given Predicate
enumerate_annotator_sets_for_predicate(Predicate, AnnotatorSet):-
	setof(X, annotators_for_predicate(Predicate,X), Annotators), 
	subseq(Annotators, AnnotatorSet).

% For each predicate:
%   Select a subsequence of the corresponding annotators
%
%   color: [1,2]
%   shape: [3]
%
%   Result: [1,2,3] , [1,3], [2,3]
enumerate_annotator_sets_for_predicates(Predicates, AnnotatorSet):-
	maplist(enumerate_annotator_sets_for_predicate,Predicates,X),
	flatten(X, AnnotatorSet).


predicates_for_object(Obj,Preds):- 
	setof(X,obj_has_predicate(X,Obj),Preds).

build_pipeline_from_predicates(ListOfPredicates,Pipeline):-
	enumerate_annotator_sets_for_predicates(ListOfPredicates, Annotators),
	build_pipeline(Annotators, P),
	build_pipeline(P,PP),
	build_pipeline(PP,Pipeline). 

build_single_pipeline_from_predicates(ListOfPredicates,Pipeline):-
	setof(X,annotators_for_predicates(ListOfPredicates, X), Annotators), % Only build one list of annotators for the given Predicates
	build_pipeline(Annotators, TempPipeline),%same as above
	build_pipeline(TempPipeline,P),
	build_pipeline(P,Pipeline).

build_pipeline_for_object(Obj,Pipeline):-
	predicates_for_object(Obj,ListOfPredicates),!,
	build_pipeline_from_predicates(ListOfPredicates,Pipeline).

build_single_pipeline_for_object(Obj,Pipeline):-
	predicates_for_object(Obj,ListOfPredicates),!,
	build_single_pipeline_from_predicates(ListOfPredicates,Pipeline).

build_pipeline_for_subclasses(Obj,Pipeline,ChildObj):-
	owl_subclass_of(ChildObj,Obj),
	build_single_pipeline_for_object(ChildObj,Pipeline).

build_pipeline_for_subclass_leafs(Obj,Pipeline,ChildObj):-
	leaf_subclasses(ChildObj,Obj),
	build_single_pipeline_for_object(ChildObj,Pipeline).

% Return all subclasses of Superclass, that do not have direct subclasses.
% If you think of the ontology as a tree,
% this will be the leaf nodes
leaf_subclasses(Sub, Superclass):-
	owl_subclass_of(Sub, Superclass), \+ owl_direct_subclass_of(_,Sub).


% Reminder on using maplist:
% test(N,R):- R is N*N.
% ?- maplist(test,[3,5,7],X).
% X = [9,25,49].
% Source: http://a-programmers-life.blogspot.de/2010/10/subsequences-in-prolog.html
%
% Return all subsequences of a given set.
% Starts with the full sequence

subsequence([H|T],[H|T2]) :- subsequence(T,T2).
subsequence([_|T],[H2|T2]) :- subsequence(T,[H2|T2]).
subsequence(_,[]).

subseq(Sequence, Subsequence):- subsequence(Sequence, Subsequence), not(Subsequence = []).

detect_volume(Obj,P):-
  owl_subclass_of(Obj,knowrob:'Container'),
  obj_has_predicate(cylindrical_shape, Obj),
  build_pipeline_for_object(Obj,P).

detect_if_individual_present(Obj,DependentObjectClass,P):-
  owl_individual_of(_, DependentObjectClass),
  build_pipeline_for_object(Obj,P).

/*
* Definitions done
*/

:- print('----------\n').
:- print('RSComponents ontology is available under http://robosherlock.org/kb/rs_components.owl#\n').
:- print('----------\n').

