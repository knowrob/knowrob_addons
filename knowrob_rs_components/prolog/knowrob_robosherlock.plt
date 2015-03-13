:- begin_tests(knowrob_robosherlock).
% :- register_ros_package('knowrob_common').
% :- register_ros_package('knowrob_actions').

% :- owl_parser:owl_parse('package://knowrob_rs/owl/rs_components_test.owl').

% test(a) :-
%         A is 2^3,
%         assertion(A == 8).

% load test objects
:- owl_parse('package://knowrob_robosherlock/owl/test_objects.owl').

% Build a Chain for PlaneEstimation and SacModelAnnotator
test(build_pipeline1):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator'],S),
  length(S,Length),
  assertion(Length == 6),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator']). 

% Build a chain for SacModelAnnotator only.
test(build_pipeline2):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#SacModelAnnotator'],S),
  length(S,Length),
  assertion(Length == 6),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator']). 

% Test the behavior, when a complete pipeline is given
% If there are multiple solutions possible, the input pipelines does not have to be equal to the
% computed pipeline. 
test(build_pipeline_complete1):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator'],S),
  length(S,Length),
  assertion(Length == 6),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator']). 


% Test the behavior, when a complete pipeline is given, but in the wrong order
test(build_pipeline_complete2):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor'],S),
  length(S,Length),
  assertion(Length == 4),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor']). 

% Test the behavior, when a complete pipeline is given, but in the wrong order
test(build_pipeline_complete3):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor'],S),
  length(S,Length),
  assertion(Length == 4),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor']). 

% check a list of annotators, that already fulfils the requirements.
% Input List should be equal to Output List.
test(build_pipeline_root_elem_only):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#CollectionReader'],S),
  length(S,Length),
  assertion(Length == 1),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader']). 

% Pass an Annotator, that has multiple inputs from different annotators as input
test(build_pipeline_with_multiple_inputs):-
  build_pipeline(['http://knowrob.org/kb/rs_components.owl#PCLFeatureExtractor'],S),
  length(S,Length),
  assertion(Length == 6),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#PCLFeatureExtractor']).  

% Test correct pipeline
test(requirement_checking1):-
  annotatorlist_requirements_fulfilled(['http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor']).

% Test correct pipeline in wrong order
test(requirement_checking2):-
  annotatorlist_requirements_fulfilled(['http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator']).

% Test pipeline that only consists of CollectionReader.
% Since it has no other dependencies, it should be fine.
test(requirement_checking3):-
  annotatorlist_requirements_fulfilled(['http://knowrob.org/kb/rs_components.owl#CollectionReader']).


% incomplete annotator sequence. CollectionReader is missing.
test(requirement_checking_incomplete1):-
  \+ annotatorlist_requirements_fulfilled(['http://knowrob.org/kb/rs_components.owl#ImagePreprocessor']).

test(input_checking):-
  can_inputs_be_provided_for_annotator_list(['http://knowrob.org/kb/rs_components.owl#ImagePreprocessor']).

test(input_checking_for_type_without_input_requirements):-
  can_inputs_be_provided_for_annotator_list(['http://knowrob.org/kb/rs_components.owl#CollectionReader']).

% Test build_pipeline_from ...
% which is the interface to the outer world
test(build_pipeline_from_predicates1):-
  build_pipeline_from_predicates([shape,color],S),!, % Test the first solution
  length(S,Length),
  assertion(Length == 8),
  assertion(S == ['http://knowrob.org/kb/rs_components.owl#CollectionReader','http://knowrob.org/kb/rs_components.owl#ImagePreprocessor','http://knowrob.org/kb/rs_components.owl#PlaneAnnotator','http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor','http://knowrob.org/kb/rs_components.owl#ClusterColorHistogramCalculator','http://knowrob.org/kb/rs_components.owl#NormalEstimator','http://knowrob.org/kb/rs_components.owl#SacModelAnnotator','http://knowrob.org/kb/rs_components.owl#PrimitiveShapeAnnotator']). 

test(blort_predicate_for_milk):-
  obj_has_predicate(blort,'http://knowrob.org/kb/rs_test_objects.owl#Milk'),!.

test(linemod_predicate_for_pancakemaker):-
  obj_has_predicate(linemod,'http://knowrob.org/kb/rs_test_objects.owl#PancakeMaker'),!.


test(build_pipeline_for_object_milk):-
  build_pipeline_for_object('http://knowrob.org/kb/rs_test_objects.owl#Milk',S),!, % Test the first solution
  length(S,Length),
  assertion(Length == 9),
  assertion(S == 
  [
  'http://knowrob.org/kb/rs_components.owl#CollectionReader',
  'http://knowrob.org/kb/rs_components.owl#ImagePreprocessor',
  'http://knowrob.org/kb/rs_components.owl#NormalEstimator',
  'http://knowrob.org/kb/rs_components.owl#PlaneAnnotator',
  'http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor',
  'http://knowrob.org/kb/rs_components.owl#SacModelAnnotator',
  'http://knowrob.org/kb/rs_components.owl#PrimitiveShapeAnnotator',
  'http://knowrob.org/kb/rs_components.owl#ClusterColorHistogramCalculator',
  'http://knowrob.org/kb/rs_components.owl#BlortAnnotator']).

test(build_pipeline_for_object_pancakemaker):-
  build_pipeline_for_object('http://knowrob.org/kb/rs_test_objects.owl#PancakeMaker',S),!, % Test the first solution
  length(S,Length),
  assertion(Length == 7),
  assertion(S == 
  [
  'http://knowrob.org/kb/rs_components.owl#CollectionReader',
  'http://knowrob.org/kb/rs_components.owl#ImagePreprocessor',
  'http://knowrob.org/kb/rs_components.owl#PlaneAnnotator',
  'http://knowrob.org/kb/rs_components.owl#PointCloudClusterExtractor',
  'http://knowrob.org/kb/rs_components.owl#Cluster3DGeometryAnnotator',
  'http://knowrob.org/kb/rs_components.owl#LinemodAnnotator',
  'http://knowrob.org/kb/rs_components.owl#ClusterColorHistogramCalculator']).
:- end_tests(knowrob_robosherlock).
