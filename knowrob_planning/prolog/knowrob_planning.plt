:- begin_tests(knowrob_planning).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_planning/owl/planning_test.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob.owl').

:- rdf_db:rdf_register_prefix(planning_test, 'http://knowrob.org/kb/planning_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_most_specific_specialization

test(owl_most_specific_specializations1) :-
  owl_most_specific([knowrob:'Container', knowrob:'Sink'], Sink),
  rdf_equal(knowrob:'Sink',Sink).
test(owl_most_specific_specializations2) :-
  findall(X, owl_most_specific([knowrob:'Container', knowrob:'HumanScaleObject'], X), Xs),
  rdf_equal(knowrob:'Container',C1), once(member(C1,Xs)),
  rdf_equal(knowrob:'HumanScaleObject',C2), once(member(C2,Xs)).
test(owl_most_specific_specializations3) :-
  owl_most_specific([knowrob:'Sink', knowrob:'HumanScaleObject'], Sink),
  rdf_equal(knowrob:'Sink',Sink).
test(owl_most_specific_specializations4) :-
  owl_most_specific([knowrob:'Sink'], Sink),
  rdf_equal(knowrob:'Sink',Sink).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% test owl_specializable

test(owl_specializable_Instance_Cls1) :-
  rdf_instance_from_class(knowrob:'SpatialThing', X1),
  owl_specializable(X1, knowrob:'Container'),
  owl_specializable(X1, knowrob:'SpatialThing-Localized'),
  owl_specializable(X1, knowrob:'HumanScaleObject'),
  \+ owl_specializable(X1, knowrob:'Event'),
  rdf_retractall(X1, _, _).
test(owl_specializable_Instance_Cls2) :-
  rdf_instance_from_class(knowrob:'Container', X1),
  owl_specializable(X1, knowrob:'SpatialThing'),
  owl_specializable(X1, knowrob:'Sink'),
  \+ owl_specializable(X1, knowrob:'HumanScaleObject'),
  rdf_retractall(X1, _, _).

test(owl_specializable_Cls_Cls1) :-
  owl_specializable(knowrob:'SpatialThing', knowrob:'Container'),
  owl_specializable(knowrob:'SpatialThing', knowrob:'SpatialThing-Localized').
test(owl_specializable_Cls_Cls2) :-
  owl_specializable(knowrob:'Container', knowrob:'SpatialThing'),
  owl_specializable(knowrob:'SpatialThing-Localized', knowrob:'SpatialThing').

test(owl_specializable_intersection_of1) :-
  rdf_instance_from_class(knowrob:'SpatialThing', X1),
  owl_specializable(X1, intersection_of([knowrob:'Container',knowrob:'HumanScaleObject'])),
  rdf_retractall(X1, _, _).
test(owl_specializable_intersection_of2) :-
  rdf_instance_from_class(knowrob:'Container', X1),
  \+ owl_specializable(X1, knowrob:'HumanScaleObject'),
  \+ owl_specializable(X1, intersection_of([knowrob:'Container',knowrob:'HumanScaleObject'])),
  rdf_retractall(X1, _, _).

test(owl_specializable_union_of1) :-
  rdf_instance_from_class(knowrob:'Container', X1),
  \+ owl_specializable(X1, knowrob:'HumanScaleObject'),
  owl_specializable(X1, union_of([knowrob:'Container',knowrob:'HumanScaleObject'])),
  rdf_retractall(X1, _, _).

test(owl_specializable_some_values1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Container'))),
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Container'))).
test(owl_specializable_some_values2) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Sink'))),
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Sink'))).
test(owl_specializable_some_values3) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'SpatialThing'))),
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'SpatialThing'))).
test(owl_specializable_some_values4) :-
  % specializable because there is a subclass of HumanScaleObject that is also a Container
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'HumanScaleObject'))).
test(owl_specializable_some_values5) :-
  % dough has no Container specializations
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Dough'))).
test(owl_specializable_some_values6) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(X1, 
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Sink'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_some_values7) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  \+ owl_specializable(X1,
    restriction(planning_test:'testProperty', some_values_from(knowrob:'Dough'))),
  rdf_retractall(X1, _, _).

test(owl_specializable_all_values1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(knowrob:'Container'))),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(knowrob:'Sink'))).
test(owl_specializable_all_values2) :-
  % SpatialThing is less specific then Container -> not specializable
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(knowrob:'SpatialThing'))),
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(knowrob:'HumanScaleObject'))).
test(owl_specializable_all_values3) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(X1,
    restriction(planning_test:'testProperty', all_values_from(knowrob:'HumanScaleObject'))),
  owl_specializable(X1,
    restriction(planning_test:'testProperty', all_values_from(knowrob:'Container'))),
  rdf_retractall(X1, _, _).

test(owl_specializable_cardinality1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', cardinality(2,2,knowrob:'Container'))).
test(owl_specializable_cardinality2) :-
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(1,1,knowrob:'Container'))).
test(owl_specializable_cardinality3) :-
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(1,1,knowrob:'Sink'))).
test(owl_specializable_cardinality4) :-
  \+ owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(2,2,knowrob:'Container'))).
test(owl_specializable_cardinality5) :-
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', cardinality(2,4,knowrob:'Dough'))).

test(owl_specializable_has_value1) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', has_value(planning_test:'TestContainer_1'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_has_value2) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrB', X1),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', has_value(planning_test:'TestContainer_1'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_has_value3) :-
  rdf_instance_from_class(knowrob:'Container', C1),
  owl_specializable(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', has_value(C1))),
  rdf_retractall(C1, _, _).
test(owl_specializable_has_value4) :-
  rdf_instance_from_class(knowrob:'Container', C1),
  \+ owl_specializable(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty', has_value(C1))),
  rdf_retractall(C1, _, _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_specialization_of

test(owl_specialization_of_Class_Class1) :-
  owl_specialization_of(knowrob:'Container', knowrob:'Container'),
  owl_specialization_of(knowrob:'Container', knowrob:'SpatialThing'),
  owl_specialization_of(knowrob:'Sink', knowrob:'Container'),
  owl_specialization_of(knowrob:'Sink', knowrob:'HumanScaleObject').
test(owl_specialization_of_Class_Class2) :-
  \+ owl_specialization_of(knowrob:'SpatialThing', knowrob:'Container').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_satisfies_restriction_up_to

:- rdf_meta test_restriction_up_to(r,t,t).
test_restriction_up_to(S,Restr,UpTo) :-
  Restr_id=Restr,
  findall( (S,Restr_id,UpTo), owl_satisfies_restriction_up_to(S,Restr_id,UpTo), Xs),
  member( (S,Restr_id,UpTo), Xs ).

test(owl_satisfies_restriction_up_to_class1) :-
  test_restriction_up_to(planning_test:'TestContainer_1', class(knowrob:'Sink'),
                         classify(planning_test:'TestContainer_1', knowrob:'Sink')).
test(owl_satisfies_restriction_up_to_class2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestContainer_1', class(knowrob:'Container'), _).

test(owl_satisfies_restriction_up_to_intersection1) :-
  % not possible because Container can't be specialized to HumanScaleObject
  \+ owl_satisfies_restriction_up_to(planning_test:'TestContainer_1', intersection_of([knowrob:'Container',knowrob:'HumanScaleObject']), _).
test(owl_satisfies_restriction_up_to_intersection2, [nondet]) :-
  rdf_instance_from_class(knowrob:'SpatialThing', X),
  test_restriction_up_to(X, intersection_of([knowrob:'Container',knowrob:'HumanScaleObject']),
                         classify(X, knowrob:'Container')),
  test_restriction_up_to(X, intersection_of([knowrob:'Container',knowrob:'HumanScaleObject']),
                         classify(X, knowrob:'HumanScaleObject')).
test(owl_satisfies_restriction_up_to_intersection3) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestContainer_1', intersection_of([knowrob:'Container']), _).

test(owl_satisfies_restriction_up_to_union1) :-
  test_restriction_up_to(planning_test:'TestContainer_1', union_of([knowrob:'Sink', knowrob:'HumanScaleObject']),
                         classify(planning_test:'TestContainer_1', knowrob:'Sink')).
test(owl_satisfies_restriction_up_to_union2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestContainer_1', union_of([knowrob:'Container']), _).
test(owl_satisfies_restriction_up_to_union3) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestContainer_1', union_of([knowrob:'Container', knowrob:'HumanScaleObject']), _).
test(owl_satisfies_restriction_up_to_union4) :-
  % not possible because Container can't be specialized to HumanScaleObject
  \+ test_restriction_up_to(planning_test:'TestContainer_1', union_of([knowrob:'HumanScaleObject']), _).
test(owl_satisfies_restriction_up_to_union5) :-
  rdf_instance_from_class(knowrob:'SpatialThing', X),
  test_restriction_up_to(X, union_of([knowrob:'HumanScaleObject']), classify(X, knowrob:'HumanScaleObject')).
test(owl_satisfies_restriction_up_to_union6, [nondet]) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, union_of([knowrob:'HumanScaleObject',knowrob:'Event']), classify(X, knowrob:'HumanScaleObject')),
  test_restriction_up_to(X, union_of([knowrob:'HumanScaleObject',knowrob:'Event']), classify(X, knowrob:'Event')).

test(owl_satisfies_restriction_up_to_has_value1) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, restriction(planning_test:'testProperty', has_value(planning_test:'TestContainer_1')),
                         specify(X, planning_test:'testProperty', planning_test:'TestContainer_1', 1)).
test(owl_satisfies_restriction_up_to_has_value2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', has_value(planning_test:'TestContainer_1')), _).

test(owl_satisfies_restriction_up_to_all_values1) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(knowrob:'Event')), _).
test(owl_satisfies_restriction_up_to_all_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(knowrob:'Sink')),
                         classify(planning_test:'TestContainer_1',knowrob:'Sink')).
test(owl_satisfies_restriction_up_to_all_values4) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(knowrob:'Container')), _).
test(owl_satisfies_restriction_up_to_all_values5) :-
  rdf_instance_from_class(owl:'Thing', X),
  % already specific enough
  \+ test_restriction_up_to(X, restriction(planning_test:'testProperty', all_values_from(knowrob:'Container')), _).

test(owl_satisfies_restriction_up_to_some_values1) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, restriction(planning_test:'testProperty', some_values_from(knowrob:'Container')),
                         specify(X, planning_test:'testProperty', knowrob:'Container', 1)).
test(owl_satisfies_restriction_up_to_some_values2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(knowrob:'Container')), _).
test(owl_satisfies_restriction_up_to_some_values3) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(knowrob:'Sink')),
                         classify(planning_test:'TestContainer_1', knowrob:'Sink')).
test(owl_satisfies_restriction_up_to_some_values5) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty',
                         some_values_from(restriction(planning_test:'testProperty',has_value(planning_test:'TestContainer_1')))),
                         specify(planning_test:'TestContainer_1',planning_test:'testProperty',planning_test:'TestContainer_1',1)).
test(owl_satisfies_restriction_up_to_some_values4) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(knowrob:'Dough')), _).
test(owl_satisfies_restriction_up_to_some_values7) :-
  % exactly 1 Container restriction still allows other value types
  test_restriction_up_to(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty',
                         some_values_from(knowrob:'HumanScaleObject')),
                         specify(planning_test:'TestSomeRestr_2',planning_test:'testProperty',knowrob:'HumanScaleObject',1)).

test(owl_satisfies_restriction_up_cardinality_values1) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(2,2,knowrob:'Container')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',knowrob:'Container',1)).
test(owl_satisfies_restriction_up_cardinality_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(3,3,knowrob:'Container')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',knowrob:'Container',2)).
test(owl_satisfies_restriction_up_cardinality_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(2,3,knowrob:'Container')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',knowrob:'Container',1)).
test(owl_satisfies_restriction_up_cardinality_values1) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty range
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(1,1,knowrob:'Dough')), _).
test(owl_satisfies_restriction_up_cardinality_values1) :-
  % not allowed because cardinality restriction (1,1) can't be specialized to (2,2)
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty', cardinality(2,2,knowrob:'Container')), _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests(knowrob_planning).
