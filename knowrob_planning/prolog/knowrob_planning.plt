:- begin_tests(knowrob_planning).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_planning/owl/planning_test.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob.owl').

:- rdf_db:rdf_register_prefix(planning_test, 'http://knowrob.org/kb/planning_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_most_specific_specialization

test(owl_most_specific_specializations1) :-
  owl_most_specific([dul:'Agent', dul:'Organization'], X),
  rdf_equal(dul:'Organization',X).
test(owl_most_specific_specializations2) :-
  findall(X, owl_most_specific([dul:'Agent', dul:'Substance'], X), Xs),
  rdf_equal(dul:'Agent',C1), once(member(C1,Xs)),
  rdf_equal(dul:'Substance',C2), once(member(C2,Xs)).
test(owl_most_specific_specializations4) :-
  owl_most_specific([dul:'Agent'], Agent),
  rdf_equal(dul:'Agent',Agent).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% test owl_specializable

test(owl_specializable_Instance_Cls1) :-
  rdf_instance_from_class(dul:'Object', X1),
  owl_specializable(X1, dul:'Agent'),
  owl_specializable(X1, dul:'Organization'),
  \+ owl_specializable(X1, dul:'Event'),
  rdf_retractall(X1, _, _).
test(owl_specializable_Instance_Cls2) :-
  rdf_instance_from_class(dul:'Agent', X1),
  owl_specializable(X1, dul:'Object'),
  owl_specializable(X1, dul:'Organization'),
  \+ owl_specializable(X1, dul:'Region'),
  rdf_retractall(X1, _, _).

test(owl_specializable_Cls_Cls1) :-
  owl_specializable(dul:'Object', dul:'Agent').
test(owl_specializable_Cls_Cls2) :-
  owl_specializable(dul:'Agent', dul:'Object').

test(owl_specializable_intersection_of1) :-
  rdf_instance_from_class(dul:'Object', X1),
  owl_specializable(X1, intersection_of([dul:'Agent',dul:'PhysicalObject'])),
  rdf_retractall(X1, _, _).
test(owl_specializable_intersection_of2) :-
  rdf_instance_from_class(dul:'Object', X1),
  \+ owl_specializable(X1, dul:'Region'),
  \+ owl_specializable(X1, intersection_of([dul:'Agent',dul:'Region'])),
  rdf_retractall(X1, _, _).

test(owl_specializable_union_of1) :-
  rdf_instance_from_class(dul:'Object', X1),
  \+ owl_specializable(X1, dul:'Region'),
  owl_specializable(X1, union_of([dul:'Agent',dul:'Region'])),
  rdf_retractall(X1, _, _).

test(owl_specializable_some_values1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(dul:'Agent'))),
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', some_values_from(dul:'Agent'))).
test(owl_specializable_some_values2) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(dul:'Organization'))),
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', some_values_from(dul:'Organization'))).
test(owl_specializable_some_values4_1) :-
  owl_specializable(dul:'Agent', dul:'PhysicalObject').
test(owl_specializable_some_values4_2) :-
  owl_specializable(planning_test:'TestObject_1', dul:'PhysicalObject').
test(owl_specializable_some_values4_3) :-
  % specializable because there is a subclass of Agent that is also a PhysicalObject
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(dul:'PhysicalObject'))).
test(owl_specializable_some_values5) :-
  % substance has no agent specializations
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', some_values_from(dul:'Substance'))).
test(owl_specializable_some_values6) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(X1, 
    restriction(planning_test:'testProperty', some_values_from(dul:'SocialAgent'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_some_values7) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  \+ owl_specializable(X1,
    restriction(planning_test:'testProperty', some_values_from(dul:'Substance'))),
  rdf_retractall(X1, _, _).

test(owl_specializable_all_values1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(dul:'Agent'))),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(dul:'Organization'))).
test(owl_specializable_all_values2) :-
  % Entity is less specific then Agent -> not specializable
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(dul:'Entity'))),
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', all_values_from(dul:'Object'))).
test(owl_specializable_all_values3) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(X1,
    restriction(planning_test:'testProperty', all_values_from(dul:'Object'))),
  owl_specializable(X1,
    restriction(planning_test:'testProperty', all_values_from(dul:'Agent'))),
  rdf_retractall(X1, _, _).

test(owl_specializable_cardinality1) :-
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', cardinality(2,2,dul:'Agent'))).
test(owl_specializable_cardinality2) :-
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(1,1,dul:'Agent'))).
test(owl_specializable_cardinality3) :-
  owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(1,1,dul:'Organization'))).
test(owl_specializable_cardinality4) :-
  \+ owl_specializable(planning_test:'TestSomeRestr_2',
    restriction(planning_test:'testProperty', cardinality(2,2,dul:'Agent'))).
test(owl_specializable_cardinality5) :-
  \+ owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', cardinality(2,4,dul:'Region'))).

test(owl_specializable_has_value1) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrA', X1),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', has_value(planning_test:'TestObject_1'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_has_value2) :-
  rdf_instance_from_class(planning_test:'TestSomeRestrB', X1),
  owl_specializable(planning_test:'TestSomeRestr_1',
    restriction(planning_test:'testProperty', has_value(planning_test:'TestObject_1'))),
  rdf_retractall(X1, _, _).
test(owl_specializable_has_value3) :-
  rdf_instance_from_class(dul:'Agent', C1),
  owl_specializable(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', has_value(C1))),
  rdf_retractall(C1, _, _).
test(owl_specializable_has_value4) :-
  rdf_instance_from_class(dul:'Agent', C1),
  \+ owl_specializable(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty', has_value(C1))),
  rdf_retractall(C1, _, _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_specialization_of

test(owl_specialization_of_Class_Class1) :-
  owl_specialization_of(dul:'Agent', dul:'Agent'),
  owl_specialization_of(dul:'Agent', dul:'Object'),
  owl_specialization_of(dul:'Organization', dul:'Agent'),
  owl_specialization_of(dul:'Organization', dul:'Object').
test(owl_specialization_of_Class_Class2) :-
  \+ owl_specialization_of(dul:'Object', dul:'Agent').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% test owl_satisfies_restriction_up_to

:- rdf_meta test_restriction_up_to(r,t,t).
test_restriction_up_to(S,Restr,UpTo) :-
  Restr_id=Restr,
  findall( (S,Restr_id,UpTo), owl_satisfies_restriction_up_to(S,Restr_id,UpTo), Xs),
  member( (S,Restr_id,UpTo), Xs ).

test(owl_satisfies_restriction_up_to_class1) :-
  test_restriction_up_to(planning_test:'TestObject_1', class(dul:'Organization'),
                         classify(planning_test:'TestObject_1', dul:'Organization')).
test(owl_satisfies_restriction_up_to_class2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestObject_1', class(dul:'Agent'), _).

test(owl_satisfies_restriction_up_to_intersection1) :-
  % not possible because Agent can't be specialized to Region
  \+ owl_satisfies_restriction_up_to(planning_test:'TestObject_1', intersection_of([dul:'Organization',dul:'Region']), _).
test(owl_satisfies_restriction_up_to_intersection2, [nondet]) :-
  rdf_instance_from_class(dul:'Object', X),
  test_restriction_up_to(X, intersection_of([dul:'Agent',dul:'Substance']),
                         classify(X, dul:'Agent')),
  test_restriction_up_to(X, intersection_of([dul:'Agent',dul:'Substance']),
                         classify(X, dul:'Substance')).
test(owl_satisfies_restriction_up_to_intersection3) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestObject_1', intersection_of([dul:'Agent']), _).

test(owl_satisfies_restriction_up_to_union1) :-
  test_restriction_up_to(planning_test:'TestObject_1', union_of([dul:'Organization', dul:'SocialAgent']),
                         classify(planning_test:'TestObject_1', dul:'Organization')).
test(owl_satisfies_restriction_up_to_union2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestObject_1', union_of([dul:'Agent']), _).
test(owl_satisfies_restriction_up_to_union3) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestObject_1', union_of([dul:'Agent', dul:'Object']), _).
test(owl_satisfies_restriction_up_to_union4) :-
  % not possible because Agent can't be specialized to Region
  \+ test_restriction_up_to(planning_test:'TestObject_1', union_of([dul:'Region']), _).
test(owl_satisfies_restriction_up_to_union5) :-
  rdf_instance_from_class(dul:'Object', X),
  test_restriction_up_to(X, union_of([dul:'Agent']), classify(X, dul:'Agent')).
test(owl_satisfies_restriction_up_to_union6, [nondet]) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, union_of([dul:'Agent',dul:'Substance']), classify(X, dul:'Agent')),
  test_restriction_up_to(X, union_of([dul:'Agent',dul:'Substance']), classify(X, dul:'Substance')).

test(owl_satisfies_restriction_up_to_has_value1) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, restriction(planning_test:'testProperty', has_value(planning_test:'TestObject_1')),
                         specify(X, planning_test:'testProperty', planning_test:'TestObject_1', 1)).
test(owl_satisfies_restriction_up_to_has_value2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', has_value(planning_test:'TestObject_1')), _).

test(owl_satisfies_restriction_up_to_all_values1) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(dul:'Event')), _).
test(owl_satisfies_restriction_up_to_all_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(dul:'Organization')),
                         classify(planning_test:'TestObject_1',dul:'Organization')).
test(owl_satisfies_restriction_up_to_all_values4) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', all_values_from(dul:'Agent')), _).
test(owl_satisfies_restriction_up_to_all_values5) :-
  rdf_instance_from_class(owl:'Thing', X),
  % already specific enough
  \+ test_restriction_up_to(X, restriction(planning_test:'testProperty', all_values_from(dul:'Agent')), _).

test(owl_satisfies_restriction_up_to_some_values1) :-
  rdf_instance_from_class(owl:'Thing', X),
  test_restriction_up_to(X, restriction(planning_test:'testProperty', some_values_from(dul:'Agent')),
                         specify(X, planning_test:'testProperty', dul:'Agent', 1)).
test(owl_satisfies_restriction_up_to_some_values2) :-
  % already specific enough
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(dul:'Agent')), _).
test(owl_satisfies_restriction_up_to_some_values3) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(dul:'Organization')),
                         classify(planning_test:'TestObject_1', dul:'Organization')).
test(owl_satisfies_restriction_up_to_some_values5) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty',
                         some_values_from(restriction(planning_test:'testProperty',has_value(planning_test:'TestObject_1')))),
                         specify(planning_test:'TestObject_1',planning_test:'testProperty',planning_test:'TestObject_1',1)).
test(owl_satisfies_restriction_up_to_some_values4) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', some_values_from(knowrob:'Region')), _).
test(owl_satisfies_restriction_up_to_some_values7) :-
  % exactly 1 Agent restriction still allows other value types
  test_restriction_up_to(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty',
                         some_values_from(dul:'Substance')),
                         specify(planning_test:'TestSomeRestr_2',planning_test:'testProperty',dul:'Substance',1)).

test(owl_satisfies_restriction_up_cardinality_values1) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(2,2,dul:'Agent')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',dul:'Agent',1)).
test(owl_satisfies_restriction_up_cardinality_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(3,3,dul:'Agent')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',dul:'Agent',2)).
test(owl_satisfies_restriction_up_cardinality_values2) :-
  test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(2,3,dul:'Agent')),
                         specify(planning_test:'TestSomeRestr_1',planning_test:'testProperty',dul:'Agent',1)).
test(owl_satisfies_restriction_up_cardinality_values1) :-
  % not allowed due to TestSomeRestr_1 restrictions on testProperty range
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_1', restriction(planning_test:'testProperty', cardinality(1,1,dul:'Substance')), _).
test(owl_satisfies_restriction_up_cardinality_values1) :-
  % not allowed because cardinality restriction (1,1) can't be specialized to (2,2)
  \+ test_restriction_up_to(planning_test:'TestSomeRestr_2', restriction(planning_test:'testProperty', cardinality(2,2,dul:'Agent')), _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests(knowrob_planning).
