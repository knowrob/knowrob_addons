:- begin_tests(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://thorin_simulation/owl/thorin_simulation.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/assembly_test.owl').

:- rdf_db:rdf_register_prefix(test, 'http://knowrob.org/kb/assembly_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(sim, 'http://knowrob.org/kb/thorin_simulation.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(assembly, 'http://knowrob.org/kb/thorin_assemblages.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(params, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(parts, 'http://knowrob.org/kb/thorin_parts.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).


owl_restriction_pl(RestrictionID, restr(Property, Restriction, Value_term)) :-
	rdf_has(RestrictionID, owl:onProperty, Property),
	restriction_facet(RestrictionID, Restriction, Value),
	restriction_class(Value, Value_term), !.
owl_restriction_pl(RestrictionID,RestrictionID).


restriction_facet(RestrictionID, R, Class) :-
	(   rdf_has(RestrictionID, owl:allValuesFrom, Class)
	->  R = only
	;   rdf_has(RestrictionID, owl:someValuesFrom, Class)
	->  R = some
	).
restriction_facet(RestrictionID, has_value, Value) :-
	rdf_has(RestrictionID, owl:hasValue, Value).
restriction_facet(R, (Min,Max), Class) :-
	(   rdf_has(R, owl:qualifiedCardinality, literal(Atom))
	->  non_negative_integer(Atom, Min, R, owl:qualifiedCardinality),
	    Max = Min
	;   rdf_has(R, owl:minCardinality, literal(MinAtom))
	->  non_negative_integer(MinAtom, Min, R, owl:minCardinality),
	    (   rdf_has(R, owl:maxCardinality, literal(MaxAtom))
	    ->  non_negative_integer(MaxAtom, Max, R, owl:maxCardinality)
	    ;	Max = inf
	    )
	;   rdf_has(R, owl:maxCardinality, literal(MaxAtom))
	->  non_negative_integer(MaxAtom, Max, R, owl:maxCardinality),
	    Min = 0
	),
	rdf_has(R, owl:onClass, Class).


restriction_class(Cls, intersect(Xs)) :-
	rdf_has(Cls, owl:intersectionOf, Intersect), !,
	rdfs_list_to_prolog_list(Intersect, Classes),
	findall(X, (
		member(R,Classes), 
		owl_restriction_pl(R,X)
	), Xs).
restriction_class(Cls, union(Xs)) :-
	rdf_has(Cls, owl:unionOf, Union), !,
	swrl:rdf_class_list_pl(Union, Classes),
	findall(X, (
		member(R,Classes), 
		owl_restriction_pl(R,X)
	), Xs).
restriction_class(Cls, Cls).


:- rdf_meta unsatisfied_restrictions(r,t).
unsatisfied_restrictions(Object, Restrictions) :-
  findall(Restr, (
      owl_unsatisfied_restriction(Object, Descr),
      owl_restriction_pl(Descr, Restr)
  ), Restrictions).


test(assembly_axle_dae_type, [nondet]) :-
  owl_individual_of(parts:'AxleDAE', params:'ReferenceShapeData'),!.
  
test(assembly_axle_side_grasp_type1, [nondet]) :-
  owl_individual_of(sim:'Axle1SideGrasp', parts:'AxleSideGrasp'), !.
  
test(assembly_axle_side_grasp_type2, [nondet]) :-
  owl_has(sim:'Axle1', knowrob_assembly:hasAffordance, Affordance),
  owl_individual_of(Affordance, parts:'AxleSideGrasp'), !.

test(assembly_axis_inconsistencies, [nondet]) :-
  owl_individual_of(sim:'Axle1', parts:'Axle'),!,
  unsatisfied_restrictions(sim:'Axle1', []).

test(assembly_axis_inconsistencies, [nondet]) :-
  owl_individual_of(sim:'Wheel1', parts:'Wheel'),!,
  unsatisfied_restrictions(sim:'Wheel1', []).

test(assembly_chassis_inconsistencies, [nondet]) :-
  owl_individual_of(sim:'Chassis1', parts:'Chassis'),!,
  unsatisfied_restrictions(sim:'Chassis1', []).

test(assembly_porsche_inconsistencies, [nondet]) :-
  owl_individual_of(sim:'PorscheBody1', parts:'PorscheBody'),!,
  unsatisfied_restrictions(sim:'PorscheBody1', []).

:- rdf_meta test_unsattisfied_restriction(r,t),
            test_unsattisfied_restricted_properties(r,t).
test_unsattisfied_restriction(Obj, Restr) :-
  unsatisfied_restrictions(Obj, Restrictions),
  member(Restr, Restrictions).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Test inconsistency of usesConnection property of assemblages
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta test_AxleWithLeftWheel_uses_connection(r),
            test_AxleWithWheels_uses_connection(r),
            test_ChassisWithFrontAxle_uses_connection(r),
            test_ChassisWithAxles_uses_connection(r),
            test_BodyOnChassis_uses_connection(r).
test_AxleWithLeftWheel_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), parts:'WheelSnapInOnLeft')).
test_AxleWithWheels_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'WheelSnapInOnRight',
          restr(knowrob_assembly:'linksAssemblage',some,assembly:'AxleWithLeftWheel')]))).
test_ChassisWithFrontAxle_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'AxleSnapInFront',
          restr(knowrob_assembly:'linksAssemblage',some,assembly:'AxleWithWheels')]))).
test_ChassisWithAxles_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'AxleSnapInBack',
          restr(knowrob_assembly:'linksAssemblage',some,assembly:'AxleWithWheels'),
          restr(knowrob_assembly:'linksAssemblage',some,assembly:'ChassisWithFrontAxle')]))).
test_BodyOnChassis_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'ChassisSnapInConnection',
          restr(knowrob_assembly:'linksAssemblage',some,assembly:'ChassisWithAxles')]))).

test(assembly_AxleWithLeftWheel_uses_connection1, [nondet]) :-
  test_AxleWithLeftWheel_uses_connection(test:'AxleWithLeftWheel1'),
  test_AxleWithLeftWheel_uses_connection(test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_uses_connection1, [nondet]) :-
  test_AxleWithWheels_uses_connection(test:'AxleWithWheels1'),
  test_AxleWithWheels_uses_connection(test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_uses_connection1, [nondet]) :-
  test_ChassisWithFrontAxle_uses_connection(test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_uses_connection1, [nondet]) :-
  test_ChassisWithAxles_uses_connection(test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_uses_connection1, [nondet]) :-
  test_BodyOnChassis_uses_connection(test:'BodyOnChassis1').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Test consistency of usesConnection property after asserting connections
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta assert_connection(r,r,t),
            assert_connection(r,r).
assert_connection(Part, Conn, SubAssemblies) :-
  rdf_instance_from_class(Conn, Conn_Instance),
  rdf_assert(Part, knowrob_assembly:usesConnection, Conn_Instance),
  % TODO: this is a bit ugly since we not only have to create an individual but also to integrate.
  %       should be done with two individual items, but the part that offers the connection restricts the linksAssemblage.
  %       GENERIC APPROACH: Compute range by taking into account restrictions imposed by related instances?
  %                         XXX: how deep should we look?
  %                   e.g.: connection has unspecified linksAssemblage, is used by a part and part restricts linksAssemblage to given type.
  % APPROACH: specify usesConnection, make consistency check again, find out that the restriction is still not fullfilled,
  %           then go deeper in the restriction.
  forall(member(SubAssembly,SubAssemblies),
         rdf_assert(Conn_Instance, knowrob_assembly:'linksAssemblage', SubAssembly)).
assert_connection(Part, Conn) :-
  rdf_instance_from_class(Conn, Conn_Instance),
  rdf_assert(Part, knowrob_assembly:usesConnection, Conn_Instance).
test(assert_connections) :-
  assert_connection(test:'AxleWithLeftWheel1',    parts:'WheelSnapInOnLeft'),
  assert_connection(test:'AxleWithLeftWheel2',    parts:'WheelSnapInOnLeft'),
  assert_connection(test:'AxleWithWheels1',       parts:'WheelSnapInOnRight',      [test:'AxleWithLeftWheel1']),
  assert_connection(test:'AxleWithWheels2',       parts:'WheelSnapInOnRight',      [test:'AxleWithLeftWheel2']),
  assert_connection(test:'ChassisWithFrontAxle1', parts:'AxleSnapInFront',         [test:'AxleWithWheels1']),
  assert_connection(test:'ChassisWithAxles1',     parts:'AxleSnapInBack',          [test:'AxleWithWheels1', test:'ChassisWithFrontAxle1']),
  assert_connection(test:'BodyOnChassis1',        parts:'ChassisSnapInConnection', [test:'ChassisWithAxles1']).

test(assembly_AxleWithLeftWheel_uses_connection2, [nondet]) :-
  \+ test_AxleWithLeftWheel_uses_connection(test:'AxleWithLeftWheel1'),
  \+ test_AxleWithLeftWheel_uses_connection(test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_uses_connection2, [nondet]) :-
  \+ test_AxleWithWheels_uses_connection(test:'AxleWithWheels1'),
  \+ test_AxleWithWheels_uses_connection(test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_uses_connection2, [nondet]) :-
  \+ test_ChassisWithFrontAxle_uses_connection(test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_uses_connection2, [nondet]) :-
  \+ test_ChassisWithAxles_uses_connection(test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_uses_connection2, [nondet]) :-
  \+ test_BodyOnChassis_uses_connection(test:'BodyOnChassis1').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Test consistency of consumesAffordance property of created connections
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta test_Wheel_needsAffordance(r),
            test_Axle_needsAffordance(r),
            test_BodyOnChassis_needsAffordance(r).
test_Wheel_needsAffordance(Obj) :-
  rdf_has(Obj, knowrob_assembly:usesConnection, Conn),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'WheelSnapInM')),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'WheelSnapInF')).
test_Axle_needsAffordance(Obj) :-
  rdf_has(Obj, knowrob_assembly:usesConnection, Conn),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'AxleSnapInM')),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'AxleSnapInF')).
test_BodyOnChassis_needsAffordance(Obj) :-
  rdf_has(Obj, knowrob_assembly:usesConnection, Conn),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'BodyChassisSnapInM')),
  test_unsattisfied_restriction(Conn,
    restr(knowrob_assembly:'consumesAffordance', (1,1), parts:'BodyChassisSnapInF')).

test(assembly_AxleWithLeftWheel_needsAffordance1, [nondet]) :-
  test_Wheel_needsAffordance(test:'AxleWithLeftWheel1'),
  test_Wheel_needsAffordance(test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_needsAffordance1, [nondet]) :-
  test_Wheel_needsAffordance(test:'AxleWithWheels1'),
  test_Wheel_needsAffordance(test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_needsAffordance1, [nondet]) :-
  test_Axle_needsAffordance(test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_needsAffordance1, [nondet]) :-
  test_Axle_needsAffordance(test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_needsAffordance1, [nondet]) :-
  test_BodyOnChassis_needsAffordance(test:'BodyOnChassis1').
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta assert_affordances(r,t),
            fully_specified(r).

assert_affordances(Part, Xs) :-
  rdf_has(Part, knowrob_assembly:usesConnection, Conn),
  forall(member(X,Xs), rdf_assert(Conn, knowrob_assembly:consumesAffordance, X)).
test(assert_affordances, [nondet]) :-
  assert_affordances(test:'AxleWithLeftWheel1', [sim:'Axle1WheelSnapInMLeft', sim:'Wheel1SnapInF']),
  assert_affordances(test:'AxleWithLeftWheel2', [sim:'Axle2WheelSnapInMLeft', sim:'Wheel3SnapInF']),
  assert_affordances(test:'AxleWithWheels1', [sim:'Axle1WheelSnapInMRight', sim:'Wheel2SnapInF']),
  assert_affordances(test:'AxleWithWheels2', [sim:'Axle2WheelSnapInMRight', sim:'Wheel4SnapInF']),
  assert_affordances(test:'ChassisWithFrontAxle1', [sim:'Axle1SnapInM', sim:'Chassis1AxleSnapInFFront']),
  assert_affordances(test:'ChassisWithAxles1', [sim:'Axle2SnapInM', sim:'Chassis1AxleSnapInFBack']),
  assert_affordances(test:'BodyOnChassis1', [sim:'Chassis1BodySnapInM', sim:'PorscheBody1ChassisSnapInF']).

test(assembly_AxleWithLeftWheel_needsAffordance2, [nondet]) :-
  \+ test_Wheel_needsAffordance(test:'AxleWithLeftWheel1'),
  \+ test_Wheel_needsAffordance(test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_needsAffordance2, [nondet]) :-
  \+ test_Wheel_needsAffordance(test:'AxleWithWheels1'),
  \+ test_Wheel_needsAffordance(test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_needsAffordance2, [nondet]) :-
  \+ test_Axle_needsAffordance(test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_needsAffordance2, [nondet]) :-
  \+ test_Axle_needsAffordance(test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_needsAffordance2, [nondet]) :-
  \+ test_BodyOnChassis_needsAffordance(test:'BodyOnChassis1').

fully_specified(Obj) :-
  once((unsatisfied_restrictions(Obj, []) ; (
    % debug unsattisfied restrictions
    write('obj: '), writeln(Obj),
    unsatisfied_restrictions(Obj, Restr),
    forall(member(R,Restr), (write('  '), writeln(R))),
    fail
  ))).
  
test(specified_AxleWithLeftWheel1) :-    fully_specified(test:'AxleWithLeftWheel1').
test(specified_AxleWithLeftWheel2) :-    fully_specified(test:'AxleWithLeftWheel2').
test(specified_AxleWithWheels1) :-       fully_specified(test:'AxleWithWheels1').
test(specified_AxleWithWheels2) :-       fully_specified(test:'AxleWithWheels2').
test(specified_ChassisWithFrontAxle) :-  fully_specified(test:'ChassisWithFrontAxle1').
test(specified_ChassisWithAxles) :-      fully_specified(test:'ChassisWithAxles1').
test(specified_BodyOnChassis) :-         fully_specified(test:'BodyOnChassis1').
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Test generating agenda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(assembly_agenda_assemblages1) :-
  agenda(test:'BodyOnChassis_a', Agenda0),
  agenda_write(Agenda0),
  agenda_item(Agenda0, classify(Conn0, parts:'ChassisSnapInConnection')),
  agenda_item(Agenda0, decompose(test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', Conn0)),
  % manually process the items
  rdf_assert(Conn0, rdf:type, parts:'ChassisSnapInConnection'),
  rdf_assert(test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', Conn0),
  % FIXME: something not working here classify/decompose connection still there and
  %        no item generated for linksAssemblage!
  agenda(test:'BodyOnChassis_a', Agenda1),
  agenda_write(Agenda1),
  \+ agenda_item(Agenda1, classify(Conn1, parts:'ChassisSnapInConnection')),
  \+ agenda_item(Agenda1, decompose(test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', Conn1)).

:- end_tests(knowrob_assembly).
