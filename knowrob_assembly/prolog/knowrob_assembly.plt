:- begin_tests(knowrob_assembly).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/assembly_map_test.owl').
:- owl_parser:owl_parse('package://knowrob_assembly/owl/assembly_test.owl').

:- rdf_db:rdf_register_prefix(assembly_test, 'http://knowrob.org/kb/assembly_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(assembly_map, 'http://knowrob.org/kb/assembly_map_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(assemblages_test, 'http://knowrob.org/kb/assemblages_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(parts, 'http://knowrob.org/kb/assembly_parts_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(params, 'http://knowrob.org/kb/knowrob_paramserver.owl#', [keep(true)]).
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
  owl_individual_of(assembly_map:'Axle1SideGrasp', parts:'AxleSideGrasp'), !.
  
test(assembly_axle_side_grasp_type2, [nondet]) :-
  owl_has(assembly_map:'Axle1', knowrob_assembly:hasAffordance, Affordance),
  owl_individual_of(Affordance, parts:'AxleSideGrasp'), !.

test(assembly_axis_inconsistencies, [nondet]) :-
  owl_individual_of(assembly_map:'Axle1', parts:'Axle'),!,
  unsatisfied_restrictions(assembly_map:'Axle1', []).

test(assembly_axis_inconsistencies, [nondet]) :-
  owl_individual_of(assembly_map:'Wheel1', parts:'Wheel'),!,
  unsatisfied_restrictions(assembly_map:'Wheel1', []).

test(assembly_chassis_inconsistencies, [nondet]) :-
  owl_individual_of(assembly_map:'Chassis1', parts:'Chassis'),!,
  unsatisfied_restrictions(assembly_map:'Chassis1', []).

test(assembly_porsche_inconsistencies, [nondet]) :-
  owl_individual_of(assembly_map:'PorscheBody1', parts:'PorscheBody'),!,
  unsatisfied_restrictions(assembly_map:'PorscheBody1', []).

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
          restr(knowrob_assembly:'linksAssemblage',some,assemblages_test:'AxleWithLeftWheel')]))).
test_ChassisWithFrontAxle_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'AxleSnapInFront',
          restr(knowrob_assembly:'linksAssemblage',some,assemblages_test:'AxleWithWheels')]))).
test_ChassisWithAxles_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'AxleSnapInBack',
          restr(knowrob_assembly:'linksAssemblage',some,assemblages_test:'AxleWithWheels'),
          restr(knowrob_assembly:'linksAssemblage',some,assemblages_test:'ChassisWithFrontAxle')]))).
test_BodyOnChassis_uses_connection(Obj) :-
  test_unsattisfied_restriction(Obj,
    restr(knowrob_assembly:'usesConnection', (1,1), intersect([
          parts:'ChassisSnapInConnection',
          restr(knowrob_assembly:'linksAssemblage',some,assemblages_test:'ChassisWithAxles')]))).

test(assembly_AxleWithLeftWheel_uses_connection1, [nondet]) :-
  test_AxleWithLeftWheel_uses_connection(assembly_test:'AxleWithLeftWheel1'),
  test_AxleWithLeftWheel_uses_connection(assembly_test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_uses_connection1, [nondet]) :-
  test_AxleWithWheels_uses_connection(assembly_test:'AxleWithWheels1'),
  test_AxleWithWheels_uses_connection(assembly_test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_uses_connection1, [nondet]) :-
  test_ChassisWithFrontAxle_uses_connection(assembly_test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_uses_connection1, [nondet]) :-
  test_ChassisWithAxles_uses_connection(assembly_test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_uses_connection1, [nondet]) :-
  test_BodyOnChassis_uses_connection(assembly_test:'BodyOnChassis1').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Test consistency of usesConnection property after asserting connections
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta assert_connection(r,r,t),
            assert_connection(r,r).
assert_connection(Part, Conn, SubAssemblies) :-
  rdf_instance_from_class(Conn, Conn_Instance),
  rdf_assert(Part, knowrob_assembly:usesConnection, Conn_Instance),
  forall(member(SubAssembly,SubAssemblies),
         rdf_assert(Conn_Instance, knowrob_assembly:'linksAssemblage', SubAssembly)).
assert_connection(Part, Conn) :-
  rdf_instance_from_class(Conn, Conn_Instance),
  rdf_assert(Part, knowrob_assembly:usesConnection, Conn_Instance).
test(assembly_assert_connections) :-
  assert_connection(assembly_test:'AxleWithLeftWheel1',    parts:'WheelSnapInOnLeft'),
  assert_connection(assembly_test:'AxleWithLeftWheel2',    parts:'WheelSnapInOnLeft'),
  assert_connection(assembly_test:'AxleWithWheels1',       parts:'WheelSnapInOnRight',      [assembly_test:'AxleWithLeftWheel1']),
  assert_connection(assembly_test:'AxleWithWheels2',       parts:'WheelSnapInOnRight',      [assembly_test:'AxleWithLeftWheel2']),
  assert_connection(assembly_test:'ChassisWithFrontAxle1', parts:'AxleSnapInFront',         [assembly_test:'AxleWithWheels1']),
  assert_connection(assembly_test:'ChassisWithAxles1',     parts:'AxleSnapInBack',          [assembly_test:'AxleWithWheels1', assembly_test:'ChassisWithFrontAxle1']),
  assert_connection(assembly_test:'BodyOnChassis1',        parts:'ChassisSnapInConnection', [assembly_test:'ChassisWithAxles1']).

test(assembly_AxleWithLeftWheel_uses_connection2, [nondet]) :-
  \+ test_AxleWithLeftWheel_uses_connection(assembly_test:'AxleWithLeftWheel1'),
  \+ test_AxleWithLeftWheel_uses_connection(assembly_test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_uses_connection2, [nondet]) :-
  \+ test_AxleWithWheels_uses_connection(assembly_test:'AxleWithWheels1'),
  \+ test_AxleWithWheels_uses_connection(assembly_test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_uses_connection2, [nondet]) :-
  \+ test_ChassisWithFrontAxle_uses_connection(assembly_test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_uses_connection2, [nondet]) :-
  \+ test_ChassisWithAxles_uses_connection(assembly_test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_uses_connection2, [nondet]) :-
  \+ test_BodyOnChassis_uses_connection(assembly_test:'BodyOnChassis1').

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
  test_Wheel_needsAffordance(assembly_test:'AxleWithLeftWheel1'),
  test_Wheel_needsAffordance(assembly_test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_needsAffordance1, [nondet]) :-
  test_Wheel_needsAffordance(assembly_test:'AxleWithWheels1'),
  test_Wheel_needsAffordance(assembly_test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_needsAffordance1, [nondet]) :-
  test_Axle_needsAffordance(assembly_test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_needsAffordance1, [nondet]) :-
  test_Axle_needsAffordance(assembly_test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_needsAffordance1, [nondet]) :-
  test_BodyOnChassis_needsAffordance(assembly_test:'BodyOnChassis1').
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta assert_affordances(r,t),
            fully_specified(r).

assert_affordances(Part, Xs) :-
  rdf_has(Part, knowrob_assembly:usesConnection, Conn),
  forall(member(X,Xs), rdf_assert(Conn, knowrob_assembly:consumesAffordance, X)).
test(assembly_assert_affordances, [nondet]) :-
  assert_affordances(assembly_test:'AxleWithLeftWheel1', [assembly_map:'Axle1WheelSnapInMLeft', assembly_map:'Wheel1SnapInF']),
  assert_affordances(assembly_test:'AxleWithLeftWheel2', [assembly_map:'Axle2WheelSnapInMLeft', assembly_map:'Wheel3SnapInF']),
  assert_affordances(assembly_test:'AxleWithWheels1', [assembly_map:'Axle1WheelSnapInMRight', assembly_map:'Wheel2SnapInF']),
  assert_affordances(assembly_test:'AxleWithWheels2', [assembly_map:'Axle2WheelSnapInMRight', assembly_map:'Wheel4SnapInF']),
  assert_affordances(assembly_test:'ChassisWithFrontAxle1', [assembly_map:'Axle1SnapInM', assembly_map:'Chassis1AxleSnapInFFront']),
  assert_affordances(assembly_test:'ChassisWithAxles1', [assembly_map:'Axle2SnapInM', assembly_map:'Chassis1AxleSnapInFBack']),
  assert_affordances(assembly_test:'BodyOnChassis1', [assembly_map:'Chassis1BodySnapInM', assembly_map:'PorscheBody1ChassisSnapInF']).

test(assembly_AxleWithLeftWheel_needsAffordance2, [nondet]) :-
  \+ test_Wheel_needsAffordance(assembly_test:'AxleWithLeftWheel1'),
  \+ test_Wheel_needsAffordance(assembly_test:'AxleWithLeftWheel2').
test(assembly_AxleWithWheels_needsAffordance2, [nondet]) :-
  \+ test_Wheel_needsAffordance(assembly_test:'AxleWithWheels1'),
  \+ test_Wheel_needsAffordance(assembly_test:'AxleWithWheels2').
test(assembly_ChassisWithFrontAxle_needsAffordance2, [nondet]) :-
  \+ test_Axle_needsAffordance(assembly_test:'ChassisWithFrontAxle1').
test(assembly_ChassisWithAxles_needsAffordance2, [nondet]) :-
  \+ test_Axle_needsAffordance(assembly_test:'ChassisWithAxles1').
test(assembly_BodyOnChassis_needsAffordance2, [nondet]) :-
  \+ test_BodyOnChassis_needsAffordance(assembly_test:'BodyOnChassis1').

fully_specified(Obj) :-
  once((unsatisfied_restrictions(Obj, []) ; (
    % debug unsattisfied restrictions
    write('obj: '), writeln(Obj),
    unsatisfied_restrictions(Obj, Restr),
    forall(member(R,Restr), (write('  '), writeln(R))),
    forall(owl_has(Obj,knowrob_assembly:hasPart,O), (write('  part:'), writeln(O))),
    fail
  ))).
  
test(specified_AxleWithLeftWheel1) :-    fully_specified(assembly_test:'AxleWithLeftWheel1').
test(specified_AxleWithLeftWheel2) :-    fully_specified(assembly_test:'AxleWithLeftWheel2').
test(specified_AxleWithWheels1) :-       fully_specified(assembly_test:'AxleWithWheels1').
test(specified_AxleWithWheels2) :-       fully_specified(assembly_test:'AxleWithWheels2').
test(specified_ChassisWithFrontAxle) :-  fully_specified(assembly_test:'ChassisWithFrontAxle1').
test(specified_ChassisWithAxles) :-      fully_specified(assembly_test:'ChassisWithAxles1').
test(specified_BodyOnChassis) :-         fully_specified(assembly_test:'BodyOnChassis1').

test(retract_some_facts) :-
  rdf_retractall(_, knowrob_assembly:'linksAssemblage', _),
  rdf_retractall(_, knowrob_assembly:'needsAffordance', _),
  rdf_retractall(_, knowrob_assembly:'usesConnection', _),
  rdf_retractall(_, knowrob_assembly:'consumesAffordance', _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(owl_most_specific_specializations5) :-
  owl_most_specific([parts:'ChassisSnapInConnection'], Specific),
  rdf_equal(parts:'ChassisSnapInConnection', Specific).

:- rdf_meta test_restriction_up_to(r,t,t).
test_restriction_up_to(S,Restr,UpTo) :-
  findall( (S,Restr,UpTo), owl_satisfies_restriction_up_to(S,Restr,UpTo), Xs),
  member( (S,Restr,UpTo), Xs ).

%test(chassis_snap_connection_unstattisfied_up_to1) :-
  %rdf_instance_from_class(parts:'ChassisSnapInConnection', Conn),
  %test_restriction_up_to(Conn, intersection_of([
    %class(parts:'ChassisSnapInConnection'),
    %restriction(knowrob_assembly:'linksAssemblage',some_values_from(assemblages_test:'ChassisWithAxles'))]),
    %integrate(Conn,knowrob_assembly:'needsAffordance',_,1)).
    
test(assembly_PorscheBody1SideGrasp_not_specializable1) :-
  \+ owl_specializable(assembly_map:'PorscheBody1SideGrasp', parts:'AxleSnapInAffordance').
    
test(assembly_PorscheBody1SideGrasp_not_specializable2) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  owl_restriction_assert(restriction(NeedsInv, some_values_from(
                         restriction(UsesInv,  some_values_from(assemblages_test:'ChassisWithAxles')))), Restr),
  % o ChassisWithAxles usesConnection only(AxleSnapInBack)
  % o AxleSnapInBack consumesAffordance_1(AxleSnapInFBack), needsAffordance_only(AxleSnapInAffordance)
  % --> fail because PorscheBody1SideGrasp not specializable to AxleSnapInAffordance
  \+ owl_specializable(assembly_map:'PorscheBody1SideGrasp', Restr).

test(assembly_AxleSnapInBack_needsAffordance_domain) :-
  % o AxleSnapInBack needsAffordance only AxleSnapInAffordance
  % o {AxleSnapInF,AxleSnapInM} subClassOf AxleSnapInBack
  % o {AxleSnapInFBack,AxleSnapInFFront} subClassOf AxleSnapInF
  % o AxleSnapInBack consumesAffordance 1 AxleSnapInFBack
  % o AxleSnapInBack consumesAffordance 1 AxleSnapInF
  % o AxleSnapInBack consumesAffordance 1 AxleSnapInM
  % --> infer that range(needsAffordance) = (AxleSnapInFBack or AxleSnapInM)
  owl_property_range_on_class(parts:'AxleSnapInBack', knowrob_assembly:'needsAffordance', Range),
  owl_description(Range, union_of(List)),
  once(( member(X,Ranges), rdf_equal(X,parts:'AxleSnapInFBack') )),
  once(( member(Y,Ranges), rdf_equal(Y,parts:'AxleSnapInM') )),
  length(List,2).

% o ChassisWithAxles usesConnection only(AxleSnapInBack)
% o AxleSnapInBack consumesAffordance 1 AxleSnapInFBack
% o AxleSnapInBack consumesAffordance 1 AxleSnapInF
% o AxleSnapInBack consumesAffordance 1 AxleSnapInM
% --> fail because AxleSnapInFFront not in range(AxleSnapInBack,needsAffordance)
test(assembly_Chassis1AxleSnapInFBack_specializable) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  owl_restriction_assert(restriction(NeedsInv, some_values_from(
                         restriction(UsesInv,  some_values_from(assemblages_test:'ChassisWithAxles')))), Restr),
  owl_specializable(assembly_map:'Chassis1AxleSnapInFBack', Restr).
test(assembly_Chassis1AxleSnapInFFront_not_specializable) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  owl_restriction_assert(restriction(NeedsInv, some_values_from(
                         restriction(UsesInv,  some_values_from(assemblages_test:'ChassisWithAxles')))), Restr),
  \+ owl_specializable(parts:'AxleSnapInFFront', Restr).
  
% o CarBody hasAffordance 1 BodyChassisSnapInF
% o BodyChassisSnapInF subClassOf BodySnapInAffordance
% o ChassisSnapInConnection needsAffordance only BodySnapInAffordance
test(assembly_ChassisSnapInConnection_specializable1) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#hasAffordance', HasInv),
  owl_restriction_assert(restriction(knowrob_assembly:'needsAffordance', some_values_from(
                         restriction(HasInv,
                         cardinality(1, 1, parts:'CarBody')))), Restr),
  owl_specializable(parts:'ChassisSnapInConnection', Restr).
test(assembly_ChassisSnapInConnection_specializable2) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#hasAffordance', HasInv),
  owl_restriction_assert(restriction(knowrob_assembly:'needsAffordance', some_values_from(
                                      restriction(HasInv,
                                      cardinality(1, 1, parts:'CarBody')))), Restr),
  rdf_instance_from_class(parts:'ChassisSnapInConnection', ChassisSnapInConnection),
  owl_specializable(ChassisSnapInConnection, Restr).

test(assembly_PorscheBody1ChassisSnapInF_not_specializable1) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  owl_restriction_assert(restriction(knowrob_assembly:'hasAffordance', some_values_from(
                                      restriction(NeedsInv, some_values_from(
                                      restriction(UsesInv,  some_values_from(assemblages_test:'ChassisWithAxles')))))), Restr),
  % o ChassisWithAxles usesConnection AxleSnapInBack
  % o AxleSnapInBack needsAffordance AxleSnapInAffordance
  % o PorscheBody1 hasAffordance 1 PorscheBody1ChassisSnapInF
  % o CarBody hasAffordance CarBodyAffordance
  % ---> fail because PorscheBody can only have CarBodyAffordance, no AxleSnapInAffordance allowed
  \+ owl_specializable(parts:'PorscheBody', Restr).

test(assembly_PorscheBody1ChassisSnapInF_not_specializable1) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#hasAffordance', HasInv),
  owl_restriction_assert(restriction(HasInv, some_values_from(
                                      restriction(knowrob_assembly:'hasAffordance', some_values_from(
                                      restriction(NeedsInv, some_values_from(
                                      restriction(UsesInv,  some_values_from(assemblages_test:'ChassisWithAxles')))))))), Restr),
  \+ owl_specializable(assembly_map:'PorscheBody1ChassisSnapInF', Restr).
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Test generating agenda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- rdf_meta test_agenda_items(t).
test_agenda_items(Assumed) :-
  assembly_test_agenda(Agenda),
  agenda_items_sorted(Agenda,Actual),
  test_agenda_items(Actual,Assumed), !.
test_agenda_items([],[]).
test_agenda_items([A|As],[B|Bs]) :-
  agenda_item_description(A,B),
  test_agenda_items(As,Bs).


test(assembly_agenda_create_BodyOnChassis) :-
  agenda_create(assembly_test:'BodyOnChassis_a', assembly_test:'AgendaStrategy_1', Agenda),
  once(( rdfs_individual_of(Agenda, knowrob_planning:'Agenda'),
         rdf_has(Agenda, knowrob_planning:'strategy', assembly_test:'AgendaStrategy_1') )),
  assertz(assembly_test_agenda(Agenda)),
  agenda_write(Agenda),
  test_agenda_items([
      item(decompose,assembly_test:'BodyOnChassis_a',knowrob_assembly:'usesConnection',_,_), % hasPart CarBody
      item(decompose,assembly_test:'BodyOnChassis_a',knowrob_assembly:'usesConnection',_,_)  % usesConnection (ChassisSnapInConnection and ...)
  ]).

test(assembly_perform_BodyOnChassis_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
  test_agenda_items([
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', parts:'BodyChassisSnapInF', _),
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', parts:'BodyChassisSnapInM', _),
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'needsAffordance', _, _), % hasPart CarBody
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'needsAffordance', _,_)   % linksAssemblage ChassisWithAxles
  ]).

test(assembly_ChassisSnapInConnection_consumesAffordance_BodyChassisSnapInF) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF')
  )),
  test_agenda_items([
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', parts:'BodyChassisSnapInM', _),
      item(integrate,ChassisSnapInConnection, knowrob_assembly:'needsAffordance', _,_)   % linksAssemblage ChassisWithAxles
  ]).

test(assembly_ChassisSnapInConnection_consumesAffordance_BodyChassisSnapInM) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM')
  )),
  test_agenda_items([
      item(decompose,assembly_map:'Chassis1AxleSnapInFBack', NeedsInv, _, _) % linksAssemblage ChassisWithAxles
  ]).

test(assembly_Chassis1AxleSnapInFBack_inv_needsAffordance) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack')
  )),
  test_agenda_items([
      item(decompose, AxleSnapInBack, UsesInv, assemblages_test:'ChassisWithAxles', _), % linksAssemblage ChassisWithAxles
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _)
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack')
  )),
  test_agenda_items([
      item(decompose, assembly_map:'Chassis1AxleSnapInFFront', NeedsInv, _, _),   % linksAssemblage ChassisWithFrontAxle
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)    % linksAssemblage AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(AxleSnapInFront, knowrob_assembly:'consumesAffordance', assembly_map:'Chassis1AxleSnapInFFront')
  )),
  test_agenda_items([
      item(decompose, AxleSnapInFront, UsesInv, _, _),   % linksAssemblage ChassisWithFrontAxle
      item(integrate, AxleSnapInFront, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)    % linksAssemblage AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(AxleSnapInFront, knowrob_assembly:'consumesAffordance', assembly_map:'Chassis1AxleSnapInFFront')
  )),
  test_agenda_items([
      item(integrate, AxleSnapInFront, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInFront, knowrob_assembly:'needsAffordance', _, _),  % linksAssemblage AxleWithWheels
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)    % linksAssemblage AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack')
  )),
  test_agenda_items([
      item(decompose, assembly_map:'Axle1WheelSnapInMRight', NeedsInv, _, _),               % linksAssemblage AxleWithWheels
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)    % linksAssemblage AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMRight')
  )),
  test_agenda_items([
      item(decompose, WheelSnapInOnRight, UsesInv, assemblages_test:'AxleWithWheels', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)    % linksAssemblage AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMRight')
  )),
  test_agenda_items([
      item(decompose, assembly_map:'Axle1WheelSnapInMLeft', NeedsInv, _, _),                   % AxleWithLeftWheel
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _),  % Wheel
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)       % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMRight'),
    rdf_has(WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMLeft')
  )),
  test_agenda_items([
      item(decompose, WheelSnapInOnLeft, UsesInv, assemblages_test:'AxleWithLeftWheel', _),
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _),  % Wheel
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)       % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMRight'),
    rdf_has(WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMLeft')
  )),
  test_agenda_items([
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'needsAffordance', _, _),  % Wheel
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _),  % Wheel
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)       % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack'),
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle1WheelSnapInMRight')
  )),
  test_agenda_items([
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _),  % Wheel
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)       % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(assembly_test:'BodyOnChassis_a', knowrob_assembly:'usesConnection', ChassisSnapInConnection),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInF),
    rdfs_individual_of(BodyChassisSnapInF, parts:'BodyChassisSnapInF'),
    rdf_has(ChassisSnapInConnection, knowrob_assembly:'consumesAffordance', BodyChassisSnapInM),
    rdfs_individual_of(BodyChassisSnapInM, parts:'BodyChassisSnapInM'),
    rdf_has(AxleSnapInBack, knowrob_assembly:'needsAffordance', assembly_map:'Chassis1AxleSnapInFBack')
  )),
  test_agenda_items([
      item(integrate, AxleSnapInBack, knowrob_assembly:'consumesAffordance', parts:'AxleSnapInM', _),
      item(integrate, AxleSnapInBack, knowrob_assembly:'needsAffordance', _, _)       % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_consumesAffordance_AxleSnapInM) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  test_agenda_items([
      item(decompose, assembly_map:'Axle2WheelSnapInMRight', NeedsInv, _, _) % AxleWithWheels
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMRight')
  )),
  test_agenda_items([
      item(decompose, WheelSnapInOnRight, UsesInv, assemblages_test:'AxleWithWheels', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _)
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#needsAffordance', NeedsInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMRight')
  )),
  test_agenda_items([
      item(decompose, assembly_map:'Axle2WheelSnapInMLeft', NeedsInv, _, _), % AxleWithLeftWheel
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _)       % Wheel
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  owl_inverse_property('http://knowrob.org/kb/knowrob_assembly.owl#usesConnection', UsesInv),
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMRight'),
    rdf_has(WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMLeft')
  )),
  test_agenda_items([
      item(decompose, WheelSnapInOnLeft, UsesInv, _, _),       % Wheel
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _)       % Wheel
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMRight'),
    rdf_has(WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMLeft')
  )),
  test_agenda_items([
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnLeft, knowrob_assembly:'needsAffordance', _, _),      % Wheel
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _)       % Wheel
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  once((
    rdf_has(WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', assembly_map:'Axle2WheelSnapInMRight')
  )),
  test_agenda_items([
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'consumesAffordance', parts:'WheelSnapInF', _),
      item(integrate, WheelSnapInOnRight, knowrob_assembly:'needsAffordance', _, _)       % Wheel
  ]).

test(assembly_AxleSnapInBack_inv_usesConnection) :-
  assembly_test_agenda(Agenda),
  agenda_perform_next(Agenda),
  agenda_write(Agenda),
  test_agenda_items([]).

:- end_tests(knowrob_assembly).
