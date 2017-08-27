
:- module(knowrob_assembly,
    [
        assemblage_underspecified/1,
        assemblage_specified/1,
        assemblage_established/1,
        assemblage_parts/2,
        assemblage_atomic_part/2,
        assemblage_parent/2,
        assemblage_blocked_affordance/2,
        assemblage_fixture/1,
        assemblage_graspable/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_planning, 'http://knowrob.org/kb/knowrob_planning.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

:-  rdf_meta
      assemblage_underspecified(r),
      assemblage_specified(r),
      assemblage_established(r),
      assemblage_parts(t),
      assemblage_atomic_part(r,r),
      assemblage_part_blocked_affordance(r,r),
      assemblage_fixture(r),
      assemblage_graspable(r),
      assemblage_parent(r,r).

assemblage_specified(Assemblage) :-
  \+ assemblage_underspecified(Assemblage),
  forall( subassemblage(Assemblage, SubAssemblage),
          assemblage_specified(SubAssemblage) ).

assemblage_underspecified(Assemblage) :-
  owl_unsatisfied_restriction(Assemblage, Restr),
  assemblage_relevant_restriction(Restr), !.
assemblage_underspecified(Assemblage) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  owl_unsatisfied_restriction(Connection, Restr),
  assemblage_relevant_restriction(Restr), !.
assemblage_relevant_restriction(Restr) :-
  rdf_has(Restr, owl:'onProperty', P),
  (
    rdf_equal(P, knowrob_assembly:'hasPart') ;
    rdf_equal(P, knowrob_assembly:'usesConnection') ;
    rdf_equal(P, knowrob_assembly:'consumesAffordance') ;
    rdf_equal(P, knowrob_assembly:'linksAssemblage')
  ).

assemblage_parts(Assemblage, Parts) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  findall([Affordance,Part], (
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance),
    rdf_has(Part, knowrob_assembly:'hasAffordance', Affordance)
  ), Parts).

assemblage_atomic_part(Assemblage, AtomicPart) :-
  ground(AtomicPart), !,
  rdf_has(AtomicPart, knowrob_assembly:'hasAffordance', Affordance),
  rdf_has(Conn, knowrob_assembly:'consumesAffordance', Affordance),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn).
assemblage_atomic_part(Assemblage, AtomicPart) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn),
  rdf_has(Conn, knowrob_assembly:'consumesAffordance', Affordance),
  rdf_has(AtomicPart, knowrob_assembly:'hasAffordance', Affordance).

subassemblage(Assemblage, SubAssemblage) :-
  rdfs_individual_of(Assemblage, Restr),
  rdfs_individual_of(Restr, owl:'Restriction'),
  rdf_has(Restr, owl:'onProperty', knowrob_assembly:'usesConnection'),
  owl_restriction(Restr, restriction(_, cardinality(_,_,Descr))),
  owl_description(Descr, Descr_x),
  ( Descr_x = restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(SubAssemblageType)) ; (
    Descr_x = intersection_of(List),
    member(restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(SubAssemblageType)), List)
  )),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn),
  once((
    owl_has(Conn, knowrob_assembly:'linksAssemblage', SubAssemblage),
    owl_individual_of(SubAssemblage, SubAssemblageType)
  )).

assemblage_parent(Assemblage, Parent) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Conn1),
  rdf_has(Conn1, knowrob_assembly:'consumesAffordance', Aff1),
  rdf_has(Part, knowrob_assembly:'hasAffordance', Aff1),
  rdf_has(Part, knowrob_assembly:'hasAffordance', Aff2),
  Aff1 \= Aff2,
  rdf_has(Conn2, knowrob_assembly:'consumesAffordance', Aff2),
  rdf_has(Parent, knowrob_assembly:'usesConnection', Conn2),
  Parent \= Assemblage,
  once((
    rdfs_individual_of(Parent, Restr),
    rdfs_individual_of(Restr, owl:'Restriction'),
    rdf_has(Restr, owl:'onProperty', knowrob_assembly:'usesConnection'),
    owl_restriction(Restr, restriction(_, cardinality(_,_,Descr))),
    owl_description(Descr, Descr_x),
    ( Descr_x = restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(Cls)) ; (
      Descr_x = intersection_of(List),
      member(restriction('http://knowrob.org/kb/knowrob_assembly.owl#linksAssemblage', some_values_from(Cls)), List)
    )),
    owl_individual_of(Assemblage, Cls)
  )).

assemblage_established(Assemblage) :-
    rdf_has(Assemblage, knowrob_assembly:'assemblageEstablished', literal(type(_,'true'))).

assemblage_fixture(FixedPart) :- rdfs_individual_of(FixedPart, knowrob_assembly:'FixedPart').
assemblage_graspable(_Part)   :- true. % TODO: check if grasp affordance available

assemblage_blocked_affordance(Part, Affordance) :-
  rdf_has(Part, knowrob_assembly:'hasAffordance', Affordance),
  % TODO: handle non connections blocking affordances? (grasps)
  once((
    rdf_has(Connection, knowrob_assembly:'blocksAffordance', Affordance),
    rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
    assemblage_established(Assemblage)
  )).
