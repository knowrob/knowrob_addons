
:- module(cram_assembly,
    [
      cram_assembly_agenda_perform/4
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_assembly')).
:- use_module(library('knowrob_planning')).

:- owl_parser:owl_parse('package://knowrob_assembly/owl/knowrob_assembly.owl').

:- rdf_db:rdf_register_prefix(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).


cram_service_client(Client) :-
  (\+ current_predicate(v_service_client, _)),
  jpl_call('org.knowrob.assembly.CRAMServiceClient', get, [], Client),
  jpl_list_to_array(['org.knowrob.assembly.CRAMServiceClient'], Arr),
  jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Client, Arr], _),
  assert(v_service_client(Client)),!.
cram_service_client(Client) :-
  current_predicate(v_service_client, _),
  v_service_client(Client).

cram_service_perform(Designator) :-
  cram_service_client(Client),
  jpl_call(Client, 'performDesignator', [Designator], _).


cram_assembly_agenda_perform(_, item(integrate,S,P,_,_), Domain, O) :-
  once((
    cram_assembly_part_select(Domain,P,O) ;
    cram_assembly_part_search(Domain,P,O)
  )),
  rdf_assert(S,knowrob_assembly:'consumesAffordance',O),
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', S),
  % perform assembly action designator
  ignore(cram_assembly_perform(Assemblage)),
  % FIXME: looks silly to retract again here, but the fact is asserted later again
  rdf_retractall(S,knowrob_assembly:'consumesAffordance',O).

cram_assembly_part_select(Domain,Predicate,Selection) :-
  % TODO: call a CRAM service that selects an object
  owl_consistent_selection(Domain,Predicate,Selection), !.

cram_assembly_part_search(_Domain,_Predicate,_Selection) :-
  % TODO: perform CRAM perceive plan
  fail.
  
cram_assembly_perform(Assemblage) :-
  \+ assemblage_underspecified(Assemblage),
  assemblage_parts(Assemblage, Parts),
  cram_assembly_perform_(Parts),
  forall(
    assemblage_parent(Assemblage, Parent),
    cram_assembly_perform(Parent)
  ).
  
cram_assembly_perform_(Parts) :-
  % TODO: call perform action designator service
  write('      CRAM: perform assembly '), rdf_write_readable(Parts), nl,
  catch(cram_service_perform('foo bar baz'), _, true).

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

assemblage_parts(Assemblage, Parts) :-
  rdf_has(Assemblage, knowrob_assembly:'usesConnection', Connection),
  findall((Affordance,Part), (
    rdf_has(Connection, knowrob_assembly:'consumesAffordance', Affordance),
    rdf_has(Part, knowrob_assembly:'hasAffordance', Affordance)
  ), Parts).
  
assemblage_underspecified(Assemblage) :-
  owl_unsatisfied_restriction(Assemblage, Restr),
  rdf_has(Restr, owl:'onProperty', P),
  (
    rdf_equal(P, knowrob_assembly:'hasPart') ;
    rdf_equal(P, knowrob_assembly:'usesConnection') ;
    rdf_equal(P, knowrob_assembly:'consumesAffordance') ;
    rdf_equal(P, knowrob_assembly:'linksAssemblage')
  ).
