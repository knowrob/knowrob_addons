
:- module(knowrob_battat,
    [
        battat_initialize/0,
        battat_initialize_sim/0
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).

battat_initialize :-
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_test.owl'),
  owl_parser:owl_parse('package://knowrob_srdl/owl/Boxy_08_2016.owl').

battat_initialize_sim :-
  battat_initialize,
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl').

