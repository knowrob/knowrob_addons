:- module(comp_execution_trace,
    [
      ctask/2,
      record/1	
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(execution_trace, 'http://ias.cs.tum.edu/kb/action_visulization.owl#', [keep(true)]).


ctask(Start, End) :-
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_action_visulization.ROSClient', ['my_client'], Client),

    jpl_call(Client, 'getTrace', [], Returned_Power),

    owl_parser:owl_parse('/home/asil/Desktop/deneme/deneme1.owl', false, false, true),
    visualisation_canvas(C).

record(List) :-
    [X|Y] = List,

    term_to_atom(X, X1),

    rdf_assert(X1, rdf:type, execution_trace:'ExecTask'),
    record(Y).

