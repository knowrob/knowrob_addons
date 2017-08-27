
:- module(knowrob_battat,
    [
        battat_initialize/0,
        battat_initialize_sim/0,
        battat_sim_plane_complete/0,
        battat_sim_plane_connection/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).

:- rdf_db:rdf_register_prefix(battat_toys, 'http://knowrob.org/kb/battat_toys.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_test, 'http://knowrob.org/kb/battat_airplane_test.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(battat_sim, 'http://knowrob.org/kb/battat_airplane_simulation.owl#', [keep(true)]).

:-  rdf_meta
      battat_sim_plane_connection(r,r,t).

battat_initialize :-
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_toys.owl'),
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_test.owl'),
  owl_parser:owl_parse('package://knowrob_srdl/owl/Boxy_08_2016.owl').

battat_initialize_sim :-
  battat_initialize,
  owl_parser:owl_parse('package://knowrob_assembly/owl/battat_airplane_simulation.owl').

battat_sim_plane_complete :-
  battat_sim_plane_connection(battat_toys:'BottomWingSlideInChassis', battat_sim:'PlaneBottomWing_1',
      [battat_sim:'PlaneChassis_1',battat_sim:'PlaneBottomWing_1']),
  battat_sim_plane_connection(battat_toys:'UnderBodySlideInFrame', battat_sim:'PlaneUnderBody_1',
      [battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneChassis_1',battat_sim:'PlaneRearWing_1',battat_sim:'PlaneBottomWing_1']),
  battat_sim_plane_connection(battat_toys:'MotorGrillSlideInUnderBody', battat_sim:'PlaneMotorGrill_1',
      [battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneMotorGrill_1']),
  battat_sim_plane_connection(battat_toys:'UpperBodySlideInFrame', battat_sim:'PlaneUpperBody_1',
      [battat_sim:'PlaneUnderBody_1',battat_sim:'PlaneRearWing_1',battat_sim:'PlaneMotorGrill_1']).

battat_sim_plane_connection(ConnType, Primary, Parts) :-
  assemblage_create_connection(ConnType, Parts, Conn),
  % TODO: talk to simon why i have to wait here
  cram_assembly_apply_connection(Primary, Conn).
