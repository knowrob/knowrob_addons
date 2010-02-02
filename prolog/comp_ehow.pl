/** <module> comp_ehow

  Description:
    Interface to the system for importing natural-language task descriptions
    as obtained from web sites like ehow.com

    Using the computables defined here, it is possible to query for a plan
    by giving a natural-language command like 'set the table' and retrieve
    a description of all required steps in form of an OWL TBOX specification.

    The temporary OWL file returned from the Java-based import system is
    directly parsed and the main concept describing the plan as a whole is
    returned.

  Requirements:
    The import system requires Cyc to be installed and running on the computer
    it is executed on. Accessing Cyc via the network is possible, check with
    Moritz if you need that. researchCyc is preferred, the free OpenCyc may
    work more or less acceptably.
    You further need to set the WNHOME environment variable to your WordNet 2.0 (!!)
    installation.

  Usage:
    rdf_triple(knowrob:forCommand, A, 'set a table').
    A = 'http://www.owl-ontologies.com/Ontology1252257693.owl#SetATable'


  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_ehow,
    [
      comp_forCommand/2,
      matching_actions/2,
      plan_subevents/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).



%% comp_forCommand(-Plan, +Command) is nondet.
%
% Launch the ehow.com import procedure to import a plan described by the natural-language
% command Command and return an OWL TBOX description of the plan.
%
% @param Pre Identifier of the earlier time point
% @param After Identifier of the later time point
% 
comp_forCommand(Plan, Command) :-
  var(Plan), nonvar(Command),
  jpl_call('instruction.exporter.owl.OWLExporter', 'convertHowtoToOWLOntology', [Command], F),
  owl_parse(F, false, false, true),
  rdf_has(Plan, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', Command))).



%% plan_subevents(+Plan, ?SubEvents) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
plan_subevents(Plan, SubEvents) :-
  findall(SubEvent, ( owl_has(Plan, rdfs:subClassOf, D),
                    owl_has(D, owl:intersectionOf, I),
                    rdfs_member(R, I),
                    rdf_has(R, owl:someValuesFrom, SubEvent)), SubEvents).


%% matching_actions(?Plan, ?Act) is semidet.
%
% Search for action instances that fit the classes described in the imported plan
%
% @param Plan Plan identifier
% @param Act Matching actions
matching_actions(Plan, Act) :-
  plan_subevents(Plan, SubEvents),
  rdf_has(Act, rdf:type, knowrob:'PuttingSomethingSomewhere'),
  member(ActCl, SubEvents),
  owl_individual_of(Act, ActCl).
