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
    it is executed on. see package opencyc

  Usage:
    rdf_triple(knowrob:forCommand, A, 'set a table').
    A = 'http://www.owl-ontologies.com/Ontology1252257693.owl#SetATable'


  Copyright (C) 2010 by Moritz Tenorth
  
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Moritz Tenorth
@license BSD
*/

:- module(comp_ehow,
    [
      comp_forCommand/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/actions')).

:- owl_parser:owl_parse('package://comp_ehow/owl/comp_ehow.owl').

:-  rdf_meta
      comp_forCommand(r, r),
      matching_actions(r, r),
      plan_subevents(r, -),
      plan_subevents_recursive(r, -),
      plan_objects(r, r),
      action_properties(r, r, r),
      action_objectActedOn(r,r),
      action_toLocation(r,r),
      action_fromLocation(r,r).


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
  jpl_new('instruction.exporter.owl.OWLExporter', [], OWLExp),
  jpl_call(OWLExp, 'convertHowtoToOWLFile', [Command], F),
  owl_parse(F, false, false, true),
  rdf_has(Plan, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', Command))).

