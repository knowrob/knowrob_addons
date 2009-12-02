%%
%% Copyright (C) 2009 by Moritz Tenorth, Daniel Nyga
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
%%
%% Copyright (C) 2009 by Moritz
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% 
%   Module comp_ehow
%   Authors: Moritz Tenorth
%
%   Description:
%     Interface to the system for importing natural-language task descriptions
%     as obtained from web sites like ehow.com
% 
%     Using the computables defined here, it is possible to query for a plan
%     by giving a natural-language command like 'set the table' and retrieve
%     a description of all required steps in form of an OWL TBOX specification.
% 
%     The temporary OWL file returned from the Java-based import system is
%     directly parsed and the main concept describing the plan as a whole is
%     returned.
% 
%   Requirements:
%     The import system requires Cyc to be installed and running on the computer
%     it is executed on. Accessing Cyc via the network is possible, check with
%     Moritz if you need that. researchCyc is preferred, the free OpenCyc may
%     work more or less acceptably.
%     You further need to set the WNHOME environment variable to your WordNet 2.0 (!!)
%     installation.
% 
%   Usage:
%     rdf_triple(knowrob:forCommand, A, 'set a table').
%     A = 'http://www.owl-ontologies.com/Ontology1252257693.owl#SetATable'
% 

:- module(comp_ehow,
    [
      comp_forCommand/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).



    % compute plan from ehow.com import
    comp_forCommand(Plan, Command) :-
      var(Plan), nonvar(Command),
      jpl_call('instruction.exporter.owl.OWLExporter', 'convertHowtoToOWLOntology', [Command], F),
      owl_parse(F, false, false, true),
      rdf_has(Plan, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', Command))).


