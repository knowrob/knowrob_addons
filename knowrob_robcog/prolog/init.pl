/*
  Copyright (C) 16 Andrei Haidu
  All rights reserved.

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

  @author Andrei Haidu
  @license BSD
*/

:- register_ros_package(knowrob_common).
:- register_ros_package(comp_temporal).
:- register_ros_package(knowrob_vis).
% :- register_ros_package(knowrob_meshes).
%:- register_ros_package(knowrob_objects).

:- register_ros_package(knowrob_robcog).
:- use_module(library('robcog_mongo_interface')).
:- use_module(library('robcog_games')).
:- use_module(library('robcog_event_logic')).
:- use_module(library('robcog_mongo_queries')).
:- use_module(library('robcog_test_queries')).

% returns the namspace when outputting values
:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_u, 'http://knowrob.org/kb/knowrob_u.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(log, 'http://knowrob.org/kb/unreal_log.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(u-map, 'http://knowrob.org/kb/u_map.owl#', [keep(true)]).

%Extended ontology example
%% :- owl_parser:owl_parse('package://knowrob_saphari/owl/saphari.owl').
%% :- rdf_db:rdf_register_ns(saphari, 'http://knowrob.org/kb/saphari.owl#', [keep(true)]).

%% :- owl_parser:owl_parse('package://knowrob_srdl/owl/openni_human1.owl').
%% :- rdf_db:rdf_register_ns(openni_human, 'http://knowrob.org/kb/openni_human1.owl#', [keep(true)]).

%% :- owl_parser:owl_parse('package://knowrob_srdl/owl/Boxy.owl').
%% :- rdf_db:rdf_register_ns(boxy, 'http://knowrob.org/kb/Boxy.owl#', [keep(true)]).

%% :- rdf_db:rdf_register_ns(saphari_map, 'http://knowrob.org/kb/saphari_map.owl#', [keep(true)]).