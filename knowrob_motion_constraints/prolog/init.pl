/*
  Copyright (C) 2013-14 Moritz Tenorth
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

  @author Moritz Tenorth
  @license BSD
*/

:- register_ros_package(srdl).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_mesh_reasoning).

:- register_ros_package(knowrob_motion_constraints).
:- use_module(library(knowrob_motion_constraints)).

% namespace for general motion constraint ontology
:- rdf_db:rdf_register_ns(constr, 'http://knowrob.org/kb/motion-constraints.owl#', [keep(true)]).

% pancake making task definition
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/pancake-making-constr.owl').
:- rdf_db:rdf_register_ns(pancake_constr, 'http://knowrob.org/kb/pancake-making-constr.owl#', [keep(true)]).

% pouring task definition
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/pouring.owl').
:- rdf_db:rdf_register_ns(motion, 'http://knowrob.org/kb/motion-def.owl#', [keep(true)]).

% object models
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/spatula-features.owl').
:- rdf_db:rdf_register_ns(spatula, 'http://knowrob.org/kb/spatula-features.owl#', [keep(true)]).

:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/mondamin-pancake-mix.owl').
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/pancake-maker.owl').

