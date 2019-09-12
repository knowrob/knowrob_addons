/** <module> knowrob_chemlab

  Copyright (C) 2013 by Asil Kaan Bozcuoglu, Moritz Tenorth, Daniel Beßler

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

@author Asil Kaan Bozcuoglu, Moritz Tenorth, Daniel Beßler
@license BSD
*/

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_mongo).
%:- register_ros_package(knowrob_math).
:- register_ros_package(srdl).
:- register_ros_package(comp_temporal).

:- register_ros_package(knowrob_chemlab).
:- use_module(library('knowrob_chemlab')).

%Extended ontology
:- owl_parser:owl_parse('package://knowrob_chemlab/owl/chemlab.owl').
:- rdf_db:rdf_register_ns(chemlab, 'http://knowrob.org/kb/chemlab.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(chemstoff, 'http://knowrob.org/kb/chemlab-substances.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(chemobjects, 'http://knowrob.org/kb/chemlab-objects.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(chemactions, 'http://knowrob.org/kb/chemlab-actions.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(chemlab_map_2015,
        'http://knowrob.org/kb/chemlab-map_review-2015.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(chemlab_map_2016,
        'http://knowrob.org/kb/chemlab-map_review-2016.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(chemlab_map_2015,
        'http://knowrob.org/kb/chemlab-map_review-2015.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(acat, 'http://knowrob.org/kb/acat-adt.owl#', [keep(true)]).

