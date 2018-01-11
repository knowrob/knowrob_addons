/** <module> comp_barcoo

  This module provides routines to interface the ODUfinder cognitive perception
  system, i.e. to read data and interpret the results.

  Copyright (C) 2012 by Nacer Khalil

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

@author Nacer Khalil
@license BSD
*/

:- module(comp_barcoo,
    [
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_barcoo, 'http://www.barcoo.com/barcoo.owl#', [keep(true)]).


%% odufinder_listener(-Listener)
odufinder_listener(Listener) :-
    jpl_new('edu.tum.cs.ias.knowrob.comp_barcoo.BarcooROSclient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startCopObjDetectionsListener', ['/knowrobTopic'], _).

%% cop_create_model_instance(+ModelType, +ObjectType) is det.
%
% Create instance of a CopPerception model that provides recognition
% services for the ObjectType
%
cop_create_model_instance(ModelType, ObjectType) :-

  atom_concat('http://knowrob.org/kb/comp_cop.owl#', ModelType, ModelT),
  rdf_instance_from_class(ModelT, ModelInst),

  rdf_assert(ModelInst, knowrob:providesModelFor, ObjectType).

%% cop_create_perception_instance(+ModelTypes, -Perception) is det.
%
% Create perception instance having all the types in ModelTypes
%
cop_create_perception_instance(ModelTypes, Perception) :-

  rdf_instance_from_class('http://knowrob.org/kb/comp_cop.owl#CopPerception', Perception),

  findall(MC, (member(MT, ModelTypes),
               atom_concat('http://knowrob.org/kb/knowrob.owl#', MT, MC),
               rdf_assert(Perception, knowrob:perceivedUsingModel, MC)), _),

  % create detection time point
  owl_instance_from_class(knowrob:'TimePoint', TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).



%% cop_create_object_instance(+ObjTypes, +CopID, -Obj) is det.
%
% Create object instance having all the types in ObjTypes
%
cop_create_object_instance(ObjTypes, CopID, Obj) :-

  member(ObjType, ObjTypes),
  string_to_atom(ObjType, TypeAtom),
%   atom_concat('http://knowrob.org/kb/knowrob.owl#', LocalTypeAtom, TypeAtom),
  atom_concat(TypeAtom, CopID, Obj),

  (rdf_has(Obj, rdf:type, TypeAtom),!;
  rdf_assert(Obj, rdf:type, TypeAtom)),

  string_to_atom(CopID, CopIDAtom),
  term_to_atom(CopIDTerm, CopIDAtom),
  rdf_assert(Obj, knowrob:copID, literal(type(xsd:string, CopIDTerm))).

