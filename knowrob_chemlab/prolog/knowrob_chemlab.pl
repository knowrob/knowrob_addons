/** <module> knowrob_chemlab

  Copyright (C) 2013-16 by Asil Kaan Bozcuoglu, Moritz Tenorth, Daniel Beßler

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


:- module(knowrob_chemlab,
    [
        comp_objectActedOn/2,
        task_screwing_objects/3,
        comp_insideOf/2,
        comp_insideOf_at_time/3,
        import_task_as_adt/3,
        adt_object_type/2,
        adt_publish/1,
        update_pipetting_scene/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('jpl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_transforms')).
:- use_module(library('knowrob_math')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_chemlab, 'http://knowrob.org/kb/knowrob_chemlab.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(acat, 'http://knowrob.org/kb/acat-adt.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    task_screwing_objects(r,?,?),
    adt_publish(r),
    adt_object_type(r,?).
    
is_screwable_on(CapIndividual, ContIndividual) :-
  owl_has(CapIndividual, knowrob_chemlab:'screwable', ContClass),
  owl_has(ContIndividual, rdf:'type', ContClass).

task_screwing_objects(Task, Cap, Container) :-
  task(Task),
  % There is a perceive subtask for container and cap
  findall(Type, (
    subtask(Task, Sub),
    rdfs_individual_of(Sub, knowrob:'UIMAPerception'),
    rdf_has(Sub, knowrob:'perceptionRequest', Desig),
    mng_designator(Desig, DesigJava),
    mng_designator_props(Desig, DesigJava, ['TYPE'], Type)
  ), Types),
  length(Types, 2),
  nth0(0, Types, Obj0),
  nth0(1, Types, Obj1),
  owl_has(Obj0_, knowrob:'name', literal(type(_,Obj0))),
  owl_has(Obj1_, knowrob:'name', literal(type(_,Obj1))),
  % Find container and cap
  (   is_screwable_on(Obj0_, Obj1_)
  ->  ( Cap = Obj0_, Container = Obj1_ )
  ;   ( Cap = Obj1_, Container = Obj0_ )
  ).


comp_objectActedOn(Act, Obj) :-
  rdf_has(Prev, knowrob:'nextAction', Act),
  subtask(Prev, Sub),
  rdfs_individual_of(Sub, knowrob:'UIMAPerception'),
  rdf_has(Sub, knowrob:'perceptionRequest', Desig),
  mng_designator(Desig, DesigJava),
  mng_designator_props(Desig, DesigJava, ['TYPE'], Type),
  owl_has(Obj, knowrob:'name', literal(type(_,Type))).

comp_objectActedOn(Act, Obj) :-
  rdf_has(Act, knowrob:'objectActedOn', Desig),
  mng_designator(Desig, DesigJava),
  mng_designator_props(Desig, DesigJava, ['TYPE'], Type),
  owl_has(Obj, knowrob:'name', literal(type(_,Type))).


knowrob_temporal:holds(Inner, 'http://knowrob.org/kb/chemlab-map_review-2015.owl#insideOf', Outer, Instant) :-
  comp_insideOf_at_time(Inner, Outer, Instant).

comp_insideOf(InnerObj, OuterObj) :-
  current_time(Instant),
  comp_insideOf_at_time(InnerObj, OuterObj, Instant).

comp_insideOf_at_time(InnerObj, OuterObj, [Instant,Instant]) :-
  comp_insideOf_at_time(InnerObj, OuterObj, Instant), !.
comp_insideOf_at_time(InnerObj, OuterObj, Instant) :-
  ground(Instant),
  map_frame_name(MapFrame),
  
  rdfs_individual_of(InnerObj, knowrob:'EnduringThing-Localized'),
  object_pose_at_time(InnerObj, Instant, [MapFrame, _, [X_Frame, Y_Frame, Z_Frame], _]),
  
  rdfs_individual_of(OuterObj, knowrob:'Container'),
  object_pose_at_time(OuterObj, Instant, [MapFrame, _, [X_Out, Y_Out, Z_Out], _]),

  rdf_has(OuterObj, srdl2comp:'box_size', literal(type(_,BoxSize))),
  rdf_has(OuterObj, srdl2comp:'aabb_offset', literal(type(_,Offsets))),
  rdf_vector_prolog(BoxSize, [X_Box,Y_Box,Z_Box]),
  rdf_vector_prolog(Offsets, [X_Off,Y_Off,Z_Off]),
  X_Positive is X_Out + X_Off + X_Box,
  X_Negative is X_Out + X_Off - X_Box,
  Y_Positive is Y_Out + Y_Off + Y_Box,
  Y_Negative is Y_Out + Y_Off - Y_Box,
  Z_Positive is Z_Out + Z_Off + Z_Box,
  Z_Negative is Z_Out + Z_Off - Z_Box,
  (X_Negative > X_Frame, X_Positive < X_Frame;
  X_Negative < X_Frame, X_Positive > X_Frame),
  (Y_Negative > Y_Frame, Y_Positive < Y_Frame;
  Y_Negative < Y_Frame, Y_Positive > Y_Frame),
  (Z_Negative > Z_Frame, Z_Positive < Z_Frame;
  Z_Negative < Z_Frame, Z_Positive > Z_Frame).    


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Final review
%

adt_object_type(Object, Type) :-
  rdf_has(Object, rdf:'type', ClsUri),
  rdfs_subclass_of(ClsUri, knowrob:'EnduringThing-Localized'),
  rdf_split_url(_,Type,ClsUri), !.

adt_publish(ADT) :-
  jpl_new('org.knowrob.chemlab.ADTDesignator', [], ADTDesignator),
  jpl_call(ADTDesignator, 'readFromIndividual', [ADT], _),
  designator_publish(ADT, ADTDesignator).

import_task_as_adt(ExperimentName, TaskType, ADTPath) :-
  rospack_package_path('knowrob_chemlab', Path),
  atom_concat(Path, '/dictionaries', DictionaryPath),
  atom_concat(Path, '/owl', OWLPath),
  jpl_new('org.knowrob.chemlab.EpisodicMemoryToADT', [OWLPath, DictionaryPath, '/home/ros/user_data', 'http://knowrob.org/kb/acat.owl', 'http://knowrob.org/kb/acat_example.owl', ExperimentName], EA),
  jpl_call(EA, 'generateADT', [TaskType], _R),
  atom_concat('/home/ros/user_data', '/adt0.owl', ADTPath).

update_pipetting_scene(T) :-
 findall(O, (rdf_has(O, knowrob:'describedInMap', 'http://knowrob.org/kb/chemlab-map_review-2016.owl#SemanticEnvironmentMap_FS745hf347hf')),Os),
 marker_update(agent(pr2:'PR2Robot1'), T),
 % Show objects
 forall(
    member(Obj, Os), ((
      owl_has(Obj, knowrob:'pathToCadModel', literal(type(_,MeshPath))),
      owl_has(Obj, knowrob:'urdfName', literal(type(_,ObjFrame))),
      visualize_chemlab_object(ObjFrame, MeshPath, T)
    ) ; true)
  ), !. 








