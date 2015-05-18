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


:- module(knowrob_chemlab,
    [
        visualize_chemlab_scene/1,
        visualize_chemlab_object/3,
        visualize_chemlab_highlight/1,
        visualize_chemlab_highlights/1
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).
:- use_module(library('knowrob_mongo')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_chemlab, 'http://knowrob.org/kb/knowrob_chemlab.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    visualize_chemlab_scene(r),
    visualize_chemlab_object(+,+,r),
    visualize_chemlab_highlight(+),
    visualize_chemlab_highlights(+).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Visualization methods
%

visualize_chemlab_highlights([ObjFrame|Rest]) :-
  visualize_chemlab_highlight(ObjFrame),
  visualize_chemlab_highlights(Rest).
  
visualize_chemlab_highlights([]) :-
  true.

visualize_chemlab_highlight(ObjFrame) :-
  atom_concat('/', ObjFrame, Buf),
  atom_concat(Buf, '_frame', MarkerId),
  highlight_object_mesh(MarkerId).

visualize_chemlab_scene(T) :-
  % Query experiment information
  experiment(Exp, T),
  experiment_map(Exp, Map, T),
  % Query all occuring objects
  findall(Obj, (
    owl_has(Exp, knowrob:'occuringObject', ObjUrl),
    rdf_split_url(_, Obj, ObjUrl)
  ), Objs),
  clear_trajectories,
  % Show the PR2
  add_agent_visualization('PR2', pr2:'PR2Robot1', T, '', ''),
  % Show objects
  forall(
    member(Obj, Objs), ((
      designator_template(Map, Obj, Template),
      owl_has(Template, knowrob:'pathToCadModel', literal(type(_,MeshPath))),
      owl_has(Template, knowrob:'urdfName', literal(type(_,ObjFrame))),
      visualize_chemlab_object(ObjFrame, MeshPath, T)
    ) ; true)
  ).

visualize_chemlab_object(ObjFrame, MeshPath, T) :-
  remove_mesh_highlight(ObjFrame),
  
  mng_lookup_transform('/map', ObjFrame, T, Transform),
  % Extract quaternion and translation vector
  matrix_rotation(Transform, Quaternion),
  matrix_translation(Transform, Translation),
  % Publish mesh marker message
  add_mesh(ObjFrame, MeshPath, Translation, Quaternion).
