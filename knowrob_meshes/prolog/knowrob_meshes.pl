/** <module> knowrob_meshes

  Copyright (C) 2015 Daniel Beßler
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

  @author Daniel Beßler
  @license BSD
*/

:- module(knowrob_meshes,
    [
      add_designator_contour_mesh/4,
      get_designator_contour_size/3,
      add_designator_checkerboard_mesh/2,
      contour_mesh_extends/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/marker_vis')).
:- use_module(library('jpl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

:- rdf_meta add_designator_contour_mesh(r,r,+,+),
            get_designator_contour_size(r,r,+),
            add_designator_checkerboard_mesh(r,r).

add_designator_contour_mesh(MarkerId, DesignatorId, Color, ContourPath) :-
    atom(DesignatorId),
    mng_designator(DesignatorId, DesigJava),
    add_designator_contour_mesh(MarkerId, DesigJava, Color, ContourPath).

add_designator_contour_mesh(MarkerId, DesigJava, Color, ContourPath) :-
    mng_designator_props('CONTOUR', DesigJava, ContourPath, ContourDesig),
    mng_designator_props('CONTOUR', DesigJava, ['TIMESTAMP'], Timestamp),
    lists_to_arrays(Color, ColorArr),
    jpl_call('org.knowrob.vis.meshes.MeshVisualization', 'addDesignatorContourMesh', [MarkerId, ContourDesig, Timestamp, ColorArr], _).


contour_mesh_extends(DesignatorId, ContourPath, Extends) :-
    atom(DesignatorId),
    mng_designator(DesignatorId, DesigJava),
    contour_mesh_extends(DesigJava, ContourPath, Extends).
 
contour_mesh_extends(DesigJava, ContourPath, [Min,Max]) :-
    mng_designator_props('CONTOUR', DesigJava, ContourPath, ContourDesig),
    mng_designator_props('CONTOUR', DesigJava, ['TIMESTAMP'], Timestamp),
    jpl_call('org.knowrob.vis.meshes.MeshVisualization',
        'getContourExtends', [ContourDesig, Timestamp], ExtendsJava),
    jpl_get(ExtendsJava, min, MinArray),
    jpl_get(ExtendsJava, max, MaxArray),
    jpl_array_to_list(MinArray,Min),
    jpl_array_to_list(MaxArray,Max).

    
get_designator_contour_size(DesignatorId, ContourPath, Size) :-
    mng_designator(DesignatorId, DesigJava),
    mng_designator_props(DesignatorId, DesigJava, ContourPath, ContourDesig),
    jpl_call('org.knowrob.vis.meshes.MeshVisualization', 'getDesignatorContourSize', [ContourDesig], Size).

add_designator_checkerboard_mesh(MarkerId, DesignatorId) :-
    mng_designator(DesignatorId, DesigJava),
    jpl_call('org.knowrob.vis.meshes.MeshVisualization', 'addDesignatorCheckerboardMesh', [MarkerId, DesigJava], _).

