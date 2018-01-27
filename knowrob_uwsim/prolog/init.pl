/*

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
:- register_ros_package(knowrob_meshes).

:- register_ros_package(knowrob_uwsim).
:- use_module(library('knowrob_uwsim')).

%Extended ontology
:- rdf_register_ns(uwsim_map, 'http://knowrob.org/kb/UWSim.owl#', [keep(true)]).
:- rdf_register_ns(fishone, 'http://knowrob.org/kb/Fish1.owl#', [keep(true)]).
:- rdf_register_ns(fishtwo, 'http://knowrob.org/kb/Fish2.owl#', [keep(true)]).
:- rdf_register_ns(fishthree, 'http://knowrob.org/kb/Fish3.owl#', [keep(true)]).
:- rdf_register_ns(fishfour, 'http://knowrob.org/kb/Fish4.owl#', [keep(true)]).
:- rdf_register_ns(fishfive, 'http://knowrob.org/kb/Fish5.owl#', [keep(true)]).
:- rdf_register_ns(fishsix, 'http://knowrob.org/kb/Fish6.owl#', [keep(true)]).
:- rdf_register_ns(fishseven, 'http://knowrob.org/kb/Fish7.owl#', [keep(true)]).
:- rdf_register_ns(fisheight, 'http://knowrob.org/kb/Fish8.owl#', [keep(true)]).
:- rdf_register_ns(fishnine, 'http://knowrob.org/kb/Fish9.owl#', [keep(true)]).
:- rdf_register_ns(sharkone, 'http://knowrob.org/kb/Shark1.owl#', [keep(true)]).
:- rdf_register_ns(uwvehicle, 'http://knowrob.org/kb/UWVehicle.owl#', [keep(true)]).
:- rdf_register_ns(fishswarm, 'http://knowrob.org/kb/Fishswarm.owl#', [keep(true)]).
:- rdf_register_ns(tortoise, 'http://knowrob.org/kb/Tortoise.owl#', [keep(true)]).
:- rdf_register_ns(uwvehicle2, 'http://knowrob.org/kb/UWVehicle2.owl#', [keep(true)]).
