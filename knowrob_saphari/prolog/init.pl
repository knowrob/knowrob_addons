%%
%% Copyright (C) 2013 by Moritz Tenorth
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

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_mongo).
:- register_ros_package(comp_temporal).

:- register_ros_package(knowrob_saphari).
:- use_module(library('knowrob_saphari')).

:- mng_db('saphari').

%Extended ontology
:- owl_parser:owl_parse('package://knowrob_saphari/owl/saphari.owl').
:- rdf_db:rdf_register_ns(saphari, 'http://knowrob.org/kb/saphari.owl#', [keep(true)]).

:- owl_parser:owl_parse('package://knowrob_srdl/owl/openni_human1.owl').
:- rdf_db:rdf_register_ns(openni_human, 'http://knowrob.org/kb/openni_human1.owl#', [keep(true)]).

:- owl_parser:owl_parse('package://knowrob_srdl/owl/Boxy.owl').
:- rdf_db:rdf_register_ns(boxy, 'http://knowrob.org/kb/Boxy.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(saphari_map, 'http://knowrob.org/kb/saphari_map.owl#', [keep(true)]).

