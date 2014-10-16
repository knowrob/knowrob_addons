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

:- register_ros_package(knowrob_sim).
:- use_module(library('knowrob_sim')).

%Extended ontology
:- owl_parser:owl_parse('package://knowrob_sim/owl/knowrob_sim.owl').
:- rdf_db:rdf_register_ns(sim, 'http://ias.cs.tum.edu/kb/knowrob_sim.owl#', [keep(true)]).

:- owl_parser:owl_parse('package://knowrob_sim/owl/simulation_map.owl').
:- rdf_db:rdf_register_ns(sim_map, 'http://ias.cs.tum.edu/kb/simulation_map.owl#', [keep(true)]).