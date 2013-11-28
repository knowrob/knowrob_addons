%%
%% Copyright (C) 2013 by Moritz Tenorth
%%
%% This file contains tests for the constraint-based motion specification 
%% tools in KnowRob.
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


:- begin_tests(knowrob_motion_constraints).

:- use_module(library('knowrob_motion_constraints')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_actions')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_mesh_reasoning')).


:- owl_parser:owl_parse('../owl/spatula-features.owl', false, false, true).
:- owl_parser:owl_parse('../owl/pouring.owl', false, false, true).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(constr, 'http://ias.cs.tum.edu/kb/motion-constraints.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(motion, 'http://ias.cs.tum.edu/kb/motion-def.owl#', [keep(true)]).


% Test if all motion phases of the flipping motion are correctly retrieved
test(plan_subevents) :-
    plan_subevents('http://ias.cs.tum.edu/kb/motion-def.owl#PouringSomething', Subs),
    Subs = ['http://ias.cs.tum.edu/kb/motion-def.owl#MoveAbovePan',
            'http://ias.cs.tum.edu/kb/motion-def.owl#TiltBottle',
            'http://ias.cs.tum.edu/kb/motion-def.owl#TiltBack'].

% Test if motion constraints for a phase are correctly retrieved
test(motion_constraint_plain) :-
    findall(C, motion_constraint('http://ias.cs.tum.edu/kb/motion-def.owl#TiltBottle', C), Cs),
    Cs = ['http://ias.cs.tum.edu/kb/motion-def.owl#DistanceConstraint_aePJVzGM',
          'http://ias.cs.tum.edu/kb/motion-def.owl#PointingAtConstraint_fo5VpFyF',
          'http://ias.cs.tum.edu/kb/motion-def.owl#HeightConstraint_ouGDWJ2K',
          'http://ias.cs.tum.edu/kb/motion-def.owl#PerpendicularityConstraint_SoYmvFF5'].

   
% Test if motion constraints for a phase and a tool are correctly retrieved
%
% DEACTIVATED TILL FEATURE REPRESENTATION HAS STABILIZED
% 
% test(motion_constraint_tool) :-
%     findall(C, motion_constraint('http://ias.cs.tum.edu/kb/motion-def.owl#TiltBottle', 
%                                  'http://ias.cs.tum.edu/kb/motion-def.owl#bottle-top', C), Cs),
%     member('http://ias.cs.tum.edu/kb/pancake-making-constr.owl#DistanceLeftSpatulaAxisPancake_aneXbLGX', Cs),!.


% Test if constraint properties are correctly determined
test(constraint_properties) :-
     constraint_properties(
        'http://ias.cs.tum.edu/kb/motion-def.owl#HeightConstraint_ouGDWJ2K', 
        'http://ias.cs.tum.edu/kb/motion-constraints.owl#DistanceConstraint',
        'http://ias.cs.tum.edu/kb/spatula-features.owl#Handle_z3rLFrlP',
        'http://ias.cs.tum.edu/kb/spatula-features.owl#FlatPhysicalSurface_DQoI3DXH',
        A,
        0.15,
        0.17),!.


test(feature_properties_line) :-
  feature_properties('http://ias.cs.tum.edu/kb/spatula-features.owl#Handle_z3rLFrlP',
                     'http://ias.cs.tum.edu/kb/knowrob.owl#LineFeature',
                     'left spatula: main axis',
                     'map',
                     [0.0,0.0,0.0],
                     [0.0,0.0,0.125]),!.

test(feature_properties_plane) :-
  feature_properties('http://ias.cs.tum.edu/kb/spatula-features.owl#FlatPhysicalSurface_DQoI3DXH',
                     'http://ias.cs.tum.edu/kb/knowrob.owl#PlaneFeature',
                     'pancake plane',
                     'map',
                     [0.0,0.0,0.0],
                     [0.0,0.0,0.1]),!.

                     
:- end_tests(knowrob_motion_constraints).

