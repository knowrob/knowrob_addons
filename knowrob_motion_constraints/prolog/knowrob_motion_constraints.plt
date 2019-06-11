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
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob/actions')).
:- use_module(library('knowrob/perception')).
:- use_module(library('knowrob_mesh_reasoning')).
:- use_module(library('jpl')).


:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/spatula-features.owl').
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/pouring.owl').
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/mondamin-pancake-mix.owl').
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/pancake-maker.owl').
:- owl_parser:owl_parse('package://knowrob_motion_constraints/owl/coke-bottle.owl').


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(constr, 'http://knowrob.org/kb/motion-constraints.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(motion, 'http://knowrob.org/kb/motion-def.owl#', [keep(true)]).


% Test if all motion phases of the flipping motion are correctly retrieved
test(plan_subevents) :-
    plan_subevents('http://knowrob.org/kb/motion-def.owl#PouringSomething', Subs),
    Subs = ['http://knowrob.org/kb/motion-def.owl#MoveAbovePan',
            'http://knowrob.org/kb/motion-def.owl#TiltBottle',
            'http://knowrob.org/kb/motion-def.owl#TiltBack'].


% Test if motion constraints for a phase are correctly retrieved
test(motion_constraint) :-
    findall(C, motion_constraint('http://knowrob.org/kb/motion-def.owl#TiltBottle', C), Cs),
    Cs = ['http://knowrob.org/kb/motion-constraints.owl#PerpendicularityConstraint_E8ysUdzg',
          'http://knowrob.org/kb/motion-constraints.owl#HeightConstraint_OZjsDn3E',
          'http://knowrob.org/kb/motion-constraints.owl#RightOfConstraint_fePCJFEB',
          'http://knowrob.org/kb/motion-constraints.owl#InFrontOfConstraint_Sv4UGtRm'].



% Test if constraint properties are correctly determined
test(constraint_properties) :-

     constraint_properties(
       'http://knowrob.org/kb/motion-def.owl#PancakeMixInRightHand',
       'http://knowrob.org/kb/knowrob.owl#PancakeMaker',
       'http://knowrob.org/kb/motion-constraints.owl#PerpendicularityConstraint_qpdE8yUz',
        Type, ToolFeature, WorldFeature, ReferenceFrame, Lower, Upper),
        
        Type = 'http://knowrob.org/kb/motion-constraints.owl#PerpendicularityConstraint',
        ToolFeature = 'http://knowrob.org/kb/knowrob.owl#Cone_L1Xfg6eB',
        WorldFeature = 'http://knowrob.org/kb/knowrob.owl#FlatPhysicalSurface_AEFloDeh',
        ReferenceFrame = '/torso_lift_link',
        Lower = 0.95,
        Upper = 1.2.


% test if feature_properties can be read
test(feature_properties_line) :-
  once(feature_properties('http://knowrob.org/kb/knowrob.owl#Cone_L1Xfg6eB',
                      Type, Label, TfFrame, Position, Direction)),
  Type = 'http://knowrob.org/kb/knowrob.owl#LineFeature',
  Label = '',
  TfFrame = '/pancake_bottle',
  Position = [-2.0630949393307674e-7,2.253400595009225e-7,0.09699999541044235],
  Direction = [6.524104173566414e-11,-8.663848111156724e-11,0.00899999588727951].

                     
test(feature_properties_plane) :-
  once(feature_properties('http://knowrob.org/kb/knowrob.owl#FlatPhysicalSurface_AEFloDeh',
                      Type, Label, TfFrame, Position, Direction)),
  Type = 'http://knowrob.org/kb/knowrob.owl#PlaneFeature',
  Label = '',
  TfFrame = '/pancake_maker',
  Position = [0.0,-6.67572021484375e-6,0.0],
  Direction = [-0.00821682345122099,-0.0047430298291146755,0.9999549984931946].

                     
:- end_tests(knowrob_motion_constraints).

