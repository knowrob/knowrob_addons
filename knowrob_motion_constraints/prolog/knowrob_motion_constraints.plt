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
:- owl_parser:owl_parse('../owl/mondamin-pancake-mix.owl', false, false, true).
:- owl_parser:owl_parse('../owl/pancake-maker.owl', false, false, true).
:- owl_parser:owl_parse('../owl/coke-bottle.owl', false, false, true).


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
test(motion_constraint) :-
    findall(C, motion_constraint('http://ias.cs.tum.edu/kb/motion-def.owl#TiltBottle', C), Cs),
    Cs = ['http://ias.cs.tum.edu/kb/motion-constraints.owl#PerpendicularityConstraint_E8ysUdzg',
          'http://ias.cs.tum.edu/kb/motion-constraints.owl#HeightConstraint_OZjsDn3E',
          'http://ias.cs.tum.edu/kb/motion-constraints.owl#RightOfConstraint_fePCJFEB',
          'http://ias.cs.tum.edu/kb/motion-constraints.owl#InFrontOfConstraint_Sv4UGtRm',
          'http://ias.cs.tum.edu/kb/motion-constraints.owl#PointingAtConstraint_f5BvdFyF'].



% Test if constraint properties are correctly determined
test(constraint_properties) :-

     constraint_properties(
       'http://ias.cs.tum.edu/kb/motion-def.owl#PancakeMixInRightHand',
       'http://ias.cs.tum.edu/kb/knowrob.owl#PancakeMaker',
       'http://ias.cs.tum.edu/kb/motion-constraints.owl#PerpendicularityConstraint_qpdE8yUz',
        Type, ToolFeature, WorldFeature, ReferenceFrame, Lower, Upper),
        
        Type = 'http://ias.cs.tum.edu/kb/motion-constraints.owl#PerpendicularityConstraint',
        ToolFeature = 'http://ias.cs.tum.edu/kb/knowrob.owl#Cone_7c7Sqyie',
        WorldFeature = 'http://ias.cs.tum.edu/kb/knowrob.owl#FlatPhysicalSurface_AEFloDeh',
        ReferenceFrame = '/torso_lift_link',
        Lower = 0.95,
        Upper = 1.2.


% test if feature_properties can be read
test(feature_properties_line) :-
  once(feature_properties('http://ias.cs.tum.edu/kb/knowrob.owl#Cone_7c7Sqyie',
                      Type, Label, TfFrame, Position, Direction)),
  Type = 'http://ias.cs.tum.edu/kb/knowrob.owl#LineFeature',
  Label = '',
  TfFrame = '/pancake_bottle',
  Position = [-9.733001888889703e-7,1.062735805135162e-6,0.4575471878051758],
  Direction = [-9.538994594215922e-11,4.656713209483243e-11,-0.00899999588727951].

                     
test(feature_properties_plane) :-
  once(feature_properties('http://ias.cs.tum.edu/kb/knowrob.owl#FlatPhysicalSurface_AEFloDeh',
                      Type, Label, TfFrame, Position, Direction)),
  Type = 'http://ias.cs.tum.edu/kb/knowrob.owl#PlaneFeature',
  Label = '',
  TfFrame = '/pancake_maker',
  Position = [0.04166119545698166,-6.67572021484375e-6,-0.06330671906471252],
  Direction = [-0.00821682345122099,-0.0047430298291146755,0.9999549984931946].

                     
:- end_tests(knowrob_motion_constraints).

