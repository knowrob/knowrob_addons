/** <module> comp_missingobj

  Description:
    Determine which objects are missing on the table by combining
    perceptions, knowledge processing and probabilistic reasoning.


  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_missingobj_offline,
    [
      comp_missingObjects/1
    ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% visualisation canvas
:- use_module(library('mod_vis')).



%%
% comp_missingObjects(+Table, -Missing)
%
% check which objects are on the table, use prob inference to determine which ones
% should be, compare both and create instances of the missing items with an existence
% probability as determined by the prob inference procedure
%
comp_missingObjects(Missing) :-

  % start the visualisation window
  visualisation_canvas(Canvas),

  % read all objects that are on the table (including queries to vision system)
  objects_on_table(Tables, AreOnTable),

  % call the probabilistic inference engine
  call_prob_inference(AreOnTable, ShouldBeOnTable),

  % infer which objects are missing
  compute_missing_objects(AreOnTable, ShouldBeOnTable, Missing, MissingTypes),

  display_tables(Tables, Canvas),

  % display which objects are detected and which are missing
  show_present_objects(AreOnTable, Canvas),
  show_missing_objects(Missing, MissingTypes, Canvas, 0).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% probabilistic inference
%
%


%%
% call_prob_inference(+AreOnTable, -ShouldBeOnTableWithProb)
%
% ask the probabilistic inference engine which objects should be on the table
%
call_prob_inference(AreOnTable, ShouldBeOnTableWithProb) :-
  lists_to_arrays(AreOnTable, AreOnTableArray),
  jpl_call('edu.tum.cs.probcog.prolog.PrologInterface', 'setObjectsOnTable', [AreOnTableArray], _),
  jpl_call('edu.tum.cs.probcog.prolog.PrologInterface', 'getMissingObjectsOnTable', [], ShouldBeOnTableArray),
  arrays_to_lists(ShouldBeOnTableArray, ShouldBeOnTableWithProb).


%%
% compute_missing_objects(+AreOnTable, +ShouldBeOnTableWithProb, -Missing, -MissingTypes)
%
% compare the list of objects that are already on the table with that
% of objects that are supposed to be there to find missing items,
% create instances of these objects with the resp. probability
%
compute_missing_objects(AreOnTable, ShouldBeOnTableWithProb, Missing, MissingTypes) :-

  % find those elements that have a prob > 0 to be on the table
  findall(Mp, (member(Mp, ShouldBeOnTableWithProb),
              nth0(1, Mp, ProbAtom),
              term_to_atom(Prob, ProbAtom),
              >(Prob, 0)), ShouldBeOnTable),

  % read types of objects on the table
  findall(Tl, (member(O, AreOnTable), rdf_has(O, rdf:type, T),rdf_split_url(_, Tl, T)), AreOnTableTypes),

  % find all objects that should be on the table, but are not member of the set of observed objects
  findall(Mp, (member(Mp, ShouldBeOnTable), nth0(0, Mp, M), not(member(M, AreOnTableTypes))), MissingTypes),


  % sort the list of missing objects
  %predsort(classProbCompare, MissingTypes, MissingTypesSorted),

  create_instances(MissingTypes, Missing, 0).



%%
% create_instances(+Types, -Instances, +Index).
%
% [used for the probcog interface]
%
% create a set of instances from the list of lists of types/probabilities
% and use just the next position in a predefined list of poses
%
create_instances([T|Types], [I|Instances], Index) :-
  create_instance(T, I, Index, Index1),
  create_instances(Types, Instances, Index1).
create_instances([], [], _).

create_instance([Type|Prob], Inst, Index, Index1) :-
  [P]=Prob,
  term_to_atom(ProbNum, P),


  (=<(0.4, ProbNum) -> (

      Index1 is Index+1,

      atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', Type, T),
      atom_concat(T, Index, Inst),

      rdf_assert(Inst, rdf:type, T),
      [P]=Prob,
      rdf_assert(Inst, knowrob:existenceProbability, P),
      pose_number(Index, Pose),
      rdf_assert(Inst, knowrob:orientation, Pose))
  ; (Index1 = Index)).


%%
% hypoDistCompare(-Delta, +P1, +P2)
%
% compare two object classes with their resp. probabilities
%
classProbCompare(Delta, [_, P1], [_, P2]) :-
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    compare(Delta, N2, N1).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% ros-perception-related stuff
%
%


%%
% objects_on_table(-Obj)
%
% read object information from ROS
%
objects_on_table(Tables, Objs) :-


  findall(Table, (rdf_has(Table, rdf:type, knowrob:'KitchenTable')), Tables),

  % HACK: onPhysical does not seem to work well due to the orientation of the table, simply
  % query for all objects that are higher than 0.5m above the ground
  findall(Obj, (rdf_has(Obj, knowrob:orientation, O), rdf_has(O, knowrob:m23, literal(type('http://www.w3.org/2001/XMLSchema#float', Z))),term_to_atom(N, Z), >=(N, 0.5), not(rdf_has(O, knowrob:existenceProbability, _))), Objs).

food_or_drink(Tables, Objs) :-

  findall(Table, (rdf_has(Table, rdf:type, knowrob:'KitchenTable')), Tables),

  % HACK: onPhysical does not seem to work well due to the orientation of the table, simply
  % query for all objects that are higher than 0.5m above the ground
  findall(Obj, (rdf_has(Obj, knowrob:orientation, O), not(rdf_has(O, knowrob:existenceProbability, _))), Objs).


%%
% create_tables(+LongList, -Tables, +Index)
%
% split the long list into short pieces and create the object instances
%

create_tables(LongList, [Table|Tables]) :-

  append([ID, Xt, Yt, _, _, _, _, _], Rest, LongList),

  % create instance if there is none of the same name yet
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable', ID, Inst),

  (
    (\+ rdf_has(Inst, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable')) ->

    Table=Inst,
    (rdf_assert(Table, rdf:type, 'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable'),

    % set the pose
    atomic_list_concat(['rotMat3D_1_0_0_',Xt,'_0_1_0_',Yt,'_0_0_1_',Zt,'_0_0_0_1'], Loc),
    rdf_assert(Table, knowrob:orientation, Loc),
    create_tables(Rest, Tables))

  ;(Table=false,create_tables(Rest, Tables))).

create_tables([], []).


%%
% create_object_instances_from_ros_list(+LongList, -Objs, +Index)
%
% split the long list into short pieces and create the object instances
%
create_object_instances_from_ros_list(LongList, [Obj|Objs], Index) :-

  append([_, _, _, _, Type, Xo, Yo, Zo], Rest, LongList),

  % create instance
  string_to_atom(Type, TypeAtom),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', TypeAtom, T),
  atom_concat(T, Index, Obj),
  rdf_assert(Obj, rdf:type, T),

  % set the pose
  atomic_list_concat(['rotMat3D_1_0_0_',Xo,'_0_1_0_',Yo,'_0_0_1_',Zo,'_0_0_0_1'], Loc),
  rdf_assert(Obj, knowrob:orientation, Loc),

  Index1 is Index+1,
  create_object_instances_from_ros_list(Rest, Objs,Index1).

create_object_instances_from_ros_list([], [], _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% visualization
%
%


%%
% show_missingObjects(+Objs)
%
% create a visualisation canvas and push all the objects given there
%
display_tables([Table|Tables], Canvas) :-

  print('adding table '),print(Table),

  % set the zoom and rotation so that we get some ego-perspective
  jpl_call(Canvas, 'setViewParameters', [156.0, 224.5, 192.75009, -20.350006, 2.2399998], _),
  jpl_call(Canvas, 'addObjectWithChildren', [Table], _),
  display_tables(Tables, Canvas).
display_tables([], _).


show_present_objects(Objs, Canvas) :-
  findall(Obj, (member(Obj, Objs),
                jpl_call(Canvas, 'addObject', [Obj], _),
                jpl_call(Canvas, 'highlight', [Obj, @(true), 0, 256, 0], _)), _).


show_missing_objects([O|Objs], [T|Types], Canvas, Counter) :-

  nth0(1, T, Prob),
  term_to_atom(ProbNum, Prob),

  (=<(0.4, ProbNum) ->
    (jpl_call(Canvas, 'addObject', [O], _),
      jpl_call(Canvas, 'highlight', [O, @(true), 256, 0, 0, Prob], _),
      Counter1 = Counter + 1,
      show_missing_objects(Objs, Types, Canvas, Counter1));

    (Counter1 = Counter + 1,
      show_missing_objects(Objs, Types, Canvas, Counter1))
  ).
show_missing_objects([], [], _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% utils
%
%

%%
% pose_number(+Id, -Pose)
%
% just read one of a set of predefined poses where the (inferred) objects are to be put on the table
%
pose_number(Id, Pose) :-
  Poses = ['http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_0',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_1',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_2',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_3',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_4',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_5',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_6',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_7',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_8',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_9',
         'http://ias.cs.tum.edu/kb/comp_missingobj.owl#rotmat3d_ontable_10'],
  nth0(Id, Poses, Pose).


%%
% remove_false_from_list(-In, -Out)
%
% filter all false from the list
%
remove_false_from_list([false|In], Out) :-
  remove_false_from_list(In, Out).

remove_false_from_list([First|In], [First|Out]) :-
  not(First=false),
  remove_false_from_list(In, Out).
remove_false_from_list([], []).


