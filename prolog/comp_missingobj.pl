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

:- module(comp_missingobj,
    [
%       vis_all_objects_on_table/1,
%       vis_all_objects_inferred/3,
      comp_missingObjectTypes/2
    ]).

:-  rdf_meta
    comp_missingObjectTypes(r, r),
    current_objects_on_table(r,r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% visualisation canvas
:- use_module(library('mod_vis')).



%% comp_missingObjectTypes(+Table, -Missing)
%
% check which objects are on the table, use prob inference to determine which ones
% should be, compare both and create instances of the missing items with an existence
% probability as determined by the prob inference procedure
%
% @param Table        Instance of a KitchenTable on which the setup is to be completed
% @param MissingTypes List of object types that should be on the table given the current setup
%
comp_missingObjectTypes(Table, MissingTypes) :-

  get_timepoint(NOW),

  % call the probabilistic inference engine
  mod_probcog_tablesetting:required_objects(Table, RequiredObjects),

  % find detected objects on table Table
  findall(Curr, holds(on_Physical(Curr, Table), NOW), PerceivedObjects),

  print('perceived:'), print(PerceivedObjects),print('\n'),

  print('required:'), print(RequiredObjects),print('\n'),

  compute_missing_objects(PerceivedObjects, MissingTypes).



%% compute_missing_objects(+PerceivedObjects, -MissingTypes) is nondet.
%
% Compare the list of objects that are already on the table with those
% objects that are supposed to be there to find missing items.
%
% @param PerceivedObjects  List of objects detected on a table
% @param MissingTypes      List of object types that should be on that table given the current setup
%
compute_missing_objects(PerceivedObjects, MissingTypes) :-

  % get sorted list of elements that have a prob > 0 to be on the table
  latest_inferred_object_types(RequiredObjectTypes),

  % read types of objects on the table
  findall(T, (member(O, PerceivedObjects), rdf_has(O, rdf:type, T)), DetectedObjectTypes),

  % find all objects that should be on the table, but are not member of the set of observed objects
  findall(ReqO, (member(ReqO, RequiredObjectTypes), not(member(ReqO, DetectedObjectTypes))), MissingTypes).



% backwards compatibility
current_objects_on_table(T, Curr) :-
  objects_on_table(_, _),
  get_timepoint(NOW),
  holds(on_Physical(Curr, T), NOW).


% visualize all tables in the environment and all objects
% that are perceived on top of one of these tables
visualize_perceived_objects(PerceivedObjects) :-

  rdfs_instance_of(Table, knowrob:'KitchenTable'),
  add_object_perception(Table, _),

  findall(Obj, (member(Obj, PerceivedObjects), add_object_perception(Obj, _)), _).


visualize_missing_objects(MissingObjects) :-
  findall(Obj, (member(Obj, MissingObjects), add_object_perception(Obj, _)), _).

