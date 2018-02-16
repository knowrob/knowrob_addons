/** <module> comp_missingobj

  Description:
    Determine which objects are missing on the table by combining
    perceptions, knowledge processing and probabilistic reasoning.


  Copyright (C) 2010 by Moritz Tenorth

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

@author Moritz Tenorth
@license BSD
*/

:- module(comp_missingobj,
    [
%       vis_all_objects_on_table/1,
%       vis_all_objects_inferred/3,
      comp_missingObjectTypes/4,
      comp_missingObjectTypes/3,
      current_objects_on_table/2,
      visualize_perceived_objects/1,
      visualize_missing_objects/1
    ]).

:-  rdf_meta
    comp_missingObjectTypes(r, r, r, r),
    current_objects_on_table(r,r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% visualisation canvas
% :- use_module(library('mod_vis')).



%% comp_missingObjectTypes(+Table, -PerceivedObjects, -MissingInstances, -MissingTypes)
%
% check which objects are on the table, use prob inference to determine which ones
% should be, compare both and create instances of the missing items with an existence
% probability as determined by the prob inference procedure
%
%
comp_missingObjectTypes(Table, PerceivedObjects, MissingInstances, MissingTypes) :-

  current_time(NOW),

  % call the probabilistic inference engine
  mod_probcog_tablesetting:required_objects(Table, _RequiredObjects),

  % find detected objects on table Table
  findall(Curr, holds(on_Physical(Curr, Table), NOW), PerceivedObjects),

  compute_missing_objects(PerceivedObjects, MissingInstances, MissingTypes).



%% comp_missingObjectTypes(PerceivedObjects, MissingInstances, MissingTypes)
%
% simplified predicate for the ROS Fall School
%
% Does not take the pose of objects into account, but assumes all perceived
% objects to be evidence
%
%
comp_missingObjectTypes(PerceivedObjects, MissingInstances, MissingTypes) :-

  % call the probabilistic inference engine
  mod_probcog_tablesetting:required_objects(_),

  % determine perceived objects
  findall(Obj, current_objects_on_table(_, Obj), PerceivedObjects),

  compute_missing_objects(PerceivedObjects, MissingInstances, MissingTypes).

% actual version:
% current_objects_on_table(T, Curr) :-
%   objects_on_table(_, _),
%   current_time(NOW),
%   holds(on_Physical(Curr, T), NOW).

% simplified version for the ROS Fall School
current_objects_on_table(_, Curr) :-
        latest_perceptions_of_types('http://knowrob.org/kb/knowrob.owl#HumanScaleObject', LatestPerceptions),
        member(Perc, LatestPerceptions),
        rdf_has(Perc, knowrob:objectActedOn, Curr).



%% compute_missing_objects(+PerceivedObjects, -MissingTypes) is nondet.
%
% Compare the list of objects that are already on the table with those
% objects that are supposed to be there to find missing items.
%
% @param PerceivedObjects  List of objects detected on a table
% @param MissingTypes      List of object types that should be on that table given the current setup
%
compute_missing_objects(PerceivedObjects, MissingInstances, MissingTypes) :-

  % get sorted list of elements that have a prob > 0 to be on the table
  latest_inferred_object_set(RequiredObjects),
  findall(ObjT, (member(Obj, RequiredObjects), rdf_has(Obj, rdf:type, ObjT)), RequiredObjectTypes),

  % read types of objects on the table
  findall(T, (member(O, PerceivedObjects), rdf_has(O, rdf:type, T)), PerceivedObjectTypes),

  % find all objects that should be on the table, but are not member of the set of observed objects
  findall(ReqO,  (member(ReqO,  RequiredObjects),     not(member(ReqO,  PerceivedObjects))),    MissingInstances),
  findall(ReqOT, (member(ReqOT, RequiredObjectTypes), not(member(ReqOT, PerceivedObjectTypes))), MissingTypes).




% visualize all tables in the environment and all objects
% that are perceived on top of one of these tables
visualize_perceived_objects(PerceivedObjects) :-

  findall(Table, (rdfs_individual_of(Table, knowrob:'KitchenTable'), add_object_perception(Table, _)), _),
  findall(Obj, (member(Obj, PerceivedObjects), add_object_perception(Obj, _)), _).


visualize_missing_objects(MissingObjects) :-
  findall(Obj, (member(Obj, MissingObjects), add_object_perception(Obj, _)), _).

