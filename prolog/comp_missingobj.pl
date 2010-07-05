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
      vis_all_objects_on_table/1,
      vis_all_objects_inferred/3,
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
comp_missingObjectTypes(T, MissingTypes) :-

  get_timepoint(NOW),

  % find detected objects on table T
  findall(Curr, holds(on_Physical(Curr, T), NOW), DetectedObjects),

  % call the probabilistic inference engine
  mod_probcog_tablesetting:required_objects(RequiredObjects),

  compute_missing_objects(DetectedObjects, RequiredObjects, MissingTypes).



%% compute_missing_objects(+AreOnTable, +ShouldBeOnTableWithProb, -Missing, -MissingTypes)
%
% compare the list of objects that are already on the table with that
% of objects that are supposed to be there to find missing items,
% create instances of these objects with the resp. probability
%
compute_missing_objects(DetectedObjects, _RequiredObjects, MissingTypes) :-

  % get sorted list of elements that have a prob > 0 to be on the table
  latest_inferred_object_set(RequiredObjectTypes),


  % read types of objects on the table
  findall(T, (member(O, DetectedObjects), rdf_has(O, rdf:type, T)), DetectedObjectTypes),

  % find all objects that should be on the table, but are not member of the set of observed objects
  findall(ReqO, (member(ReqO, RequiredObjectTypes), not(member(ReqO, DetectedObjectTypes))), MissingTypes).


% TODO:

% * copID
% * jloID
% * query for last detection of an instance of the respective type
%


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % 
% % %
% % % TODO: update and remove the following
% % %
% % % 



current_objects_on_table(T, Curr) :-
get_timepoint(NOW),
holds(on_Physical(Curr, T), NOW).

mostLikelyObjects(Objs, Probs, InfObjs) :-
  findall(InfObj, (member(O,Objs),
                    rdf_has(O,rdf:type,T),
                    member(InfObj,Probs),
                    rdf_has(InfObj,rdf:type,T)), InfObjs).


mostLikelyObj(Objs, Probs, MLObj) :-
  mostLikelyObjects(Objs, Probs, InfObjs),
  predsort(compare_inference_probs2, InfObjs, [MLInfObj|_]),
  rdf_has(MLInfObj,rdf:type,Type),
  member(MLObj,Objs),
  rdf_has(MLObj,rdf:type,Type).


% mostLikelyObjCopID(Objs, Probs, MLObj, CID) :-
%   mostLikelyObj(Objs, Probs, MLObj),
%   rdf_has(MLObj, knowrob:copID, CID).


% inferMissingObjects(Instances) :-
%    objects_on_table(_,_),!,
%    required_objects(Instances).


missingObject(Table, MLObj, CID):-
     findall(Obj, current_objects_on_table(Table, Obj), Objs),
     findall(InfObj, (rdf_has(Inf, rdf:type, knowrob:'TableSettingModelInference'), rdf_has(Inf,knowrob:objectActedOn,InfObj)), Probs),
     mostLikelyObjCopID(Objs, Probs, MLObj, CID).


vis_all_objects_on_table(Table):-
  add_object_perception(Table, _),
  current_objects_on_table(Table, O),
  add_object_perception(O, _).

vis_all_objects_inferred(T,O,P):-
  currently_inferred_objects(O),
  rdf_has(Inf,knowrob:objectActedOn,O),
  rdf_has(Inf, rdf:type, knowrob:'TableSettingModelInference'),
  rdf_has(Inf,knowrob:probability,P),
  term_to_atom(N,P),
  N>T,
  add_object_perception(O,  _).





% % %% compare_inference_probs2(-Delta, +P1, +P2)
% % %
% % % compare two object classes with their resp. probabilities
% % %
% % compare_inference_probs2('>', I1, I2) :-
% %     probability_of_instance(I1,P1),
% %     probability_of_instance(I2,P2),
% %     term_to_atom(N1, P1),
% %     term_to_atom(N2, P2),
% %     N1 < N2.
% % 
% % compare_inference_probs2('<', I1, I2) :-
% %     probability_of_instance(I1,P1),
% %     probability_of_instance(I2,P2),
% %     term_to_atom(N1, P1),
% %     term_to_atom(N2, P2),
% %     N1>=N2.
% % 
% % 
% % probability_of_instance(I,P):-
% %    rdf_has(Inf,knowrob:objectActedOn,I),
% %    rdf_has(Inf,knowrob:probability, P).

