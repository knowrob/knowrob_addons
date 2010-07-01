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
      comp_missingObjectTypes/2
    ]).


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

  % find detected objects on the table
  findall(Curr, current_objects_on_table(T, Curr), DetectedObjects),

  % call the probabilistic inference engine
  mod_probcog_tablesetting:required_objects(RequiredObjects),

  compute_missing_objects(DetectedObjects, RequiredObjects, MissingTypes).



%% compute_missing_objects(+AreOnTable, +ShouldBeOnTableWithProb, -Missing, -MissingTypes)
%
% compare the list of objects that are already on the table with that
% of objects that are supposed to be there to find missing items,
% create instances of these objects with the resp. probability
%
compute_missing_objects(DetectedObjects, RequiredObjects, MissingTypes) :-

  % find those elements that have a prob > 0 to be on the table
  findall(InfOT,  (member(InfO, RequiredObjects),
                   rdf_has(InfO, rdf:type, InfOT),
                   rdf_has(Inf, knowrob:objectActedOn, InfO),
                   rdfs_individual_of(Inf, knowrob:'TableSettingModelInference'),
                   rdf_has(Inf, knowrob:probability, InfProb),
                   term_to_atom(Prob, InfProb),
                   >(Prob, 0)), RequiredObjectTypes),

  % read types of objects on the table
  findall(T, (member(O, DetectedObjects), rdf_has(O, rdf:type, T)), DetectedObjectTypes),

  % find all objects that should be on the table, but are not member of the set of observed objects
  findall(ReqO, (member(ReqO, RequiredObjectTypes), not(member(ReqO, DetectedObjectTypes))), MissingTypes).






