/** <module> robcog_event_logic

  Copyright (C) 2016 Andrei Haidu

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

  @author Andrei Haidu
  @license BSD
*/


:- module(robcog_event_logic,
    [
        r_set_ep/0,
        r_set_ep/1,
        r_get_ep/1,
        r_rm_ep/0,
        r_class/2,
        r_contact/1,
        r_contact/3,
        r_grasp/1,
        r_grasp/2,
        r_holds/2,
        r_holds/3
    ]).


% returns the namspace when outputting values
:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_u, 'http://knowrob.org/kb/knowrob_u.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(log_u, 'http://knowrob.org/kb/robcog_log.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(u_map, 'http://knowrob.org/kb/u_map.owl#', [keep(true)]).


% :- meta_predicate cram_holds(0, ?, ?).
% :- discontiguous cram_holds/2.
%:- meta_predicate holds(0, ?, ?).
%% :- meta_predicate ep(0, ?).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    class(r,r),
    contact(r),
    contact(r, r, ?),
    grasp(r),
    grasp(r, ?),

    holds(:, +),
    holds(:, ?, ?).

%% r_set_ep() is nondet.
%
% Sets the current working episode instance
%
r_set_ep :-
  rdf_has(EpInst, rdf:type, knowrob:'UnrealExperiment'),
  set_ep(EpInst).

%% r_set_ep(+EpInst) is nondet.
%
% Sets the current working episode instance
%
% @param EpInst Identifier of the instance
%
r_set_ep(EpInst) :-
    nb_setval(ep_inst, EpInst).

%% r_get_ep(-EpInst) is nondet.
%
% Gets the current working episode instance
%
% @param EpInst Identifier of the instance
%
r_get_ep(EpInst) :-
    nb_getval(ep_inst, EpInst).


%% r_rm_ep(-EpInst) is nondet.
%
% Remove the current working episode instance
%
r_rm_ep :-
    nb_delete(ep_inst).


%% r_unify_time_interval(+StartValue, +EndValue, ?ST, ?ET) is nondet.
%
% Unifies ST and ET depending on their instantiation (var, nonvar)
%
% @param StartValue Numeric value of the event start timepoint
% @param EndValue Numeric value of the event start timepoint
% @param ST Numeric value of the start timepoint to check
% @param ET Numeric value of the end timepoint to check
%
r_unify_time_interval(StartValue, EndValue, ST, ET) :-
    (   var(ST), var(ET)            % if ST and ET are variables (not inst)
            -> ST = StartValue,     % bind with event time variables
               ET = EndValue
               ;
        var(ST), nonvar(ET)         % if ST var and ET nonvar (one is inst)
            -> ET >= StartValue,    % check if ET is in the interval
               ET =< EndValue,
               ST = StartValue      % bind the non inst var
               ;
        nonvar(ST), var(ET)         % if ST nonvar and ET var  (one is inst)
            -> ST >= StartValue,    % check if ST is in the interval
               ST =< EndValue,
               ET = EndValue        % bind the non inst var
               ;
        % default, if ST and ET nonvar
               ST =< ET,
               ST >= StartValue,    % check if ST is in the interval
               ST =< EndValue,
               ET >= StartValue,    % check if ET is in the interval
               ET =< EndValue
    ).


%% r_class(?Inst, ?Class) is nondet.
%
% Gets the type (Class) of the instance
%
% @param Inst Identifier of the instance
% @param Class Identifier of the instance type (Class)
%
r_class(Inst, Class) :-
    rdf_has(Inst, rdf:type, Class),
    not(rdf_equal(Class, owl:'NamedIndividual')).


%% r_contact(?EvInst) is nondet.
%
% Checks for contact events in the current episode
%
% @param EvInst Identifier of the contact instance
%
r_contact(EvInst) :-
    get_ep(EpInst),
    rdf_has(EpInst, knowrob:'subAction', EvInst),
    rdf_has(EvInst, rdf:type, knowrob_u:'TouchingSituation').


%% r_contact(?ObjInst1, ?ObjInst2, ?StartEndList) is nondet.
%
% Checks for contacts between the two instances
%
% @param ObjInst1 Identifier of the first object instance
% @param ObjInst2 Identifier of the second object instance
% @param StartEndList List of the start and end timepoint [ST, ET]
%
r_contact(ObjInst1, ObjInst2, [ST, ET]) :-
    contact(EvInst),
    rdf_has(EvInst, knowrob_u:'inContact', ObjInst1),
    rdf_has(EvInst, knowrob_u:'inContact', ObjInst2),
    ObjInst1 \== ObjInst2,
    rdf_has(EvInst, knowrob:'startTime', StartInst),
    rdf_has(EvInst, knowrob:'endTime', EndInst),
    time_point_value(StartInst, StartValue),
    time_point_value(EndInst, EndValue),
    r_unify_time_interval(StartValue, EndValue, ST, ET).


%% r_contact(?ObjInst1, ?ObjInst2, +T) is nondet.
%
% Checks for contacts between the two instances
%
% @param ObjInst1 Identifier of the first object instance
% @param ObjInst2 Identifier of the second object instance
% @param T Numeric value of the timepoint
%
r_contact(ObjInst1, ObjInst2, T) :-
    number(T),
    contact(ObjInst1, ObjInst2, [T, T]).


%% r_grasp(?EvInst) is nondet.
%
% Checks for grasp events in the current episode
%
% @param EvInst Identifier of the grasp instance
%
r_grasp(EvInst) :-
    get_ep(EpInst),
    rdf_has(EpInst, knowrob:'subAction', EvInst),
    rdf_has(EvInst, rdf:type, knowrob:'GraspingSomething').


%% r_grasp(?ObjInst, ?StartEndList) is nondet.
%
% Checks for grasp event with start and end time
%
% @param ObjInst Identifier of the object instance
% @param StartEndList List of the start and end timepoint [ST, ET]
%
r_grasp(ObjInst, [ST, ET]) :-
    grasp(EvInst),
    rdf_has(EvInst, knowrob:'objectActedOn', ObjInst),
    rdf_has(EvInst, knowrob:'startTime', StartInst),
    rdf_has(EvInst, knowrob:'endTime', EndInst),
    time_point_value(StartInst, StartValue),
    time_point_value(EndInst, EndValue),
    r_unify_time_interval(StartValue, EndValue, ST, ET).


%% r_grasp(?ObjInst1, ?ObjInst2, +T) is nondet.
%
% Checks for grasp event at timepoint
%
% @param ObjInst Identifier of the object instance
% @param T Numeric value of the timepoint
%
r_grasp(ObjInst, T) :-
    number(T),
    grasp(ObjInst, [T, T]).














%=====================================================================
% ========= TESTING PHASE =========

%% holds(+contact:compound, +T) is nondet.
%
% Usage: holds(contact(ObjClass1, ObjClass2), +T).
% e.g.   holds(contact(knowrob:'KitchenIsland', knowrob:'Bowl'), 3.4).
%
% Check the two object classes are in contact at the given timepoint
%
% @param ObjClass1 Identifier of the first object class
% @param ObjClass2 Identifier of the second object class
%
r_holds(contact(ObjClass1, ObjClass2), TStartValue, TEndValue) :-
    contact(ObjClass1, ObjClass2, TStartValue, TEndValue).













    %% % get the objects in contact
    %% rdf_has(EvInst, knowrob_u:'inContact', O1Inst),
    %% rdf_has(EvInst, knowrob_u:'inContact', O2Inst),    
    %% % make sure the objects are distinct
    %% O1Inst \== O2Inst,
    %% % check for the right type of contacts    
    %% rdf_has(O1Inst, rdf:type, O1Type),
    %% rdf_has(O2Inst, rdf:type, ObjClass2),
    %% % avoid NamedIndividual returns for the object types
    %% not(rdf_equal(O1Type, owl:'NamedIndividual')),
    %% not(rdf_equal(ObjClass2, owl:'NamedIndividual')).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% get the instance of the episode
%% occurs2 :-
%%     writeln('Occurs2').


%% comp_above_of(Top, Bottom) :-
%%     get_timepoint(T),
%%     holds(comp_above_of(Top, Bottom), T).



%% holds(+BelowOf:compound, +T) is nondet.
%
% Usage: holds(comp_above_of(?Top, ?Bottom), +T)
%
% Check if Bottom has been in the area of and below Top at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
%% holds(comp_above_of(Top, Bottom),T) :-


%%     % get object center for Top
%%     object_detection(Top, T, VPT),
%%     rdf_triple(knowrob:eventOccursAt, VPT,    TopMatrix),
%%     rdf_triple(knowrob:m03, TopMatrix, TCxx),strip_literal_type(TCxx, TCx),atom_to_term(TCx,TX,_),
%%     rdf_triple(knowrob:m13, TopMatrix, TCyy),strip_literal_type(TCyy, TCy),atom_to_term(TCy,TY,_),
%%     rdf_triple(knowrob:m23, TopMatrix, TCzz),strip_literal_type(TCzz, TCz),atom_to_term(TCz,TZ,_),

%% %     rdf_triple(knowrob:heightOfObject, Top, literal(type(_,Th))),atom_to_term(Th,TH,_),

%%     % query for objects at center point
%%     objectAtPoint2D(TX,TY,Bottom),

%%     % get height of objects at center point
%%     object_detection(Bottom, T, VPB),
%%     rdf_triple(knowrob:eventOccursAt, VPB, BottomMatrix),
%%     rdf_triple(knowrob:m23, BottomMatrix, BCzz), strip_literal_type(BCzz, BCz),atom_to_term(BCz,BZ,_),
%% %     rdf_triple(knowrob:heightOfObject, Bottom, literal(type(_,Bh))),atom_to_term(Bh,BH,_),

%% %     print('bottom height:'), print(BH),

%%     % the criterion is if the difference between them is less than epsilon=5cm
%%     <( BZ, TZ),
%%     Top \= Bottom.



%% holds_g(Goal, T) :-
%%     call(Goal, _, _, StartValue, EndValue),
%%     T >= StartValue, T =< EndValue.

%% holds(+contact:compound, +T) is nondet.
%
% Usage: holds(contact(ObjClass1, ObjClass2), +T).
% e.g.   holds(contact(knowrob:'KitchenIsland', knowrob:'Bowl'), 3.4).
%
% Check the two object classes are in contact at the given timepoint
%
% @param ObjClass1 Identifier of the first object class
% @param ObjClass2 Identifier of the second object class
% @param T Numeric value of the time point
%
%% holds(contact(ObjClass1, ObjClass2), T) :-
%%     contact(ObjClass1, ObjClass2, StartValue, EndValue),
%%     T >= StartValue, T =< EndValue.

%% holds(+contact:compound, +T, ?EvInst) is nondet.
%
% Usage: holds(contact(ObjClass1, ObjClass2), +T, ?EvInst).
% e.g.   holds(contact(knowrob:'KitchenIsland', knowrob:'Bowl'), 3.4, EvInst).
%
% Check the two object classes are in contact at the given timepoint
% return the event instance.
%
% @param ObjClass1 Identifier of the first object class
% @param ObjClass2 Identifier of the second object class
% @param T Numeric value of the time point
% @param EvInst  Identifier of the generated instance of event Class
%
%% holds(contact(ObjClass1, ObjClass2), T, EvInst) :-
%%     contact(ObjClass1, ObjClass2, StartValue, EndValue, EvInst),
%%     T >= StartValue, T =< EndValue.




%% holds(+grasp:compound, +T) is nondet.
%
% Usage: holds(grasp(ObjClass1, ObjClass2), +T).
% e.g.   holds(grasp(knowrob:'Bowl'), 3.4).
%
% Check the two object classes are in contact at the given timepoint
%
% @param ObjClass Identifier of the grasped object
%
r_holds(grasp(ObjClass), T) :-
    grasp(ObjClass, StartValue, EndValue),
    T >= StartValue, T =< EndValue.