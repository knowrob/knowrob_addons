
/** <module> knowrob_sim_games

  Copyright (C) 2014-15 by Moritz Tenorth, Andrei Haidu

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

:- module(pizza_queries,
    [    
    check_pizza_short/1,
    check_pizza_ext/1,
    
    grasp_cup/4,
    pour_sauce/4,
    sauce_contact_overl/5,
    grasp_spoon/4,
    topping_transl/3,
    add_topping/4,
    add_toppings_during/4,
    first_topping_ev/2,
    smear_pizza_contact/3,
    smear_sauce_before/4,
    
    first_in_ev_list/2,
    last_in_ev_list/2,
    first_last_from_list/3,
    
    loop_pizza_exp
    ]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    add_topping(+, +, +, r),
    add_toppings_during(+, +, r, +).

%%
grasp_cup(EpInst, EventInst, Start, End) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for grasping events
	event_type(EventInst, knowrob:'GraspingSomething'),
	% check object acted on
	acted_on(EventInst, knowrob:'Cup').

%%
pour_sauce(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for particle translation
	event_type(EventInst, knowrob_sim:'ParticleTranslation'),
	% check for the type of the particle
	particle_type(EventInst, knowrob:'Sauce').
	
%%
sauce_contact_overl(EpInst, EventInst, OverlInst, Start, End) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for touching situation
	event_type(EventInst, knowrob_sim:'TouchingSituation'),
	% check that the container grasping overlaps the sauce contacts
	comp_overlapsI(OverlInst, EventInst),
	% check that Obj1 is only in contact with Obj2
	only_in_contact(EventInst, knowrob:'Sauce', knowrob:'Pizza').

%%
grasp_spoon(EpInst, EventInst, Start, End) :-	
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, Start, End),
	% check for grasping events
	event_type(EventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(EventInst, knowrob:'Spoon').

%%
topping_transl(EpInst, EventInst, DuringEventInst) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, _, _),
	% check for particle translation
	event_type(EventInst, knowrob_sim:'ParticleTranslation'),	
	% check that the particle transl happened during topping on the spoon
	comp_duringI(EventInst, DuringEventInst).	
	
%%
add_topping(EpInst, EventInst, DuringEventInst, Topping) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, _, _),
	% check for touching situation
	event_type(EventInst, knowrob_sim:'TouchingSituation'),
	% check that the topping is in contact with the tool
	in_contact(EventInst, Topping, knowrob:'Spoon'),
	% check that the topping manipulation happened during the tool grasp
	comp_duringI(EventInst, DuringEventInst),
	% check that during the topping spreading only ONE particle translation happened
	findall(TranslInst, topping_transl(EpInst, TranslInst, EventInst), AllToppingTranslInst),
	% check the size for strictly one
	length(AllToppingTranslInst, Length), Length == 1.

%%
add_toppings_during(EpInst, DuringEventInst, Topping, AllToppingEventInst) :-
	% check for all topping events of the given type 
	findall(AddToppingEventInst,
			add_topping(EpInst, AddToppingEventInst, DuringEventInst, Topping),
			AllToppingEventInst),
	% check that at least once the given topping has been added
	length(AllToppingEventInst, AllToppingLength), AllToppingLength > 0.

%%
first_topping_ev(FirstToppingEventInst, AllToppingEvents):-
	% get the first event from the list
	first_in_ev_list(AllToppingEvents, FirstToppingEventInst).

%%
smear_pizza_contact(EpInst, EventInst, BeforeEventInst) :-
	% get events which occurred in the experiments
	sg_occurs(EpInst, EventInst, _, _),
	% check for touching situation
	event_type(EventInst, knowrob_sim:'TouchingSituation'),	
	% check objects in contact
	in_contact(EventInst, knowrob:'Spoon', knowrob:'Pizza'),
	% check that the contact happened before starting to put the toppings
	comp_beforeI(EventInst, BeforeEventInst).

%%
smear_sauce_before(EpInst, BeforeEventInst, Start, End) :-	
	% check all contacts between the spoon and the pizza, before adding any topping
	findall(PizzaContactInst, 
			smear_pizza_contact(EpInst, PizzaContactInst, BeforeEventInst), 
			AllSmearPizzaContacts),	
	% get the first and last event from the event list
	first_last_from_list(AllSmearPizzaContacts, FirstEvent, LastEvent),	
	% get the start from the first event
	sg_occurs(EpInst, FirstEvent, Start, _),	
	% get the end from the last event
	sg_occurs(EpInst, LastEvent, _, End).


%% get the first event from the list
first_in_ev_list([FirstEv], FirstEv).

first_in_ev_list([H,K|T], First):-
	% H happened before K, so use H to compare agains the remaining Tail
	comp_beforeI(H,K),
	% recursively check for the first event in the remaining Tail
	first_in_ev_list([H|T], First).

first_in_ev_list([H,K|T], First):-
	% H happened after K, so use K to compare agains the remaining Tail
	comp_afterI(H,K),
	% recursively check for the first event in the remaining Tail
	first_in_ev_list([K|T], First).


%% get the last event from the list
last_in_ev_list([LastEv], LastEv).

last_in_ev_list([H,K|T], Last):-
	% H happened after K, so use H to compare agains the remaining Tail
	comp_afterI(H,K),
	% recursively check for the last event in the remaining Tail
	last_in_ev_list([H|T], Last).

last_in_ev_list([H,K|T], Last):-
	% H happened before K, so use K to compare against the remaining Tail
	comp_beforeI(H,K),
	% recursively check for the last event in the remaining Tail
	last_in_ev_list([K|T], Last).

 
%% get the start and ending event inst from the list
first_last_from_list(EventInstList, FirstEventInst, LastEventInst) :-
	% get the first event
	first_in_ev_list(EventInstList, FirstEventInst),
	% get the last event
	last_in_ev_list(EventInstList, LastEventInst).
	
%%
check_pizza_short(EpInst) :-

	% check if the container has been grasped
	grasp_cup(EpInst, GraspCupEventInst, _, _),	
	
	% check for particle translation
	pour_sauce(EpInst, SauceTranslEventInst, _, _),	
	
	% check that the particle transl happened during the container grasp
	comp_duringI(SauceTranslEventInst, GraspCupEventInst),
	
	% check that the sauce is only in contact with the pizza during the grasp
	sauce_contact_overl(EpInst, _, GraspCupEventInst, _, _),
			
	% check that the spoon has been grasped
	grasp_spoon(EpInst, GraspSpoonEventInst, _, _),
	
	% check for add toppings events of the given type, while the spoon has been grasped
	add_toppings_during(EpInst, GraspSpoonEventInst, knowrob:'Cheese', AllCheeseToppingEventInst),	

	% check for add topping events of the given type, while the spoon has been grasped
	add_toppings_during(EpInst, GraspSpoonEventInst, knowrob:'Champignon', AllChampignonToppingEventInst),

	% append the two topping lists
	append(AllCheeseToppingEventInst, AllChampignonToppingEventInst, AllToppingEvents), 

	% get the first add topping event
	first_topping_ev(FirstToppingEventInst, AllToppingEvents),

	% check for smearing the pizza sauce, before any topping has been added	
	smear_sauce_before(EpInst, FirstToppingEventInst, SmearingStart, SmearingEnd),
	
	
	
	% save the positon of the links in the database
	get_links_positions(EpInst, 'Sauce', SmearingStart, 'pizza_sauce_start_pos'),
	
	% save the positon of the links in the database
	get_links_positions(EpInst, 'Sauce', SmearingEnd, 'pizza_sauce_end_pos'),
	
	% save the spoon smearing trajectory
	get_collision_traj(EpInst, 'Spoon', 'spoon_body_link', 'spoon_head_collision',
		 SmearingStart, SmearingEnd, 'pizza_smearing_traj'),
	
	% get the start and end of the first topping event
	sg_occurs(EpInst, FirstToppingEventInst, FirstToppingStart, FirstToppingEnd),
	% save the add topping trajectory
	get_collision_traj(EpInst, 'Spoon', 'spoon_body_link', 'spoon_head_collision',
		 FirstToppingStart, FirstToppingEnd, 'pizza_add_topping_traj').
	


%%
check_pizza_ext(EpInst) :-
	
	%%% GRASP CUP
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspCupInst, _, _),
	% check for grasping events
	event_type(GraspCupInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspCupInst, knowrob:'Cup'),
	
	%%% POUR SAUCE
	% get events which occurred in the experiments
	sg_occurs(EpInst, SauceTranslEventInst, _, _),
	% check for particle translation
	event_type(SauceTranslEventInst, knowrob_sim:'ParticleTranslation'),
	% check for the type of the particle
	particle_type(SauceTranslEventInst, knowrob:'Sauce'),	
	% check that the particle transl happened during the container grasp
	comp_duringI(SauceTranslEventInst, GraspCupInst),	
	
	%%% SAUCE CONTACT
	% get events which occurred in the experiments
	sg_occurs(EpInst, SauceContactEventInst, _, _),
	% check for touching situation
	event_type(SauceContactEventInst, knowrob_sim:'TouchingSituation'),	
	% check that the container grasping overlaps the sauce contacts
	comp_overlapsI(GraspCupInst, SauceContactEventInst),
	% check that the sauce is only in contact with the pizza
	only_in_contact(SauceContactEventInst, knowrob:'Sauce', knowrob:'Pizza'),
	
	%%% GRASP TOOL
	% get events which occurred in the experiments
	sg_occurs(EpInst, GraspSpoonEventInst, _, _),
	% check for grasping events
	event_type(GraspSpoonEventInst, knowrob:'GraspingSomething'),	
	% check object acted on
	acted_on(GraspSpoonEventInst, knowrob:'Spoon'),	
	% check that the container grasp happened before the spatula grasp
	comp_beforeI(GraspCupInst, GraspSpoonEventInst),
	
	%TODO generalize topping adding using the ontology
	%%% ADD CHEESE TOPPING
	% check for all add cheese topping events 
	findall(CheeseEventInst,
			add_topping(EpInst, CheeseEventInst, GraspSpoonEventInst, knowrob:'Cheese'),
			AllCheeseToppingInst),
	% check that at least once a cheese topping has been added
	length(AllCheeseToppingInst, CheeseLength), CheeseLength > 0,
			
	%%% ADD CHAMPIGNON TOPPING 
	% check for all add champignon topping events
	findall(ChampignonEventInst,
			add_topping(EpInst, ChampignonEventInst, GraspSpoonEventInst, knowrob:'Champignon'),
			AllChampignonToppingInst),
	% check that at least once a champignon topping has been added
	length(AllChampignonToppingInst, ChampignonLength), ChampignonLength > 0, 

	%%% SMEARING SAUCE	
	% append the two topping lists
	append(AllCheeseToppingInst, AllChampignonToppingInst, AllToppingEvents), 
	% get the first add topping event
	first_topping_ev(FirstToppingEventInst, AllToppingEvents),
	% check all contacts between the spoon and the pizza, before adding topping
	findall(PizzaContactInst, 
			smear_pizza_contact(EpInst, PizzaContactInst, FirstToppingEventInst), 
			AllSmearPizzaContacts),	
	% get the first and last event from the event list
	first_last_from_list(AllSmearPizzaContacts, FirstEvent, LastEvent),	
	% get the start from the first event
	sg_occurs(EpInst, FirstEvent, SmearingStart, _),	
	% get the end from the last event
	sg_occurs(EpInst, LastEvent, _, SmearingEnd),
	
	% save the positon of the links in the database
	get_links_positions(EpInst, 'Sauce', SmearingStart, 'pizza_sauce_start_pos'),
	
	% save the positon of the links in the database
	get_links_positions(EpInst, 'Sauce', SmearingEnd, 'pizza_sauce_end_pos'),
	
	% save the spoon smearing trajectory
	get_collision_traj(EpInst, 'Spoon', 'spoon_body_link', 'spoon_head_collision',
		 SmearingStart, SmearingEnd, 'pizza_smearing_traj'),
	
	% get the start and end of the first topping event
	sg_occurs(EpInst, FirstToppingEventInst, FirstToppingStart, FirstToppingEnd),
	% save the add topping trajectory
	get_collision_traj(EpInst, 'Spoon', 'spoon_body_link', 'spoon_head_collision',
		 FirstToppingStart, FirstToppingEnd, 'pizza_add_topping_traj').
		 

%%
loop_pizza_exp :-
	
	% load all experiments
	sg_load_experiments('/home/haidu/sr_experimental_data/pizza'),
	
	% connect to the raw data 
	connect_to_db('SIM-pizza-db'),	
	
	% get the instance of the current experiment
	exp_inst(EpInst),
	
	% check if pancake succesfully created
	%check_pizza_ext(EpInst),
	check_pizza_short(EpInst),
	
	% get experiment tag
	exp_tag(EpInst, ExpTag),
	write('** In '), write(ExpTag), write(' the pizza was succesfully created, exp instance: '), write(EpInst),  nl.



	
	
	