/** <module> mod_probcog

  Interface to the ProbCog probabilistic reasoning framework

  Copyright (C) 2010 Moritz Tenorth
  All rights reserved.

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

:- module(mod_probcog,
    [
      probcog_query/2,
      probcog_query/3
    ]).

:- use_module(library('jpl')).



%% probcog_query(+Model, +QueryPredicates, -Res) is nondet.
% 
% Send a query to the ProbCog probabilistic reasoning engine using the
% specified model for the given set of query predicates.
%
% The result is a list of answers for each query, which by themselves are
% lists of bindings. Example:
% [ [['q1_value1', 'q1_prob1'], ['q1_value2', 'q1_prob2']], [['q2_value1', 'q2_prob1']] ]
% 
% There are two kinds of queries: When asking for a predicate with query variables
% (marked with a preceding '?'), such as
% 
% ?- probcog_query(['usesAnyIn(person1, ?, meal1)', 'sitsAtIn(person1, ?, meal1)'], Res).
% 
% the system returns only the bindings for each of these variables, together with
% their probabilities. If there are several query variables, the bindings are joined
% with an underscore. Example:
%
% ?- probcog_query(['usesAnyIn(person1, ?, meal1)', 'sitsAtIn(person1, ?, meal1)'], Res).
% Res = [[['TableKnife_meal1', '1.0'], ['Bowl-Eating_meal1', '0.638'],], [['Seat3', '0.17'], ['Seat1', '0.41']]]
%
% The second kind of query asks for all possible bindings of a predicate and returns
% the complete term:
% 
% ?- probcog_query(['usesAnyIn'], Res).
% Res = [[['usesAnyIn(person1, TableKnife, meal1)', '1.0'], ['usesAnyIn(person1, Bowl-Eating, meal1)', '0.67']]]
% 
%
% Evidence is read by the performInference procedure depending on the model to be
% used and the wrapper predicates specified in the corresponding Prolog module.
%
% @param Model           The model that is to be used for inference
% @param QueryPredicates Predicates whose most likely values are to be inferred, List of the form [q1, q2, q3]
% @param Res             Variable bindings for the query predicates as a list [[binding_1_q1,binding_1_q2,binding_1_q3],[binding_2_q1,binding_2_q2,binding_2_q3],..]
% 
probcog_query(Model, QueryPredicates, Res) :-
  jpl_list_to_array(QueryPredicates, Array),
  probcog_module_name(Model, ModuleName),
  jpl_call('edu.tum.cs.probcog.prolog.PrologInterface', 'performInference', [Model, ModuleName, Array], ResArray),
  arrays_to_lists(ResArray, Res).


%% probcog_query(+QueryPredicates, -Res) is nondet.
%
% Send a query to the ProbCog probabilistic reasoning engine using the default
% model for the given set of query predicates.
%
% @param QueryPredicates Predicates whose most likely values are to be inferred, List of the form [q1, q2, q3]
% @param Res             Variable bindings for the query predicates as a list [[binding_1_q1,binding_1_q2,binding_1_q3],[binding_2_q1,binding_2_q2,binding_2_q3],..]
% 
probcog_query(QueryPredicates, Res) :-
  probcog_model(QueryPredicates, Model),
  probcog_query(Model, QueryPredicates, Res).


%% probcog_model(+QueryPredicates, -Model) is det.
%
% Determine the default model for a set of query predicates.
%
probcog_model(_, 'tableSetting_fall10').


%% Mapping from model name to corresponding prolog module name
%
% Required for JPL interface to query evidences properly
probcog_module_name('tableSetting_fall10', 'mod_probcog_tablesetting').
