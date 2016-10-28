
/** <module> robcog_mongo_interface

  Copyright (C) 2016 by Andrei Haidu

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

:- module(robcog_mongo_interface,
  [
		mongo_robcog_conn/1,
        mongo_robcog_query/1
  ]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Create the mongo robcog interface object
% (makes sure the interface is only created once)

% set flag
:- assert(connection_flag(fail)).

% from knowrob_mongo.pl
mongo_robcog_conn :-
    mongo_robcog_conn(_).

% check flag, then init interface
mongo_robcog_conn(MongoRobcog) :-
    connection_flag(fail), 	
    jpl_new('org.knowrob.knowrob_robcog.MongoRobcogConn',[], MongoRobcog),
    retract(connection_flag(fail)),
    assert(connection_flag(MongoRobcog)),!.

% if set, return object
mongo_robcog_conn(MongoRobcog) :-
    connection_flag(MongoRobcog).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Create the mongo robcog query object
% (makes sure the interface is only created once)

% set flag
:- assert(query_flag(fail)).

mongo_robcog_query :-
  mongo_robcog_query(_).

% check flag, then init interface
mongo_robcog_query(MongoQuery) :-
    query_flag(fail),  
    mongo_robcog_conn(MongoRobcog), 
    jpl_new('org.knowrob.knowrob_robcog.MongoRobcogQueries',[MongoRobcog], MongoQuery),
    retract(query_flag(fail)),
    assert(query_flag(MongoQuery)),!.

% if set, return object
mongo_robcog_query(MongoQuery) :-
    query_flag(MongoQuery).