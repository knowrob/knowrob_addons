/** <module> knowrob_learning

  Copyright (C) 2016 Asil Kaan Bozcuoglu
  
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

  @author Asil Kaan Bozcuoglu
  @license BSD
*/

:- module(knowrob_reinforcement,
    [
      choose_map/2,
      define_goal/3,
      choose_state_factory/3,
      start_simulation/3,
      import_tf/2,
      use_simulation_data/1,
      visualize_trajectory/2,
      trajectory_duration/2,
      trajectory_length/3
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/mongo')).

choose_map(SemanticMapPath, JavaListMap) :-
   owl_parse(SemanticMapPath),
   owl_individual_of(A, knowrob:'SemanticEnvironmentMap'), 
   %marker_update(object(A)), !,
   (( A = 'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j', 
      jpl_new('org.knowrob.reinforcement.KitchenWorld', [], KitchenWorld),
      jpl_call(KitchenWorld, 'generateDomain', [], Domain),
      JavaListMap = [KitchenWorld, Domain]
   ); JavaListMap = []).

define_goal(reach(GoalPositionX, GoalPositionY, GoalPositionZ), Offset, JavaListMap) :-
   jpl_new('org.knowrob.reinforcement.KitchenTF', [GoalPositionX, GoalPositionY, GoalPositionZ, GoalPositionX, GoalPositionY, GoalPositionZ, Offset], KitchenTf),
   jpl_new('org.knowrob.reinforcement.KitchenEnvironment', [GoalPositionX, GoalPositionY, GoalPositionZ, GoalPositionX, GoalPositionY, GoalPositionZ], KitchenEnv),
   ArgList = ['org.knowrob.reinforcement.KitchenEnvironment'],
   jpl_new( '[Ljava.lang.String;', ArgList, Arg),
   jpl_call('org.knowrob.reinforcement.BasicBehavior', 'runNode', [KitchenEnv, Arg], _X),
   jpl_call(KitchenEnv, 'setTF', [KitchenTf], _Y),
   jpl_call(KitchenEnv, 'sleep', [5000], _Z),
   JavaListMap = [KitchenTf, KitchenEnv].

choose_state_factory(HashClass, Parameters, HashFactory) :-
   jpl_new(HashClass, Parameters, HashFactory).

start_simulation(JavaListMap, algorithm(Alg), NoOfTrials) :-
    nth0(1, JavaListMap, Domain), 
    nth0(3, JavaListMap, KitchenEnv),
    nth0(4, JavaListMap, HashFactory),
    %jpl_call(KitchenEnv, 'getCurrentPoses', [], PoseList),
    %jpl_new('burlap.statehashing.simple.SimpleHashableStateFactory', [], HashFactory),
    jpl_new(Alg, [Domain, 0.99, HashFactory, 0.0, 0.5, 0.3], LearningAgent),
    forall(between(0,NoOfTrials,_J), (run_simulation(LearningAgent, KitchenEnv, _Episode))).

run_simulation(LearnAgent, Environment, Episode) :-
    jpl_call(LearnAgent, 'runLearningEpisode', [Environment], Episode).

import_tf(ListOfTrials, Path) :- 
    forall(member(Trial, ListOfTrials), 
           (
            atom_concat('mongoexport --db roslog --collection ', Trial, FirstPartOfCommand),
            atom_concat(Path, Trial, FullPath),
            atom_concat(' --out ', FullPath, FilePrefixCommand),
            atom_concat(FilePrefixCommand, '.json', SecondPartOfCommand),
            atom_concat(FirstPartOfCommand, SecondPartOfCommand, Command),
            jpl_call('java.lang.Runtime', 'getRuntime', [], Runtime),
            jpl_call(Runtime, 'exec', [Command], _X)
           )).

:- assert(use_sim_data(fail)).
use_simulation_data :- use_simulation_data(_).

use_simulation_data(Trials) :-
    use_sim_data(fail),
    retract(use_sim_data(fail)),
    assert(use_sim_data(Trials)),!.

use_simulation_data(Trials) :-
    use_sim_data(Trials).

visualize_trajectory(Trial, Link) :-
    get_tf_db(Trial, TfMemory),
    mng_query_latest(Trial, one(DBObjLatest), 'transforms.header.stamp', 'timepoint_1445000000'), %we can set a timepoint which is > 1445000000 since simulation time starts from 0
    jpl_call('org.knowrob.reinforcement.BasicBehavior', 'getTimestamp', [DBObjLatest], EndTime),
    mng_query_earliest(Trial, one(DBObjEarliest), 'transforms.header.stamp', 'timepoint_0'),
    jpl_call('org.knowrob.reinforcement.BasicBehavior', 'getTimestamp', [DBObjEarliest], StartTime),
    marker(trajectory(Link), Trj, Trial), marker_type(Trj, sphere),
    marker_update(Trj, interval(StartTime, EndTime, dt(2.0))),
    jpl_call(TfMemory, 'setTfTableName', ['tf'], _B).

trajectory_duration(Trial, Duration) :-
    get_tf_db(Trial, TfMemory),
    mng_query_latest(Trial, one(DBObjLatest), 'transforms.header.stamp', 'timepoint_1445000000'), %we can set a timepoint which is > 1445000000 since simulation time starts from 0
    mng_query_earliest(Trial, one(DBObjEarliest), 'transforms.header.stamp', 'timepoint_0'),
    jpl_call('org.knowrob.reinforcement.BasicBehavior', 'getTimestamp', [DBObjEarliest, DBObjLatest], Duration),
    jpl_call(TfMemory, 'setTfTableName', ['tf'], _B).

trajectory_length(Trial, Link, Length) :-
    get_tf_db(Trial, _TfMemory),
    mng_query_latest(Trial, one(DBObjLatest), 'transforms.header.stamp', 'timepoint_1445000000'), %we can set a timepoint which is > 1445000000 since simulation time starts from 0
    jpl_call('org.knowrob.reinforcement.BasicBehavior', 'getTimestamp', [DBObjLatest], EndTime),
    mng_query_earliest(Trial, one(DBObjEarliest), 'transforms.header.stamp', 'timepoint_0'),
    jpl_call('org.knowrob.reinforcement.BasicBehavior', 'getTimestamp', [DBObjEarliest], StartTime),
    mng_lookup_position('/base_footprint', Link, StartTime, StartPosition),
    mng_lookup_position('/base_footprint', Link, EndTime, EndPosition),
    nth0(0, StartPosition, StartX),
    nth0(1, StartPosition, StartY),
    nth0(2, StartPosition, StartZ),
    nth0(0, EndPosition, EndX),
    nth0(1, EndPosition, EndY),
    nth0(2, EndPosition, EndZ),
    DiffX is StartX - EndX,
    DiffY is StartY - EndY,
    DiffZ is StartZ - EndZ,
    SDiffX is DiffX * DiffX,
    SDiffY is DiffY * DiffY,
    SDiffZ is DiffZ * DiffZ,
    SDiff1 is SDiffX + SDiffY,
    SDiff is SDiff1 + SDiffZ,
    Length is sqrt(SDiff).      
  
get_tf_db(Trial, TfMemory) :-
    use_simulation_data(Trials),
    member(Trial, Trials),
    jpl_call('org.knowrob.tfmemory.TFMemory', 'getInstance', [], TfMemory),
    jpl_call(TfMemory, 'setTfTableName', [Trial], _A).
