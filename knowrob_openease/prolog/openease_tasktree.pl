/** <module> openease_video

  Copyright (C) 2016 Asil Kaan Bozcuoglu
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

  @author Asil Kaan Bozcuoglu
  @license BSD
*/

:- module(openease_tasktree,
    [
        visualize_task_tree_in_timeline/3
    ]).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('lists')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_cram, 'http://knowrob.org/kb/knowrob_cram.owl#', [keep(true)]).

visualize_task_tree_in_timeline(StartTime, EndTime, Type ) :-
    task_start(T, StartTime),
    task_end(T, EndTime),
    (rdf_has(T, _X, Type); rdf_has(T, _X, literal(type(_, Type)))),
    (rdf_has(T, knowrob:'taskContext', literal(type(_,TaskContext))); TaskContext = ''),
    (rdf_has(T, knowrob:'goalContext', literal(type(_, TaskGoal))); TaskGoal = ''),
    atomic_list_concat([TaskContext, TaskGoal], ' ', TaskTip), term_to_atom(T, TaskIdAtom),
    term_to_atom(TaskTip, TaskAtom), term_to_atom(Type, TypeAtom),
    jpl_new( '[Ljava.lang.String;', [TaskAtom, TaskIdAtom, TaskIdAtom, TypeAtom, StartTime, EndTime], TaskDetail),
    findall(_TaskDetail, (task(_TaskId), subtask(T, _TaskId), task_type(_TaskId, _Type), task_start(_TaskId, _S), task_end(_TaskId, _E),
                         (rdf_has(_TaskId, knowrob:'taskContext', literal(type(_,_TaskContext))); _TaskContext = ''), 
                         (rdf_has(_TaskId, knowrob:'goalContext', literal(type(_,_TaskGoal))); _TaskGoal = ''),
                         atomic_list_concat([_TaskContext, _TaskGoal], ' ', _TaskTip), term_to_atom(_TaskId, _TaskIdAtom), term_to_atom(T, _ParentTaskIdAtom),
                         term_to_atom(_TaskTip, _TaskAtom), term_to_atom(_Type, _TypeAtom),
                         jpl_new( '[Ljava.lang.String;', [_TaskAtom, _TaskIdAtom, _ParentTaskIdAtom, _TypeAtom, _S, _E], _TaskDetail)), _TaskDetails),
    append([TaskDetail], _TaskDetails, _TaskDetailsFinal),
    jpl_new( '[[Ljava.lang.String;', _TaskDetailsFinal, _Tasks),
    findall(_TaskAtom, (task(__TaskId), term_to_atom(__TaskId, _TaskAtom)), HighlightedTaskDetails),
    jpl_new( '[Ljava.lang.String;', HighlightedTaskDetails, _HighlightedTasks),
    jpl_new( '[Ljava.lang.String;', ['\'http://knowrob.org/kb/knowrob.owl#UIMAPerception\'', '\'http://knowrob.org/kb/knowrob.owl#CRAMAction\'', '\'http://knowrob.org/kb/knowrob.owl#HeadMovement\'', '\'http://knowrob.org/kb/knowrob.owl#ArmMovement\'', '\'http://knowrob.org/kb/knowrob.owl#CRAMPerform\'', '\'http://knowrob.org/kb/knowrob.owl#CRAMAchieve\'', '\'http://knowrob.org/kb/knowrob.owl#CRAMMonitor\'', '\'http://knowrob.org/kb/knowrob.owl#CRAMPerceive\'', '\'http://knowrob.org/kb/knowrob.owl#BaseMovement\'', TypeAtom], _Types),
    update_task_tree(_Tasks, _HighlightedTasks, _Types).
