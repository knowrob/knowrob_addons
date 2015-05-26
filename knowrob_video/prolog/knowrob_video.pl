/** 

  Copyright (C) 2015 Asil Kaan Bozcuoglu
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

:- module(knowrob_video,
    [
	show_speech_between_interval/5,
	show_hud_text_between_interval/4,
        show_speech_after_time/4,
	show_hud_text_after_time/3,
        show_speech_before_time/4,
	show_hud_text_before_time/3,
        show_speech_at_time/5,
      	show_hud_text_at_time/4,
        is_time_inside_interval_duration/3,
        get_goals_at_time/2,
        get_contexts_at_time/2,
        get_perception_at_time/2,
        get_perception_at_time/3,
        get_active_designators_at_time/4,
        set_experiment_name/1,
        check_experiment_videos_exists/2	
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

video_interface :-
    summary_interface(_).

:- assert(vid_interface(fail)).
video_interface(V) :-
    vid_interface(fail),
    jpl_new('org.knowrob.video.VideoFactory', [], V),
    retract(vid_interface(fail)),
    assert(vid_interface(V)),!.

video_interface(V) :-
    vid_interface(V).

show_speech_between_interval(Text, Position, T_Start, T_End, Current_Time) :-
   time_between(Current_Time, T_Start, T_End),
   add_speech_bubble(Text, Position).

show_hud_text_between_interval(Text, T_Start, T_End, Current_Time) :-
   time_between(Current_Time, T_Start, T_End),
   add_text(Text).

show_speech_after_time(Text, Position, T_After, Current_Time) :-
   time_earlier_then(T_After, Current_Time),
   add_speech_bubble(Text, Position).

show_hud_text_after_time(Text, T_After, Current_Time) :-
   time_earlier_then(T_After, Current_Time),
   add_text(Text).

show_speech_before_time(Text, Position, T_Before, Current_Time) :-
   time_earlier_then(Current_Time, T_Before),
   add_speech_bubble(Text, Position).

show_hud_text_before_time(Text, T_Before, Current_Time) :-
   time_earlier_then(Current_Time, T_Before),
   add_text(Text).

show_speech_at_time(Text, Position, Time, Duration, Current_Time) :-
   is_time_inside_interval_duration(Time, Duration, Current_Time), 
   add_speech_bubble(Text, Position).

show_hud_text_at_time(Text, Time, Duration, Current_Time) :-
   is_time_inside_interval_duration(Time, Duration, Current_Time),
   add_text(Text).

is_time_inside_interval_duration(Time, Duration, Current_Time) :-
   time_earlier_then(Time, Current_Time),
   time_term(Time, Time_Term),
   T_Limit_Term is Time_Term + Duration,
   time_term(T_Limit, T_Limit_Term),
   time_earlier_then(Current_Time, T_Limit).

get_goals_at_time(Time, Goals) :-
   findall(Goal,
           (task(Task, Time),
           rdf_has(Task, knowrob:'goalContext', literal(type(_,Goal)))), Goals).

get_contexts_at_time(Time, Contexts) :-
   findall(Goal,
           (task(Task, Time),
           rdf_has(Task, knowrob:'taskContext', literal(type(_,Goal)))), Contexts).

get_perception_at_time(Time, Perception) :-
   task(Task, Time),
   rdf_has(Task, knowrob:'perceptionResult', PercDesig),
   mng_designator_props(PercDesig, 'TYPE', Perception).

get_perception_at_time(Time, Props, Perception) :-
   task(Task, Time),
   rdf_has(Task, knowrob:'perceptionResult', PercDesig),
   mng_designator_props(PercDesig, Props, Perception).

get_active_designators_at_time(Time, DesignatorFields, Props, Results) :-
   findall(Result, 
           (task(Task, Time),
           member(DesignatorField, DesignatorFields),
           rdf_has(Task, DesignatorField, Desig),
           mng_designator_props(Desig, Props, Result)), Results).

set_experiment_name(VideoPath) :-
    video_interface(V),
    term_to_atom( VideoPath, VideoAtom),
    jpl_call(V, 'setPathOfVideo', [VideoAtom], @(void)).

check_experiment_videos_exists(ExpName, ListOfAddress) :-
    video_interface(V),
    term_to_atom( ExpName, ExpAtom),
    jpl_call(V, 'giveAddressOfVideos', [ExpAtom], AddressJava),
    jpl_array_to_list(AddressJava, ListOfAddress).
