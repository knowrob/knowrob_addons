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
:- module(knowrob_plan_summary,
    [
	generate_pdf_summary/2,
	generate_html_summary/2,
	generate_jpg_summary/2,
      	create_latex_with_semantic_map/1,
	add_robot_poses/2,
	add_plan_trajectory/4	
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('jpl')).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

summary_interface :-
    summary_interface(_).

:- assert(sum_interface(fail)).
summary_interface(PDF) :-
    sum_interface(fail),
    jpl_new('org.knowrob.summary.PDFFactory', [], PDF),
    retract(sum_interface(fail)),
    assert(sum_interface(PDF)),!.

summary_interface(PDF) :-
    sum_interface(PDF).

generate_pdf_summary(GoalsRequireHighlightPose, LatexPath) :-
    generate_summary(GoalsRequireHighlightPose, LatexPath),
    summary_interface(PDF),
    jpl_call(PDF, 'generatePDF', [], _R).

generate_html_summary(GoalsRequireHighlightPose, LatexPath) :-
    generate_summary(GoalsRequireHighlightPose, LatexPath),
    summary_interface(PDF),
    jpl_call(PDF, 'generateHTML', [], _R).

generate_jpg_summary(GoalsRequireHighlightPose, LatexPath) :-
    generate_summary(GoalsRequireHighlightPose, LatexPath),
    summary_interface(PDF),
    jpl_call(PDF, 'generateJPG', [], _R).

generate_summary(GoalsRequireHighlightPose, LatexPath) :-
    create_latex_with_semantic_map(LatexPath),
    add_robot_poses('/base_link', GoalsRequireHighlightPose),
    task_goal(T, 'DEMO'),
    task_start(T,S),
    task_end(T,E),
    add_plan_trajectory('/base_link',S,E,10).

create_latex_with_semantic_map(LatexPath) :-

    findall(
        ObjectDetail,
        (   
	    rdfs_individual_of(W, knowrob:'SummaryFurniture'),
	    rdf_has(W, knowrob:'widthOfObject', Wi),
	    rdf_has(W, knowrob:'heightOfObject', Hi),
	    rdf_has(W, knowrob:'depthOfObject', Di),
	    rdfs_individual_of(S, knowrob:'SemanticMapPerception'),
	    rdf_has(S, knowrob:'objectActedOn', W),
	    rdf_has(S, knowrob:'eventOccursAt', M),
	    rdf_has(M, knowrob:'m13', X),
	    rdf_has(M, knowrob:'m03', Y),
	    rdf_has(M, knowrob:'m01', Or),
	    term_to_atom( W, Na),
            term_to_atom( Wi, Wa),
	    term_to_atom( Hi, Ha),
	    term_to_atom( Di, Da),
	    term_to_atom( X, Xa),
	    term_to_atom( Y, Ya),
	    term_to_atom( Or, Ora),
	    jpl_new( '[Ljava.lang.String;', [Na,Wa,Ha,Da,Xa,Ya,Ora], ObjectDetail)            
        ),
        ObjectDetails
    ),

    jpl_new( '[[Ljava.lang.String;', ObjectDetails, Objects),



    findall(
        WallDetail,
        (   
	    rdfs_individual_of(W, knowrob:'Wall'),
	    rdf_has(W, knowrob:'widthOfObject', Wi),
	    rdfs_individual_of(S, knowrob:'SemanticMapPerception'),
	    rdf_has(S, knowrob:'objectActedOn', W),
	    rdf_has(S, knowrob:'eventOccursAt', M),
	    rdf_has(M, knowrob:'m13', X),
	    rdf_has(M, knowrob:'m03', Y),
	    rdf_has(M, knowrob:'m01', Or),
            term_to_atom( Wi, Wa),
	    term_to_atom( X, Xa),
	    term_to_atom( Y, Ya),
	    term_to_atom( Or, Ora),
	    jpl_new( '[Ljava.lang.String;', [Wa,Xa,Ya,Ora], WallDetail)            
        ),
        WallDetails
    ),
    jpl_new( '[[Ljava.lang.String;', WallDetails, Walls),

    summary_interface(PDF),
    jpl_call(PDF, 'setPathForGeneratedLatex', [LatexPath], @(void)),
    jpl_call(PDF, 'createLatex', [Objects, Walls], _R).

add_robot_poses(Link, GoalsRequireHighlightPose) :-
    findall(
        PositionDetail,
        (   
	    member(Goal, GoalsRequireHighlightPose),
	    task_goal(T, Goal),
	    task_start(T,S), 
	    belief_at(robot(Link,Loc), S),
	    nth0(7, Loc, X),
	    nth0(3, Loc, Y),
	    %rdf_has(Loc, knowrob:'m13', X),
	    %rdf_has(Loc, knowrob:'m03', Y),
	    term_to_atom( X, Xa),
	    term_to_atom( Y, Ya),
	    jpl_new( '[Ljava.lang.String;', [Xa,Ya], PositionDetail)	            
        ),
        PositionDetails
    ),

    jpl_new( '[[Ljava.lang.String;', PositionDetails, Positions),
    summary_interface(PDF),
    jpl_call(PDF, 'locateRobotPositions', [Positions], _R).

add_plan_trajectory(Link,S,E,Interval) :-
    summary_interface(PDF),
    jpl_call(PDF, 'addPlanTrajectory', [Link, S, E, Interval], _R).

