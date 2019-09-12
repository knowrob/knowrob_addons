/** <module> openease

  Copyright (C) 2019 Daniel Beßler
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

  @author Daniel Beßler
  @license BSD
*/

:- module(openease,
    [
      camera_pose/2,
        highlight/1,
        highlight/2,
        unhighlight/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('lists')).
:- use_module(library('knowrob/marker_vis')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).

%% camera_pose(+Position:list, +Orientation:list) is det
%
% Sends a pose via the ROS topic _|/camera/pose|_.
% Visualization clients may choose to manipulate some 3D camera accordingly.
%
% @param Position [float x,y,z]
% @param Orientation [float qx,qy,qz,qw]
%
camera_pose([X,Y,Z], [QX,QY,QZ,QW]) :-
    ros_publish('/camera/pose', 'geometry_msgs/Pose', _{
      position:    _{x: X, y: Y, z: Z},
      orientation: _{x: QX, y: QY, z: QZ, w: QW}
    }).

openease_highlight_msg(Objects,[R,G,B],Msg) :-
  openease_highlight_msg(Objects,[R,G,B,1],Msg), !.
openease_highlight_msg(Objects,[R,G,B,A],_{
    objects: ['array(string)',Objects],
    color: ['std_msgs/ColorRGBA',_{r:[float32,R], 
                                   g:[float32,G],
                                   b:[float32,B],
                                   a:[float32,A]}]
  }).

highlight(Object) :-
  highlight(Object,[1,0,0,1]).

highlight(Object,Color) :-
  atom(Object),!,
  highlight([Object],Color).

highlight(Objects,Color) :-
  is_list(Objects),!,
  findall(X,(
    member(O,Objects),
    % FIXME: this won't work with the object state publisher
    once(((
      marker_term(O,Term),
      marker_name(Term,X));
      X=O))
  ), Names),
  openease_highlight_msg(Names,Color,Msg),
  ros_publish('/openease/highlight', 'knowrob_openease/Highlight', Msg).

unhighlight(Obj) :-
  highlight(Obj,[0,0,0,0]).
