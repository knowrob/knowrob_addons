%%
%% Copyright (C) 2010 by Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(comp_spatial).
:- register_ros_package(mod_vis).
:- register_ros_package(comp_missingobj).


% :- use_module(library('comp_missingobj')).
:- use_module(library('comp_missingobj_offline')).

:- owl_parser:owl_parse('../owl/comp_missingobj.owl',  false, false, true).
:- owl_parser:owl_parse('../owl/comp_missingobj_offline.owl',  false, false, true).

