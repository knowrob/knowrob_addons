/** <module> knowrob_paramserver

  Copyright (C) 2017 Mihai Pomarlan

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

  @author Mihai Pomarlan
  @license BSD
*/

:- module(knowrob_paramserver,
    [
      resolve_frame_specification/5,
      get_associated_transform/7,
      get_annotator_confidence_threshold/3,
      get_object_type_shape_data/3,
      get_object_shape_data/3,
      get_controller_nwsr/2,
      get_controller_specification/2,
      get_controller_motion_old_threshold/3,
      get_controller_minimum_convergence_time/3,
      get_controller_server_timeout/3,
      get_controller_update_period/3,
      get_controller_watchdog_period/3,
      get_controller_still_moving_threshold/4,
      get_controller_joint_group/4,
      get_controller_arm_joints/6,
      get_controller_base_frame/3,
      get_grasp_specification/6,
      get_object_component_placement/3,
      get_annotated_waypoint/3
    ]).

:- use_module(library('jpl')).
:- use_module(library('lists')).
:- use_module(library('util')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(paramserver, 'http://knowrob.org/kb/knowrob_paramserver.owl#',  [keep(true)]).


%% get_annotator_confidence_threshold(+AnnotatorType, +ObjectType, -ConfidenceThreshold) is det.
%
% Find the confidence threshold to use for the (ObjectType, AnnotatorType) pair.
% Returns a real number representing the confidence threshold.
%
% @param AnnotatorType   anyURI representation of the annotator type name (see the RS ontology)
% @param ObjectType      anyURI representation of the object type name (see an object/parts ontology)
% @param ConfidenceThreshold    real number, the confidence threshold to use for this Annotator when applied to Object
%
get_annotator_confidence_threshold(AnnotatorType, ObjectType, ConfidenceThreshold) :-

  % find an instance of AnnotatorConfidenceThreshold ...
  rdfs_individual_of(CTCandidate, paramserver:'AnnotatorConfidenceThreshold'),

  % ... validForAnnotatorType AnnotatorType ...
  rdf_has(CTCandidate, paramserver:'validForAnnotatorType', literal(type(xsd:'anyURI', AnnotatorType))),

  % ... validForObjectType ObjectType ...
  rdf_has(CTCandidate, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectType))),

  % ... and set ConfidenceThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, ConfidenceThreshold).

%% get_object_type_shape_data(+ObjectType, +Annotator, -FilePath) is det.
%
% Find a path to a shape file for an Object/Annotator pair.
% Useful when querying for an object type.
% Returns a file path.
%
% @param ObjectType      anyURI representation of the object type name (see an object/parts ontology)
% @param Annotator       annotator individual (see the RS ontology)
% @param FilePath        path to the shape data file
%
get_object_type_shape_data(ObjectType, Annotator, FilePath) :-

  % find the required shape data type for this Annotator ...
  rdf_has(Annotator, paramserver:'needsShapeType', literal(type(xsd:'anyURI', ReferenceType))),

  % ... find an instance of that type ...
  rdfs_individual_of(RCandidate, ReferenceType),

  % ... validForObjectType ObjectType ...
  rdf_has(RCandidate, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectType))),

  % ... and set FilePath to its value.
  rdf_has(RCandidate, paramserver:'hasPath', literal(type(xsd:'anyURI', FP))),
  atom_string(FP, FilePath).

%% get_object_shape_data(+Object, +Annotator, -FilePath) is det.
%
% Find a path to a shape file for an Object/Annotator pair.
% Useful when querying for an object type.
% Returns a file path.
%
% @param Object          object individual (see an object/parts ontology)
% @param Annotator       annotator individual (see the RS ontology)
% @param FilePath        path to the shape data file
%
get_object_shape_data(Object, Annotator, FilePath) :-

  % find the required shape data type for this Annotator ...
  rdf_has(Annotator, paramserver:'needsShapeType', literal(type(xsd:'anyURI', ReferenceType))),

  % ... find an instance of that type ...
  rdfs_individual_of(RCandidate, ReferenceType),

  % ... inverse hasShape Object ...
  rdf_has(Object, paramserver:'hasShape', RCandidate),

  % ... and set FilePath to its value.
  rdf_has(RCandidate, paramserver:'hasPath', literal(type(xsd:'anyURI', FP))),
  atom_string(FP, FilePath).

%% get_controller_nwsr(+ControllerType, -NWSR) is det.
%
% Find the nWSR to use for a particular giskard controller.
% Returns a real number representing the number of repetitions for the optimizer.
%
% @param ControllerType   anyURI: some string to identify the controller
% @param NWSR             real number, the required nWSR
%
get_controller_nwsr(ControllerType, NWSR) :-

  % find an instance of NWSR ...
  rdfs_individual_of(NWSRCandidate, paramserver:'NWSR'),

  % ... validForControllerType ControllerType ...
  rdf_has(NWSRCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... and set NWSR to its value.
  rdf_has(NWSRCandidate, paramserver:'hasValue', literal(type(_, NAt))),
  atom_number(NAt, NWSR).

%% get_controller_specification(+ControllerType, -FilePath) is det.
%
% Find a path to a YAML file for a Controller.
% Returns that file path.
%
% @param ControllerType  anyURI some string to identify a controller
% @param FilePath        path to the YAML file
%
get_controller_specification(ControllerType, FilePath) :-

  % find an instance of ControllerSpecification ...
  rdfs_individual_of(RCandidate, paramserver:'ControllerSpecification'),

  % ... validForControllerType ControllerType ...
  rdf_has(RCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... and set FilePath to its value.
  rdf_has(RCandidate, paramserver:'hasPath', literal(type(xsd:'anyURI', FP))),
  atom_string(FP, FilePath).

%% get_controller_motion_old_threshold(+ControllerType, -MotionOldThreshold, -Unit) is det.
%
% Find the motion old threshold to use for the ControllerType.
% Returns a real number representing a time after which the motion is thought old, and a string representing the unit.
%
% @param ControllerType     anyURI some string to identify the controller type
% @param MotionOldThreshold real number, the required duration
% @param Unit               anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_motion_old_threshold(ControllerType, MotionOldThreshold, Unit) :-

  % find an instance of ControllerMotionOld ...
  rdfs_individual_of(CTCandidate, paramserver:'ControllerMotionOld'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, MotionOldThreshold).

%% get_controller_minimum_convergence_time(+ControllerType, -MinimumConvergenceTime, -Unit) is det.
%
% Find the minimum convergence time expected for ControllerType.
% Returns a real number representing a time before which no convergence is tested, and a string representing the unit.
%
% @param ControllerType         anyURI some string to identify the controller type
% @param MinimumConvergenceTime real number, the required duration
% @param Unit                   anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_minimum_convergence_time(ControllerType, MinimumConvergenceTime, Unit) :-

  % find an instance of ControllerMotionOld ...
  rdfs_individual_of(CTCandidate, paramserver:'MinimumConvergenceTime'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, MinimumConvergenceTime).

%% get_controller_server_timeout(+ControllerType, -ServerTimeout, -Unit) is det.
%
% Find the actionserver timeout for ControllerType.
% Returns a real number representing a time after which the actionserver is declared dead, and a string representing the unit.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param ServerTimeout    real number, the required duration
% @param Unit             anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_server_timeout(ControllerType, ServerTimeout, Unit) :-

  % find an instance of ServerTimeout ...
  rdfs_individual_of(CTCandidate, paramserver:'ServerTimeout'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, ServerTimeout).

%% get_controller_update_period(+ControllerType, -UpdatePeriod, -Unit) is det.
%
% Find the controller update period for ControllerType.
% Returns a real number representing a time after which the actionserver is declared dead, and a string representing the unit.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param UpdatePeriod     real number, the required duration
% @param Unit             anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_update_period(ControllerType, UpdatePeriod, Unit) :-

  % find an instance of UpdatePeriod ...
  rdfs_individual_of(CTCandidate, paramserver:'UpdatePeriod'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, UpdatePeriod).

%% get_controller_watchdog_period(+ControllerType, -WatchdogPeriod, -Unit) is det.
%
% Find the controller watchdog period for ControllerType.
% Returns a real number representing a time after which the controller is reset, and a string representing the unit.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param WatchdogPeriod   real number, the required duration
% @param Unit             anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_watchdog_period(ControllerType, WatchdogPeriod, Unit) :-

  % find an instance of WatchdogPeriod ...
  rdfs_individual_of(CTCandidate, paramserver:'WatchdogPeriod'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, WatchdogPeriod).

%% get_controller_still_moving_threshold(+ControllerType, +GroupName, -StillMoving, -Unit) is det.
%
% Find the still-moving threshold for ControllerType and Joint Group.
% Returns a real number representing a velocity below which joints in the GroupName jointgroup are believed to have stopped.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param GroupName        string some string to identify the joint group
% @param StillMoving      real number, the required velocity
% @param Unit             anyURI representation of the unit name (see the paramserver ontology)
%
get_controller_still_moving_threshold(ControllerType, GroupName, StillMoving, Unit) :-

  % find an instance of ControllerStillMovingThreshold ...
  rdfs_individual_of(CTCandidate, paramserver:'ControllerStillMovingThreshold'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... validForJointGroup some joint group ...
  rdf_has(CTCandidate, paramserver:'validForJointGroup', JointGroup),

  % ... that has GroupName as its name ...
  rdf_has(JointGroup, paramserver:'hasName', literal(type(xsd:'string', GroupName))), 

  % ... set Unit to its unit ...
  rdf_has(CTCandidate, paramserver:'hasUnit', CUn),
  atom_string(CUn, Unit),

  % ... and set MotionOldThreshold to its value.
  rdf_has(CTCandidate, paramserver:'hasValue', literal(type(_, CAt))),
  atom_number(CAt, StillMoving).

get_jointgroup_joints(JointListElement, Ini, Joints) :-
  \+ rdf_has(JointListElement, paramserver:'hasSuccessor', _),
  rdf_has(JointListElement, paramserver:'hasElement', El),
  rdf_has(El, paramserver:'hasValue', literal(type(xsd:'string', J))),
  atom_string(J, Joint),
  append(Ini, [Joint], Joints).

get_jointgroup_joints(JointListElement, Ini, Joints) :-
  rdf_has(JointListElement, paramserver:'hasSuccessor', Successor),
  rdf_has(JointListElement, paramserver:'hasElement', El),
  rdf_has(El, paramserver:'hasValue', literal(type(xsd:'string', J))),
  atom_string(J, Joint),
  append(Ini, [Joint], NewIni),
  get_jointgroup_joints(Successor, NewIni, Joints).

get_onto_side(S, OS) :-
  downcase_atom(S, Sd),
  =(Sd, 'left'),
  =(OS, 'http://knowrob.org/kb/knowrob_paramserver.owl#Left').

get_onto_side(S, OS) :-
  downcase_atom(S, Sd),
  =(Sd, 'right'),
  =(OS, 'http://knowrob.org/kb/knowrob_paramserver.owl#Right').

get_onto_side(S, OS) :-
  rdfs_individual_of(S, paramserver:'Side'),
  =(S, OS).

%% get_controller_joint_group(+ControllerType, +RobotType, +GroupName, -Joints) is det.
%
% Find the joints belonging to a joint group GroupName, valid for a Controller- and RobotType.
% Returns a list of strings with the joint names.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param RobotType        anyURI some string to identify the robot type
% @param GroupName        string some string to identify the joint group
% @param Joints           string list of joint names
%
get_controller_joint_group(ControllerType, RobotType, GroupName, Joints) :-

  % find an instance of JointGroup ...
  rdfs_individual_of(CTCandidate, paramserver:'JointGroup'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... validForRobotType RobotType ...
  rdf_has(CTCandidate, paramserver:'validForRobotType', literal(type(xsd:'anyURI', RobotType))),

  % ... that has GroupName as its name ...
  rdf_has(CTCandidate, paramserver:'hasName', literal(type(xsd:'string', GroupName))),

  % ... then find its list of joints ...
  rdf_has(CTCandidate, paramserver:'hasComponent', JointListElement),

  % ... and return all component joints it has
  get_jointgroup_joints(JointListElement, [], RevJoints),
  reverse(RevJoints, Joints).

%% get_controller_arm_joints(+ControllerType, +RobotType, +Side, -Joints, -Name, -EEFJointName) is det.
%
% Find the joints belonging to the arm for Side, valid for a Controller- and RobotType.
% Returns a list of strings with the joint names, the name of the group, and the FK or EEF joint name.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param RobotType        anyURI some string to identify the robot type
% @param Side             anyURI some string to identify the side; either a name of a paramserver Side individual or 'left' or 'right' 
% @param Joints           string list of joint names
% @param Name             string name of the joint group
% @param EEFJointName     string name of the EEF joint, aka the FK joint for the group
%
get_controller_arm_joints(ControllerType, RobotType, Side, Joints, Name, EEFJointName) :-

  % find an instance of ArmJoints ...
  rdfs_individual_of(CTCandidate, paramserver:'ArmJoints'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... validForRobotType RobotType ...
  rdf_has(CTCandidate, paramserver:'validForRobotType', literal(type(xsd:'anyURI', RobotType))),

  % ... which has an EEFJoint ...
  rdf_has(CTCandidate, paramserver:'hasComponent', EEFJointListElement),
  rdf_has(EEFJointListElement, paramserver:'hasElement', EEFJoint),
  rdfs_individual_of(EEFJoint, paramserver:'EEFJointName'),

  % ... that is validForSide Side ...
  get_onto_side(Side, OntoSide),
  rdf_has(EEFJoint, paramserver:'validForSide', OntoSide),

  % ... then retrieve the group name and EEF joint name
  rdf_has(CTCandidate, paramserver:'hasName', literal(type(xsd:'string', GN))),
  atom_string(GN, Name),
  rdf_has(EEFJoint, paramserver:'hasValue', literal(type(xsd:'string', EJ))),
  atom_string(EJ, EEFJointName),

  % ... and return all component joints the group has
  get_jointgroup_joints(EEFJointListElement, [], RevJoints),
  reverse(RevJoints, Joints).

resolve_frame_specification(_, _, _, FrameSpec, FrameName) :-
  owl_same_as(FrameSpec, 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol'),
%% TODO: in the future, we may want to indirect the MapFrameSymbol as well, maybe on environment
%% However for now the name of the map frame will be stored directly in the map symbol.
  rdf_has(FrameSpec, paramserver:'hasValue', literal(type(xsd:'string', FN))),
  atom_string(FN, FrameName).

resolve_frame_specification(_, _, _, FrameSpec, FrameName) :-
  rdfs_individual_of(FrameSpec, paramserver:'CoordinateFrameName'),
  rdf_has(FrameSpec, paramserver:'hasValue', literal(type(xsd:'string', FN))),
  atom_string(FN, FrameName).

resolve_frame_specification(GripperType, _, _, FrameSpec, FrameName) :-
  atomic(GripperType),
  rdfs_individual_of(FrameSpec, paramserver:'CoordinateFrameSymbol'),
  rdf_has(FrameSpec, paramserver:'standsFor', FNI),
  rdf_has(FNI, paramserver:'validForGripperType', literal(type(xsd:'anyURI', GripperType))),
  rdf_has(FNI, paramserver:'hasValue', literal(type(xsd:'string', FN))),
  atom_string(FN, FrameName).

resolve_frame_specification(_, ObjectType, _, FrameSpec, FrameName) :-
  atomic(ObjectType),
  rdfs_individual_of(FrameSpec, paramserver:'CoordinateFrameSymbol'),
  rdf_has(FrameSpec, paramserver:'standsFor', FNI),
  rdf_has(FNI, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectType))),
  rdf_has(FNI, paramserver:'hasValue', literal(type(xsd:'string', FN))),
  atom_string(FN, FrameName).

resolve_frame_specification(_, _, RobotType, FrameSpec, FrameName) :-
  atomic(RobotType),
  rdfs_individual_of(FrameSpec, paramserver:'CoordinateFrameSymbol'),
  rdf_has(FrameSpec, paramserver:'standsFor', FNI),
  rdf_has(FNI, paramserver:'validForRobotType', literal(type(xsd:'anyURI', RobotType))),
  rdf_has(FNI, paramserver:'hasValue', literal(type(xsd:'string', FN))),
  atom_string(FN, FrameName).

%% get_controller_base_frame(+ControllerType, +RobotType, -BaseFrameName) is det.
%
% Find the base frame for ControllerType. In the knowledge base this can be a simple frame name
% which will be returned as-is or a frame symbol which will be dereferenced based on RobotType.
% Returns the base frame name.
%
% @param ControllerType   anyURI some string to identify the controller type
% @param RobotType        anyURI some string to identify the robot type
% @param BaseFrameName    string name of the controller base frame
%
get_controller_base_frame(ControllerType, RobotType, BaseFrameName) :-

  % First, try to find a CoordinateFrameName ...
  rdfs_individual_of(CTCandidate, paramserver:'CoordinateFrameSpecification'),

  % ... validForControllerType ControllerType ...
  rdf_has(CTCandidate, paramserver:'validForControllerType', literal(type(xsd:'anyURI', ControllerType))),

  % ... and retrieve its name
  resolve_frame_specification(_, _, RobotType, CTCandidate, BaseFrameName).

ensure_number(AorN, N) :-
  number(AorN),
  =(AorN, N).

ensure_number(AorN, N) :-
  atom(AorN),
  atom_number(AorN, N).

get_associated_pose(GripperType, ObjectType, RobotType, ParSpec, TrIn, Relation, Pose) :-
  % find a transform individual based on Relation to ParSpec
  rdf_has(ParSpec, Relation, TrIn),
  % resolve reference frame
  rdf_has(TrIn, paramserver:'hasReferenceFrame', RFS),
  resolve_frame_specification(GripperType, ObjectType, RobotType, RFS, RefFrame),!,
  % obtain translation and rotation components
  rdf_has(TrIn, 'http://knowrob.org/kb/knowrob.owl#translation', literal(type(_, Tran))),
  knowrob_math:parse_vector(Tran, TransformationVector),
  rdf_has(TrIn, 'http://knowrob.org/kb/knowrob.owl#quaternion', literal(type(_, Rot))),
  knowrob_math:parse_vector(Rot, RotationVector),
  % assemble return value
  =(Pose, [RefFrame, TransformationVector, RotationVector]),!.

get_associated_transform(GripperType, ObjectType, RobotType, ParSpec, TrIn, Relation, Transform) :-
  % find a transform individual based on Relation to ParSpec
  rdf_has(ParSpec, Relation, TrIn),
  % extract the Pose related properties
  get_associated_pose(GripperType, ObjectType, RobotType, ParSpec, TrIn, Relation, Pose),!,
  nth0(0, Pose, RefFrame),
  nth0(1, Pose, Translation),
  nth0(2, Pose, Rotation),
  % resolve target frame
  rdf_has(TrIn, paramserver:'hasTargetFrame', TFS),
  resolve_frame_specification(GripperType, ObjectType, RobotType, TFS, TgFrame),!,
  % assemble return value
  =(Transform, [RefFrame, TgFrame, Translation, Rotation]),!.

get_transforms(GripperType, ObjectType, RobotType, ListEl, Ini, Result) :-
  \+ rdf_has(ListEl, paramserver:'hasSuccessor', _),
  get_associated_transform(GripperType, ObjectType, RobotType, ListEl, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasElement', Tr),
  append(Ini, [Tr], Result).

get_transforms(GripperType, ObjectType, RobotType, ListEl, Ini, Result) :-
  rdf_has(ListEl, paramserver:'hasSuccessor', Successor),
  get_associated_transform(GripperType, ObjectType, RobotType, ListEl, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasElement', Tr),
  append(Ini, [Tr], NewIni),
  get_transforms(GripperType, ObjectType, RobotType, Successor, NewIni, Result).

get_grasp_transform_list(_, _, _, GraspSpec, Relation, Result) :-
  \+ rdf_has(GraspSpec, Relation, _),
  =([], Result).

get_grasp_transform_list(GripperType, ObjectType, RobotType, GraspSpec, Relation, Result) :-
  rdf_has(GraspSpec, Relation, ListEl),
  get_transforms(GripperType, ObjectType, RobotType, ListEl, [], Result).

%% get_grasp_specification(+GripperType, +ObjectType, +RobotType, -GraspTransform, -PregraspTransforms, -PostgraspTransforms) is det.
%
% Find a grasp specification for the GripperType/ObjectType pair, and return the grasp transform and a list
% of pre- and postgrasp transforms. A transform is returned as a list:
% [string ref_frame_name, string target_frame_name, [float tran_x, float tran_y, float tran_z], [float rot_x, float rot_y, float rot_z, float rot_w]].
% RobotType is used to resolve CoordinateFrameSymbols, if needed.
% Translations are returned in meters.
%
% @param GripperType        anyURI some string to identify the gripper type
% @param ObjectType         anyURI some string to identify the object type
% @param RobotType          anyURI some string to identify the robot type
% @param GraspTransform     list of items representing a transform for grasping
% @param PreGraspTransforms list of list of items representing transforms preceding a grasp
% @param PostGraspTransform list of list of items representing transforms done after grasping
%
get_grasp_specification(GripperType, ObjectType, RobotType, GraspTransform, PregraspTransforms, PostgraspTransforms) :-

  % Find a grasp specification ...
  rdfs_individual_of(CTCandidate, paramserver:'GraspSpecification'),

  % ... validForGripperType GripperType ...
  rdf_has(CTCandidate, paramserver:'validForGripperType', literal(type(xsd:'anyURI', GripperType))),

  % ... validForObjectType ObjectType ...
  rdf_has(CTCandidate, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectType))),

  % ... retrieve pre- and postgrasps ...
  get_grasp_transform_list(GripperType, ObjectType, RobotType, CTCandidate, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasPregraspTransform', PregraspTransforms),
  get_grasp_transform_list(GripperType, ObjectType, RobotType, CTCandidate, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasPostgraspTransform', PostgraspTransforms),

  % ... and retrieve its grasp transform
  get_associated_transform(GripperType, ObjectType, RobotType, CTCandidate, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasGraspTransform', GraspTransform).

%% get_object_component_placement(+ObjectType, +ComponentType, -Pose)
%
% Find a pose for the ComponentType/ObjectType pair, and return it.
% A pose is returned as a list:
% [string ref_frame_name, [float tran_x, float tran_y, float tran_z], [float rot_x, float rot_y, float rot_z, float rot_w]].
% ObjectType is used to resolve CoordinateFrameSymbols, if needed.
% Translations are returned in meters.
%
% @param ObjectType         anyURI some string to identify the object type
% @param ComponentType      anyURI some string to identify the component type
% @param Pose               list of items representing a pose of the component relative to the object
%
get_object_component_placement(ObjectType, ComponentType, Pose) :-
  
  % Find an ObjectComponentPlacement ...
  rdfs_individual_of(CTCandidate, paramserver:'ObjectComponentPlacement'),

  % ... validForObjectType ObjectType ...
  rdf_has(CTCandidate, paramserver:'validForObjectType', literal(type(xsd:'anyURI', ObjectType))),

  % ... validForObjectComponentType ComponentType ...
  rdf_has(CTCandidate, paramserver:'validForObjectComponentType', literal(type(xsd:'anyURI', ComponentType))),

  % ... and retrieve the pose
  get_associated_pose(_, ObjectType, _, CTCandidate, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Pose).

%% get_annotated_waypoint(+ManeuverType, +RobotType, -Transform)
%
% Find a transform for the ManeuverType/RobotType pair, and return it.
% A transform is returned as a list:
% [string ref_frame_name, string target_frame, [float tran_x, float tran_y, float tran_z], [float rot_x, float rot_y, float rot_z, float rot_w]].
% RobotType is used to resolve CoordinateFrameSymbols, if needed.
% Translations are returned in meters.
%
% @param ManeuverType   anyURI some string to identify the maneuver type
% @param RobotType      anyURI some string to identify the robot type
% @param Transform      list of items representing a transform describing the waypoint
%
get_annotated_waypoint(ManeuverType, RobotType, Transform) :-
  
  % Find an AnnotatedWaypoint ...
  rdfs_individual_of(CTCandidate, paramserver:'AnnotatedWaypoint'),

  % ... validForManeuverType ManeuverType ...
  rdf_has(CTCandidate, paramserver:'validForManeuverType', literal(type(xsd:'anyURI', ManeuverType))),

  % ... validForRobotType RobotType ...
  rdf_has(CTCandidate, paramserver:'validForRobotType', literal(type(xsd:'anyURI', RobotType))),

  % ... and retrieve the transform
  get_associated_transform(_, _, RobotType, CTCandidate, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform).

