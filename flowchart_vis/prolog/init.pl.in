
:- register_ros_package(mod_vis).
:- register_ros_package(ias_semantic_map).
:- register_ros_package(knowrob_mesh_reasoning).
:- register_ros_package(knowrob_cad_models).
:- register_ros_package(knowrob_motion_constraints).
:- register_ros_package(comp_ehow).
:- register_ros_package(rosprac).

% test objects for CAD segmentation
:- owl_parser:owl_parse('../owl/test-objects.owl', false, false, true).

% pancake instructions:
%:- owl_parser:owl_parse('@COMP_EHOW_PACKAGE_PATH@/make_pancakes.owl', false, false, true).
:- rdf_db:rdf_register_ns(ehow, 'http://ias.cs.tum.edu/kb/ehow_input.owl#', [keep(true)]).


% motion constraint specs:
:- owl_parser:owl_parse('@KNOWROB_MOTION_CONSTRAINTS_PACKAGE_PATH@/owl/pancake-making-constr.owl', false, false, true).

:- rdf_db:rdf_register_ns(constr, 'http://ias.cs.tum.edu/kb/motion-constraints.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake_constr, 'http://ias.cs.tum.edu/kb/pancake-making-constr.owl#', [keep(true)]).

% spatula properties
:- owl_parser:owl_parse('@KNOWROB_MOTION_CONSTRAINTS_PACKAGE_PATH@/owl/spatula-features.owl', false, false, true).
:- rdf_db:rdf_register_ns(spatula, 'http://ias.cs.tum.edu/kb/spatula-features.owl#', [keep(true)]).


