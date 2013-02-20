

rdf_has('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder_LxRYFduN', rdf:type, OBJECTCLASS),
          rdf_reachable(OBJECTCLASS, rdfs:subClassOf, knowrob:'Cylinder'),
          rdf_triple(knowrob:longitudinalDirection, C, Dir),
          rdf_has(Dir, knowrob:vectorX, literal(type(xsd:'float',LongSideX))),
          rdf_has(Dir, knowrob:vectorY, literal(type(xsd:'float',DirY))),
          rdf_has(Dir, knowrob:vectorZ, literal(type(xsd:'float',LongSideZ)))





register_ros_package(mod_vis).

owl_parse('spatula-test.owl', false, false, true), register_ros_package(ias_semantic_map), visualisation_canvas(_).

comp_physical_parts(knowrob:'Spatula_OdhJed', Parts).

current_object_pose(knowrob:'Spatula_OdhJed', SpPose).

add_object(knowrob:'Spatula_OdhJed',_).

annotation_plane_normal(PartInst, NormalVec), 
knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J), 
annotation_pose_list(J, Pose), 
pose_into_global_coord($SpPose, Pose, PoseGl), 
PoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_], 
create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', [1,0,0,X,0,1,0,Y,0,0,1,Z,0,0,0,1],  ['VisualPerception'], Cyl),  
rdf_assert(Cyl, knowrob:longitudinalDirection, NormalVec), 
add_object(Cyl, _), highlight_object(Cyl, _).




% create side vector for plane annotation
% TODO: take orientation of spatula into account -> rotate vectors
current_object_pose(knowrob:'Spatula_OdhJed', SpPose), 
annotation_plane_shortside(PartInst, ShortSide), 
annotation_plane_longside(PartInst, LongSide), 
knowrob_mesh_reasoning:mesh_annotation_java_obj(PartInst, J), 

annotation_pose_list(J, Pose), pose_into_global_coord(SpPose, Pose, PoseGl), PoseGl=[_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_], knowrob_coordinates:list_to_vector3d([X,Y,Z], PlaneCenter), rdf_has(LongSide, knowrob:vectorX, literal(type(xsd:'float',LongSideX))), rdf_has(LongSide, knowrob:vectorY, literal(type(xsd:'float',LongSideY))), rdf_has(LongSide, knowrob:vectorZ, literal(type(xsd:'float',LongSideZ))), knowrob_coordinates:list_to_vector3d([LongSideX,LongSideY,LongSideZ], DirVec), jpl_new('javax.vecmath.Vector3d', [DirVec], SideCenterVec), jpl_call(SideCenterVec, scaleAdd, [0.5, PlaneCenter], _), knowrob_coordinates:vector3d_to_list(SideCenterVec, SideCenter), SideCenter=[PX,PY,PZ], create_object_perception('http://ias.cs.tum.edu/kb/knowrob.owl#Cylinder', [1,0,0,PX,0,1,0,PY,0,0,1,PZ,0,0,0,1],  ['VisualPerception'], Cyl), rdf_assert(Cyl, knowrob:longitudinalDirection, ShortSide), add_object(Cyl, _).



