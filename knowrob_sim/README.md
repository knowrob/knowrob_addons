###################################
# EXAMPLE CALLS FOR KNOWROB/WEBROB 
###################################

NOTE: calls to add_trajectory_sim only work when visualization (web server) is running. The other calls also work in the prolog shell alone

- To start a shell: 
>rosrun rosprolog rosprolog knowrob_roslog_launch

- To start a local server: 
>roslaunch knowrob_roslog_launch knowrob.launch 

Make sure a mongo docker container with the right tf data is running


- Jan pick-and-place experiment file 
>load_experiment('/home/yfang/hydro_workspace/sandbox/catkin_ws/src/knowrob_data/logs/robots/cram/pick-and-place/cram_log.owl').

- Have a look which timepoints exist 
>owl_individual_of(A, 'http://knowrob.org/kb/knowrob.owl#TimePoint').

- Visualize a trajectory by manually inputting start and end times 
add_trajectory_sim(linkname, start, end, timesteps, markertype, color) 
>add_trajectory_sim('hit_hand', 'http://ias.cs.tum.edu/kb/cram_log.owl#timepoint_693', 'http://ias.cs.tum.edu/kb/cram_log.owl#timepoint_4929', 100, 0,0.5).

- Load our sim experiment example owl file 
>load_experiment('/home/yfang/hydro_workspace/sandbox/catkin_ws/src/knowrob_sim/example_files/test2.owl').

- Visualize the trajectory of a certain event interval 
>simact(T, knowrob_sim:'TouchingSituation'), simact_start(T,Start), simact_end(T, End), add_trajectory_sim('hit_hand', Start, End, 100,0,0.5).

- Select an event interval involving a certain objecttype 
>simact(T, knowrob_sim:'TouchingSituation', knowrob:'Cup'), simact_start(T,Start), simact_end(T, End).

- Select an event interval involving two certain objecttypes 
>simact(T, knowrob_sim:'TouchingSituation', knowrob:'Cup', knowrob:'KitchenTable'), simact_start(T,Start), simact_end(T, End).


- Ask for an event that involves an object that is a subclass of "Object-SupportingFurniture" 
//Useful for detecting support (?) (Could assume that all contacts with those kind of objects are supported-by relations, in lieu of a real detection of such things)
>rdf_has(Event, knowrob:'objectInContact', ObjectInstance1), rdf_has(ObjectInstance1, rdf:type, ObjClass), rdf_reachable(ObjClass, rdfs:subClassOf, knowrob:'Object-SupportingFurniture').