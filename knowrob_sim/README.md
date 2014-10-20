########################################
# Example calls for knowrob_sim/webrob #
########################################

NOTE: calls to add_trajectory_sim only work when visualization (web server) is running. The other calls also work in the prolog shell alone

- To start a shell: 
>rosrun rosprolog rosprolog knowrob_roslog_launch

- To start a local server: 
>roslaunch knowrob_roslog_launch knowrob.launch 

Load our sim experiment example owl file (change the path appropriately):
>load_experiment('[...]/catkin_ws/src/knowrob_addons/knowrob_sim/example_files/sim_data.owl').

Have a look which timepoints exist:
>owl_individual_of(A, 'http://knowrob.org/kb/knowrob.owl#TimePoint').

Visualize a trajectory by manually inputting start and end times 
add_trajectory_sim(linkname, start, end, timesteps, markertype, color) 
>add_trajectory_sim('Hand', 'http://knowrob.org/kb/cram_log.owl#timepoint_21590', 'http://knowrob.org/kb/cram_log.owl#timepoint_35487', 100, 0,0.5).


Visualize the trajectory of a certain event interval:
>simact(T, knowrob_sim:'TouchingSituation'), simact_start(T,Start), simact_end(T, End), add_trajectory_sim('Hand', Start, End, 100,0,0.8).

# Warning: there is some unexpected behavior from webrob in that it seems to only return one result at a time, but the predicates are actually called with all possible solutions. This gives problems for the visualization because instead of visualizing 1 segment, it will visualize all matching segments at once #
Select a contact interval involving a certain objecttype:
>simact_contact(T, knowrob_sim:'TouchingSituation', knowrob:'Cup', Obj), simact_start(T,Start), simact_end(T, End).

Select a contact interval involving two certain objecttypes:
>simact_contact(T, knowrob_sim:'TouchingSituation', knowrob:'Cup', knowrob:'KitchenTable', Obj1, Obj2), simact_start(T,Start), simact_end(T, End).
Select an interval during which Cup is grasped:
>simact(T, knowrob:'GraspingSomething'), rdf_has(T, knowrob:'objectActedOn', Obj), rdf_has(Obj, rdf:type, knowrob:'Cup'), simact_start(T,Start), simact_end(T, End).
Visualize interval during which Cup is grasped:
>simact(T, knowrob:'GraspingSomething'), rdf_has(T, knowrob:'objectActedOn', Obj), rdf_has(Obj, rdf:type, knowrob:'Cup'), simact_start(T, Start), simact_end(T, End), add_trajectory_sim('Cup', Start, End, 1,3,0.8).

Ask for an event interval during which a specific object type was lifted
>simlift(E, knowrob:'Cup', Obj), simact_start(E, Start), simact_end(E, End).
Ask for an event interval during which a specific object was lifted
>simlift_specific(E, knowrob:'Cup_object_yYXWRWQpelZIFK').
Ask when the liftonly part of lifting started and stopped
>simlift_liftonly(knowrob:'Cup', Start, End).

Ask when the flipping started and stopped
>simflip_full(knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, OObj, TObj, LObj).
Ask when the fliponly part of the flipping started and stopped
>simflip_fliponly(knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, OObj, TObj, LObj).
Visualize flipping
>simflip_full(knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, _, _, _), add_trajectory_sim('Spatula', Start, End, 100,3,0.5).

Did flipping occur in this episode?
>simflipping(O,T,S,ToolGrasped, ToolContactObject, ObjectLifted, ObjectFlipped, ToolReleased).
Visualize flipping first 3 steps
>simflipping(O,T,S,ToolGrasped, ToolContactObject, ObjectLifted, ObjectFlipped, ToolReleased), add_trajectory_sim('Spatula', ToolGrasped, ToolContactObject, 0.05,2,'fuchsia'), add_trajectory_sim('Spatula', ToolContactObject, ObjectLifted, 0.05,2,'red'), add_trajectory_sim('Spatula', ObjectLifted, ObjectFlipped, 0.05,2,'maroon').

Add timeline to diagram canvas (add_diagram not working yet for timelines in general)
>diagram_canvas.
>rdf_has(_, knowrob:'experimentName', literal(type(_, Expname))), string_concat(Exp, ' Timeline', Title), findall(Type, (simact(T), rdf_split_url(_,Type,T)), X), sim_timeline_val(X, Times), add_diagram(diagram_id2, 'Title', timeline, 'Time', 'Events', 300, 300, '12px', [[['X'],['Times']]]). 

Ask for an event that involves an object that is a subclass of "Object-SupportingFurniture" 
//Useful for detecting support (?) (Could assume that all contacts with those kind of objects are supported-by relations, in lieu of a real detection of such things):
>rdf_has(Event, knowrob:'objectInContact', ObjectInstance1), rdf_has(ObjectInstance1, rdf:type, ObjClass), rdf_reachable(ObjClass, rdfs:subClassOf, knowrob:'Object-SupportingFurniture').