########################################
# Example calls for knowrob_sim/webrob #
########################################

NOTE: calls to add_trajectory_sim only work when visualization (web server) is running. The other calls also work in the prolog shell alone

#### Getting started ####
- To start a shell: 
>rosrun rosprolog rosprolog knowrob_roslog_launch

- To start a local server: 
>roslaunch knowrob_roslog_launch knowrob.launch 

Load our sim experiment example owl file (change the path appropriately):

*experiment files for knowrob_sim can be found in knowrob_data/logs/simulation/*
>load_experiment('[...]/sim_exp1.owl').

#### Basics ####

Have a look which timepoints exist:
>owl_individual_of(A, 'http://knowrob.org/kb/knowrob.owl#TimePoint').

Visualize the trajectory of a certain event interval for a certain object:
add_trajectory_sim(linkname, start, end, timesteps, markertype) 
>simact(T, knowrob_sim:'TouchingSituation'), simact_start(T,Start), simact_end(T, End), add_trajectory('Hand', Start, End, 0.5,3).

**Warning: there is some unexpected behavior because all solutions to are query are called upon processing a query. This gives problems for the visualization because instead of visualizing 1 segment, it will visualize all matching segments at once**

Select a contact interval involving a certain objecttype:
>simact_contact(Exp, Event, knowrob:'Cup', Obj), simact_start(Exp, Event, Start), simact_end(Exp, Event, End).

Select a contact interval involving two certain objecttypes:
>simact_contact(Exp, Event, knowrob:'Cup', knowrob:'KitchenTable', Obj1, Obj2), simact_start(Exp, Event,Start), simact_end(Exp, Event, End).

#### Example queries ####
Select an interval during which Cup is grasped:
>simgrasped(Exp, Event, knowrob:'Cup', ObjectInstance)., simact_start(Exp, Event, Start), simact_end(Exp, Event, End).

Visualize interval during which Cup is grasped:
>simgrasped(Exp, Event, knowrob:'Cup', ObjectInstance)., simact_start(Exp, Event, Start), simact_end(Exp, Event, End), add_trajectory_sim('Cup', Start, End, 0.5, 3).

Visualize the trajectories of a specific object being lifted for all available experiments (uses the fact that all solutions are visited by the backend. Uses global variable to change the color of consecutive trajectories)
> nb_setval(counter,0x0000ff), simlift_liftonly(Exp, knowrob:'Cup', Start, End), add_count(0x404000), add_trajectory('Cup', Start, End, 0.5,3), nb_getval(counter,Val), highlight_trajectory('Cup', Start, End, Val).

Ask for an event interval during which a specific object type was lifted
>simlift(Exp, Event, knowrob:'Cup', Obj), simact_start(Exp, Event, Start), simact_end(Exp, Event, End).

Ask for an event interval during which a specific object was lifted
>simlift_specific(Exp, Event, knowrob:'Cup_object_yYXWRWQpelZIFK').

Ask when the liftonly part of lifting started and stopped
>simlift_liftonly(Exp, knowrob:'Cup', Start, End).

Ask when the flipping started and stopped
>simflip_full(Exp, knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, OObj, TObj, LObj).

Ask when the fliponly part of the flipping started and stopped
>simflip_fliponly(Exp, knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, OObj, TObj, LObj).

Visualize flipping
>simflip_full(knowrob:'LiquidTangibleThing', knowrob:'Spatula', knowrob:'PancakeMaker', Start, End, _, _, _), add_trajectory_sim('Spatula', Start, End, 0.5, 3).

Did flipping occur in this episode?
>simflipping(Exp, O,T,S,ToolGrasped, ToolContactObject, ObjectLifted, ObjectFlipped, ToolReleased).

Visualize flipping first 3 steps
>simflipping(Exp, O,T,S,ToolGrasped, ToolContactObject, ObjectLifted, ObjectFlipped, ToolReleased), add_trajectory_sim('Spatula', ToolGrasped, ToolContactObject, 0.5, 3), add_trajectory_sim('Spatula', ToolContactObject, ObjectLifted, 0.5, 3), add_trajectory_sim('Spatula', ObjectLifted, ObjectFlipped, 0.5, 3).

Add timeline to diagram canvas (add_diagram not working yet for timelines in general)
>diagram_canvas.
>rdf_has(_, knowrob:'experimentName', literal(type(_, Expname))), string_concat(Expname, ' Timeline', Title), findall(Type, (simact(E,T), rdf_split_url(_,Type,T)), X), sim_timeline_val(X, Times), add_diagram(diagram_id2, 'Title', timeline, 'Time', 'Events', 300, 300, '12px', [[['X'],['Times']]]).