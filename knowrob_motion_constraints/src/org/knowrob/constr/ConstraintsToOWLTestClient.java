package org.knowrob.constr;

import java.util.ArrayList;

import ros.*;
import ros.pkg.knowrob_motion_constraints.msg.MotionConstraint;
import ros.pkg.knowrob_motion_constraints.msg.MotionPhase;
import ros.pkg.knowrob_motion_constraints.msg.MotionTask;
import ros.pkg.knowrob_motion_constraints.srv.ConstraintsToOWL;


public class ConstraintsToOWLTestClient {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;


	/**
	 * Initialize the ROS environment if it has not yet been initialized
	 * 
	 * @param node_name A unique node name
	 */
	protected static void initRos(String node_name) {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init(node_name);
		}
		n = ros.createNodeHandle();

	}

	/**
	 * Constructor. Advertises the needed ROS services.
	 */
	public ConstraintsToOWLTestClient() {
		initRos("constraints_to_owl_client");

	}

	public String callConversionService() {

		ServiceClient<ConstraintsToOWL.Request, ConstraintsToOWL.Response, ConstraintsToOWL> sc =
				n.serviceClient("constraints_to_owl" , new ConstraintsToOWL(), false);

		
		// fill request with test data
		ConstraintsToOWL.Request req = new ConstraintsToOWL.Request();

		
		// Task definition
		req.task = new MotionTask();
		req.task.name = "PouringSomething";
		req.task.label = "test pouring task";
		

		MotionConstraint top_distance_constraint = 
				makeConstraint("distance bottle-top to oven-center", 
						"DistanceConstraint", 
						"bla 1", 
						"bottle-top", 
						"oven-center", 
						"", 
						0.03, 
						0.07);

		MotionConstraint top_height_constraint = 
				makeConstraint("height bottom-top over oven-center", 
						"HeightConstraint", 
						"bla 2", 
						"bottle-top", 
						"oven-center", 
						"/oven", 
						0.25, 
						0.3);

	             
		MotionConstraint bottle_upright_constraint = 
				makeConstraint("bottle upright", 
						"PerpendicularityConstraint", 
						"bla 1", 
						"bottle-axis",
						"oven-center",
						"", 
						0.95, 
						1.2);
	             
	             
		MotionConstraint bottle_pointing_oven_center = 
				makeConstraint("bottle pointing oven center", 
						"PointingAtConstraint", 
						"bla 3", 
						"bottle-axis", 
						"oven-center", 
						"", 
						-0.1, 
						0.1);
	             
	             
		MotionConstraint bottle_tilted_down = 
				makeConstraint("bottle tilted down", 
						"PerpendicularityConstraint", 
						"bla 4", 
						"bottle-axis", 
						"oven-center", 
						"", 
						-0.2, 
						-0.1);

		
		// // // // // // // // // // // // // // // // // // //
		// Motion phase 1
		
		MotionPhase phase1 = new MotionPhase();
		phase1.name = "MoveAbovePan";
		phase1.label = "test label phase 1";

		phase1.constraints.add(top_distance_constraint);
		phase1.constraints.add(top_height_constraint);
		phase1.constraints.add(bottle_upright_constraint);
		req.task.phases.add(phase1);

		
		// // // // // // // // // // // // // // // // // // //
		// Motion phase 2
		
		MotionPhase phase2 = new MotionPhase();
		phase2.name = "TiltBottle";
		phase2.label = "test label phase 2";

		phase2.constraints.add(top_distance_constraint);
		phase2.constraints.add(top_height_constraint);
		phase2.constraints.add(bottle_pointing_oven_center);
		phase2.constraints.add(bottle_tilted_down);
		req.task.phases.add(phase2);


		
		// // // // // // // // // // // // // // // // // // //
		// Motion phase 3
		
		MotionPhase phase3 = new MotionPhase();
		phase3.name = "MoveAbovePan";
		phase3.label = "test label phase 3";

		phase3.constraints.add(top_distance_constraint);
		phase3.constraints.add(top_height_constraint);
		phase3.constraints.add(bottle_upright_constraint);
		req.task.phases.add(phase3);
		
		
		ConstraintsToOWL.Response res;
		try {
			res = sc.call(req);
			return res.owl_data;
			
		} catch (RosException e) {
			e.printStackTrace();
		}
		sc.shutdown();

		return "";
	}
	
	
	
	private MotionConstraint makeConstraint(
			String name, 
			String type, 
			String label, 
			String toolFeature, 
			String worldFeature, 
			String refFeature, 
			double d, double e) {
		
		MotionConstraint res = new MotionConstraint();
		res.name = name;
		res.label = label;
		
		res.types = new ArrayList<String>();
		res.types.add(type);
		
		res.toolFeature = toolFeature;
		res.worldFeature  = worldFeature;
		res.refFeature = refFeature;
		
		res.constrLowerLimit = d;
		res.constrUpperLimit = e;
		
		return res;
	}
	
	
	

	public static void main(String[] args) {

		ConstraintsToOWLTestClient client = new ConstraintsToOWLTestClient();
		String res = client.callConversionService();
		System.out.println(res);
	}
}



