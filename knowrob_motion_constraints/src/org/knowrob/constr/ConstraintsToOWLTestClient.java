/*
 * Copyright (c) 2012-14 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
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
		phase3.name = "TiltBack";
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



