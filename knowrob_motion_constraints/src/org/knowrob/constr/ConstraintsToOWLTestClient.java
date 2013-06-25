package org.knowrob.constr;

import ros.*;
import ros.pkg.knowrob_motion_constraints.msg.MotionConstraintTemplate;
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
		req.task.name = "FlippingAPancake";
		req.task.label = "test label task";
		
		
		
		// Constraint templates
		MotionConstraintTemplate tmpl1 = new MotionConstraintTemplate();
		tmpl1.name = "HeightLeftSpatulaPancake";
		tmpl1.label = "test label template 1";
		tmpl1.types.add("HeightConstraint");
		tmpl1.toolFeature = "Handle";
		tmpl1.worldFeature = "FlatPhysicalSurface";
		req.task.templates.add(tmpl1);

		MotionConstraintTemplate tmpl2 = new MotionConstraintTemplate();
		tmpl2.name = "AlignLeftSpatulaFrontPancake";
		tmpl2.label = "test label template 2";
		tmpl2.types.add("PerpendicularityConstraint");
		tmpl2.toolFeature = "FrontSide";
		tmpl2.worldFeature = "FlatPhysicalSurface";
		req.task.templates.add(tmpl2);
		
		
		// // // // // // // // // // // // // // // // // // //
		// Motion phase 1
		MotionPhase phase1 = new MotionPhase();
		phase1.name = "BothSpatulasTouch";
		phase1.label = "test label phase 1";
		req.task.phases.add(phase1);
		
		// Motion constraints
		MotionConstraint c1_1 = new MotionConstraint();
		c1_1.name = "HeightLeftSpatulaPancake_bDGnttMX";
		c1_1.label = "constr1-1 test label";
		c1_1.types.add("SlowMotionConstraint");
		c1_1.types.add("HeightLeftSpatulaPancake");
		c1_1.template = tmpl1;
		c1_1.active = true;
		c1_1.constrLowerLimit = 0.0;
		c1_1.constrUpperLimit = 0.01;
		phase1.constraints.add(c1_1);
		
		MotionConstraint c1_2 = new MotionConstraint();
		c1_2.name = "AlignLeftSpatulaFrontPancake_bDGnttMX";
		c1_2.label = "constr1-2 test label";
		c1_2.types.add("SlowMotionConstraint");
		c1_2.types.add("AlignLeftSpatulaFrontPancake");
		c1_2.template = tmpl2;
		c1_2.active = true;
		c1_2.constrLowerLimit = -0.05;
		c1_2.constrUpperLimit = 0.05;
		phase1.constraints.add(c1_2);



		// // // // // // // // // // // // // // // // // // //
		// Motion phase 2
		
		MotionPhase phase2 = new MotionPhase();
		phase2.name = "BothSpatulasApproach";
		phase2.label = "test label phase 2";
		req.task.phases.add(phase2);

		// Motion constraints
		MotionConstraint c2_1 = new MotionConstraint();
		c2_1.name = "HeightLeftSpatulaPancake_aneXbLGX";
		c2_1.label = "constr2-1 test label";
		c2_1.types.add("SlowMotionConstraint");
		c2_1.types.add("HeightLeftSpatulaPancake");
		c2_1.template = tmpl1;
		c2_1.active = true;
		c2_1.constrLowerLimit = 0.15;
		c2_1.constrUpperLimit = 0.17;
		phase2.constraints.add(c2_1);
		
		MotionConstraint c2_2 = new MotionConstraint();
		c2_2.name = "AlignLeftSpatulaFrontPancake_aneXbLGX";
		c2_2.label = "constr2-2 test label";
		c2_2.types.add("SlowMotionConstraint");
		c2_2.types.add("AlignLeftSpatulaFrontPancake");
		c2_2.template = tmpl2;
		c2_2.active = true;
		c2_2.constrLowerLimit = -0.1;
		c2_2.constrUpperLimit = 0.1;
		phase2.constraints.add(c2_2);
		
		
		
		

		
		

		
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

	public static void main(String[] args) {

		ConstraintsToOWLTestClient client = new ConstraintsToOWLTestClient();
		String res = client.callConversionService();
		System.out.println(res);
	}
}



