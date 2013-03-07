package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.knowrob.constr.ConstraintsToOWLService.ConvertToOwlCallback;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import ros.*;
import ros.pkg.constraint_msgs.msg.Constraint;
import ros.pkg.constraint_msgs.msg.ConstraintCommand;
import ros.pkg.constraint_msgs.msg.ConstraintConfig;
import ros.pkg.constraint_msgs.msg.Feature;
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
				n.serviceClient("/knowrob_motion_constraints/constraints_to_owl" , new ConstraintsToOWL(), false);

		
		// fill request with test data
		ConstraintsToOWL.Request req = new ConstraintsToOWL.Request();

		// name and phases
		req.name = "TestAction";
		req.phases = new ArrayList<String>(Arrays.asList("Approach", "Touch"));

		
		// constraint templates
		Constraint c1 = new Constraint();
		c1.name = "PointToolAtObj";
		c1.function = "pointing_at";
		
		c1.tool_feature = new Feature();
		c1.tool_feature.frame_id = "map";
		c1.tool_feature.type = 0;
		c1.tool_feature.name = "Spatula1";
		
		c1.world_feature = new Feature();
		c1.tool_feature.frame_id = "map";
		c1.tool_feature.type = 0;
		c1.tool_feature.name = "PancakeMaker1";
		req.conf.constraints.add(c1);
		
		Constraint c2 = new Constraint();
		c2.name = "KeepToolAboveObj";
		c2.function = "height";
		
		c2.tool_feature = new Feature();
		c2.tool_feature.frame_id = "map";
		c2.tool_feature.type = 0;
		c2.tool_feature.name = "Spatula1";
		
		c2.world_feature = new Feature();
		c2.tool_feature.frame_id = "map";
		c2.tool_feature.type = 0;
		c2.tool_feature.name = "PancakeMaker1";
		req.conf.constraints.add(c2);
		
		
		// constraint values
		ConstraintCommand p1 = new ConstraintCommand();
		p1.weight  = new double[]{ 1,  0};
		p1.pos_lo  = new double[]{ 0.0,  0.0};
		p1.pos_hi  = new double[]{ 0.5,  0.3};
		p1.min_vel = new double[]{-0.2, -0.2};
		p1.max_vel = new double[]{ 0.2,  0.2};
		req.values.add(p1);
		
		ConstraintCommand p2 = new ConstraintCommand();
		p2.weight  = new double[]{ 0,  1};
		p2.pos_lo  = new double[]{ 0.1,  0.2};
		p2.pos_hi  = new double[]{ 0.1,  0.6};
		p2.min_vel = new double[]{-0.1, -0.1};
		p2.max_vel = new double[]{ 0.1,  0.1};
		req.values.add(p2);
		
		
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



