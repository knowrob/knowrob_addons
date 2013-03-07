package org.knowrob.constr;

import java.util.ArrayList;
import java.util.HashMap;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;

import ros.*;
import ros.pkg.constraint_msgs.msg.Constraint;
import ros.pkg.constraint_msgs.msg.ConstraintCommand;
import ros.pkg.constraint_msgs.msg.ConstraintConfig;
import ros.pkg.knowrob_motion_constraints.srv.ConstraintsToOWL;



public class ConstraintsToOWLService {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for motion constraints ontology	
	public final static String CONSTR = "http://ias.cs.tum.edu/kb/motion-constraints.owl#";

	// Base IRI for new ontology
	public final static String CONSTR_DEF = "http://ias.cs.tum.edu/kb/motion-def.owl#";


	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(CONSTR);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("constr:", CONSTR);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
		PREFIX_MANAGER.setPrefix("constrdef:", CONSTR_DEF);
	}

	// mapping ROS-KnowRob identifiers
	protected static final HashMap<String, String> rosToKnowrob = new HashMap<String, String>();
	static {

		//TODO: check if identifiers match the ones used by Georg
		rosToKnowrob.put("direction", "DirectionConstraint");
		rosToKnowrob.put("distance", "DistanceConstraint");
		rosToKnowrob.put("height", "HeightConstraint");
		rosToKnowrob.put("angle", "AngleConstraint");
		rosToKnowrob.put("pointing_at", "PointingAtConstraint");
		rosToKnowrob.put("perpendicular", "PerpendicularityConstraint");

	}


	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;



	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	////////////////////////////////////////////////////////////////////////////////


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
	public ConstraintsToOWLService() {

		try {

			initRos("constraints_to_owl");

			n.advertiseService("/knowrob_motion_constraints/constraints_to_owl", 
					new ConstraintsToOWL(), 
					new ConvertToOwlCallback());
			ros.spin();

		} catch (RosException e) {
			e.printStackTrace();	
		}
	}




	/**
	 * 
	 * Callback class for querying the Web for the object type of a barcode
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceServer.Callback<ConstraintsToOWL.Request, ConstraintsToOWL.Response> {

		@Override
		public ConstraintsToOWL.Response call(ConstraintsToOWL.Request req) {

			ConstraintsToOWL.Response res = new ConstraintsToOWL.Response();
			res.owl_data = "";

			// make sure that data has been received 
			if (req.conf   != null && req.conf.constraints.size()>0 && 
					req.values != null && req.values.size() > 0) {

				OWLOntology ontology = null;


				// Create ontology manager and data factory
				manager = OWLManager.createOWLOntologyManager();
				factory = manager.getOWLDataFactory();
				pm = PREFIX_MANAGER;

				// Create empty OWL ontology
				try {

					ontology = manager.createOntology(IRI.create(CONSTR));
					manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

				} catch (OWLOntologyCreationException e) {
					e.printStackTrace();
				}


				// Import motion constraints ontology
				OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(CONSTR));
				AddImport addImp = new AddImport(ontology,oid);
				manager.applyChange(addImp);


				// check hidden assumption in the constraint messages:
				// number of constraints in conf needs to be equal to length of arrays in values
				assert(req.conf.constraints.size() == req.values.get(0).weight.length);


				// create OWL description of motion specification
				createITaSCMotionOWL(req.name, req.phases, req.conf, req.values, ontology, factory);


				//OWLOntology owlmap = export.createOWLMapDescription(namespace, 
				//			"SemanticEnvironmentMap" + new SimpleDateFormat("yyyyMMddHHmmss").format(Calendar.getInstance().getTime()), 
				//			semMapObj2MapObj(namespace, req.map.objects), address);
				
				res.owl_data = OWLFileUtils.saveOntologytoString(ontology, ontology.getOWLOntologyManager().getOntologyFormat(ontology));

			}

			return res;
		}
	}


	protected void createITaSCMotionOWL(String name, ArrayList<String> phases, ConstraintConfig conf, ArrayList<ConstraintCommand> values, OWLOntology ontology, OWLDataFactory factory) {


		OWLObjectProperty constrainedBy   = factory.getOWLObjectProperty(IRI.create(KNOWROB + "constrainedBy"));
		
		OWLObjectProperty toolFeature     = factory.getOWLObjectProperty(IRI.create(CONSTR + "toolFeature"));
		OWLObjectProperty worldFeature    = factory.getOWLObjectProperty(IRI.create(CONSTR + "worldFeature"));

		OWLDataProperty constrLowerLimit  = factory.getOWLDataProperty(IRI.create(CONSTR + "constrLowerLimit"));
		OWLDataProperty constrUpperLimit  = factory.getOWLDataProperty(IRI.create(CONSTR + "constrUpperLimit"));
		OWLDataProperty constrMinVelocity = factory.getOWLDataProperty(IRI.create(CONSTR + "constrMinVelocity"));
		OWLDataProperty constrMaxVelocity = factory.getOWLDataProperty(IRI.create(CONSTR + "constrMaxVelocity"));
		OWLDataProperty constrWeight      = factory.getOWLDataProperty(IRI.create(CONSTR + "constrWeight"));

		
		// iterate over phases, create ITaSCMotion identifiers for each of them
		
		// iterate over constraint types
		ArrayList<OWLClass> phases_cls = new ArrayList<OWLClass>();
		OWLClass itascmotion = factory.getOWLClass(IRI.create(KNOWROB + "ITaSCMotion"));
		
		for(String p : phases) {

			OWLClass phaseCl = factory.getOWLClass(IRI.create(CONSTR + p));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCl, itascmotion)));
			
			phases_cls.add(phaseCl);
		}
		
		
		// iterate over constraint types
		for(int c = 0; c < conf.constraints.size(); c ++) {

			Constraint constr = conf.constraints.get(c);

			// determine constraint type and create subclass
			String constr_t = CONSTR + rosToKnowrob.get(constr.function);
			String constr_n = CONSTR + constr.name;

			OWLClass constrType = factory.getOWLClass(IRI.create(constr_t));
			OWLClass constrCls = factory.getOWLClass(IRI.create(constr_n));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, constrType)));


			// annotate subclass with feature values
			OWLNamedIndividual tool = factory.getOWLNamedIndividual(IRI.create(KNOWROB + constr.tool_feature.name));
			OWLClassExpression toolFeatureRestr = factory.getOWLObjectHasValue(toolFeature, tool);
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, toolFeatureRestr))); 

			OWLNamedIndividual world = factory.getOWLNamedIndividual(IRI.create(KNOWROB + constr.world_feature.name));
			OWLClassExpression worldFeatureRestr = factory.getOWLObjectHasValue(worldFeature, world);
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, worldFeatureRestr))); 


			// iterate over temporal dimension: subsequent sets of constraints
			for(int i = 0; i < values.get(c).weight.length; i ++) {

				ConstraintCommand val = values.get(c);

				OWLClass constrVal = factory.getOWLClass(IRI.create(OWLThing.getUniqueID(constr_n)));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, constrCls)));

				// set properties
				OWLClassExpression weightRestr = factory.getOWLDataHasValue(constrWeight, factory.getOWLLiteral(val.weight[i]));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, weightRestr))); 

				OWLClassExpression lowerLimitRestr = factory.getOWLDataHasValue(constrLowerLimit, factory.getOWLLiteral(val.pos_lo[i]));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, lowerLimitRestr))); 
				OWLClassExpression upperLimitRestr = factory.getOWLDataHasValue(constrUpperLimit, factory.getOWLLiteral(val.pos_hi[i]));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, upperLimitRestr))); 

				OWLClassExpression minVelRestr = factory.getOWLDataHasValue(constrMinVelocity, factory.getOWLLiteral(val.min_vel[i]));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, minVelRestr))); 
				OWLClassExpression maxVelRestr = factory.getOWLDataHasValue(constrMaxVelocity, factory.getOWLLiteral(val.max_vel[i]));
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrVal, maxVelRestr))); 
				
				// TODO: link constraints to phases in the task

				OWLClassExpression constrainedByRestr = factory.getOWLObjectSomeValuesFrom(constrainedBy, constrVal);
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phases_cls.get(i), constrainedByRestr))); 
				
			}
		}
	}

	public static void main(String[] args) {

		ConstraintsToOWLService server = new ConstraintsToOWLService();

	}
}







