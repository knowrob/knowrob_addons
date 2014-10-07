package org.knowrob.constr;


import java.awt.BorderLayout;
import java.awt.Frame;

import org.knowrob.gui.ConstraintEditor;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;

import ros.*;
import ros.pkg.knowrob_motion_constraints.srv.ConstraintsToOWL;



public class ConstraintsToOWLService {



	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	protected ConstraintEditor editor;

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

		// start editor GUI
		Frame myFrame = new Frame("Constraint editor");

		editor = new ConstraintEditor();
		editor.init();
		
	    // add applet to the frame
	    myFrame.add(editor, BorderLayout.CENTER);
	    myFrame.pack();
	    myFrame.setVisible(true);
		editor.setFrame(myFrame);


		try {

			initRos("constraints_to_owl");

			n.advertiseService("constraints_to_owl", 
					new ConstraintsToOWL(), 
					new ConvertToOwlCallback());
			ros.spin();

		} catch (RosException e) {
			e.printStackTrace();	
		}
	}


	/**
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceServer.Callback<ConstraintsToOWL.Request, ConstraintsToOWL.Response> {

		@Override
		public ConstraintsToOWL.Response call(ConstraintsToOWL.Request req) {

			editor.clearControlP5();
			editor.setTask(null);
			
			MotionTask task = new MotionTask(req.task, editor.controlP5);
			
			OWLOntology ontology = null;

			// Create ontology manager and data factory
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = MotionTask.PREFIX_MANAGER;

			// Create empty OWL ontology
			try {

				ontology = manager.createOntology(IRI.create(MotionTask.MOTION));
				manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			} catch (OWLOntologyCreationException e) {
				e.printStackTrace();
			}

			// Import motion constraints ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(MotionTask.CONSTR));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			task.writeToOWL(manager, factory, pm, ontology);
			
			ConstraintsToOWL.Response res = new ConstraintsToOWL.Response();
			
			res.owl_data = beautifyOWL(OWLFileUtils.saveOntologytoString(ontology, manager.getOntologyFormat(ontology)));
			
			task.recomputeBoxDimensions();
			editor.setTask(task);
			
			return res;

		}
	}


	private String beautifyOWL(String owl_data) {
		
		String header = "\n\n" +
		"<!DOCTYPE rdf:RDF [\n" +
	    "    <!ENTITY local_path 'file://@LOCAL_PACKAGE_PATH@/owl/'>\n" +
		"    <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n" +
		"    <!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\" >\n" +
		"    <!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\" >\n" +
		"    <!ENTITY motion \"http://knowrob.org/kb/motion-def.owl#\" >\n" +
		"    <!ENTITY constr \"http://knowrob.org/kb/motion-constraints.owl#\" >\n" +
		"    <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\" >\n" +
		"    <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n" +
		"]>\n\n<rdf:RDF";
		
		owl_data = owl_data.replace("rdf:resource=\"http://knowrob.org/kb/knowrob.owl#", 
				"rdf:resource=\"&knowrob;");
		owl_data = owl_data.replace("rdf:about=\"http://knowrob.org/kb/knowrob.owl#", 
				"rdf:about=\"&knowrob;");

		owl_data = owl_data.replace("rdf:resource=\"http://knowrob.org/kb/motion-def.owl#", 
				"rdf:resource=\"&motion;");
		owl_data = owl_data.replace("rdf:about=\"http://knowrob.org/kb/motion-def.owl#", 
				"rdf:about=\"&motion;");

		owl_data = owl_data.replace("rdf:resource=\"http://knowrob.org/kb/motion-constraints.owl#", 
				"rdf:resource=\"&constr;");
		owl_data = owl_data.replace("rdf:about=\"http://knowrob.org/kb/motion-constraints.owl#", 
				"rdf:about=\"&constr;");

		owl_data = owl_data.replace("rdf:datatype=\"http://www.w3.org/2001/XMLSchema#", 
									"rdf:datatype=\"&xsd;");
		
		owl_data = owl_data.replace("<owl:imports rdf:resource=\"&constr;\"/>", 
				"<owl:imports rdf:resource=\"&local_path;motion-constraints.owl\"/>");
		
		
		
		owl_data = owl_data.replace("<rdf:RDF", header);
		return owl_data;
	}



	public static void main(String[] args) {
		new ConstraintsToOWLService();
	}
}







