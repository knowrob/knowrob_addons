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
			
			res.owl_data = OWLFileUtils.saveOntologytoString(ontology, manager.getOntologyFormat(ontology));
			
			
			task.recomputeBoxDimensions();
			editor.setTask(task);
			
			return res;

		}
	}


	public static void main(String[] args) {
		new ConstraintsToOWLService();
	}
}







