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







