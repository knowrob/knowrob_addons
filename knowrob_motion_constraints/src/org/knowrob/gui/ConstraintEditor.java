/*
 *    Copyright (C) 2012
 *      ATR Intelligent Robotics and Communication Laboratories, Japan
 *
 *    Permission is hereby granted, free of charge, to any person obtaining a copy
 *    of this software and associated documentation files (the "Software"), to deal
 *    in the Software without restriction, including without limitation the rights
 *    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *    of the Software, and to permit persons to whom the Software is furnished to do so,
 *    subject to the following conditions:
 *
 *    The above copyright notice and this permission notice shall be included in all
 *    copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 *    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 *    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
package org.knowrob.gui;

import java.awt.Frame;
import java.awt.event.MouseEvent;
import java.io.File;

import org.knowrob.constr.MotionTask;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.AddImport;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLImportsDeclaration;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlEvent;
import controlP5.ControlP5;
import controlP5.ControllerInterface;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;
import edu.tum.cs.ias.knowrob.owl.utils.OWLImportExport;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import processing.core.PApplet;
import ros.pkg.knowrob_motion_constraints.srv.ConstraintsToOWL;


/**
 * Graphical editor for motion constraint specifications
 * 
 * @author Moritz Tenorth, tenorth@cs.uni-bremen.de
 *
 */
public class ConstraintEditor extends PApplet {

	private static final long serialVersionUID = -284448276454939406L;

	public ControlP5 controlP5;

	protected MotionTask task;


	private long lastClickTime = 0;

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



	/**
	 * Initialize the applet and load the GUI elements and embedded applets.
	 */
	@Override
	public void setup() {

		size(1250, 750, P2D);
		textMode(SCREEN);
		background(color(255));
		controlP5 = new ControlP5(this);
		frameRate(20);

		stroke(150);
		strokeWeight(1);
		
		task = new MotionTask("Flipping a pancake", controlP5);
		
		controlP5.addButton("Save to OWL", 10.0f).setWidth(120).setColorForeground(0).setColorBackground(180).setPosition(40,15);

		// set initial values for 'add' box dimensions
		task.recomputeBoxDimensions();
	}


	@Override
	public void draw() {

		background(255);
		
		if(task!=null) {
			task.draw(this, 40, 60, controlP5);
		}

	}




	@Override
	public void mouseClicked(MouseEvent e) {

		long diff = System.currentTimeMillis()-lastClickTime ;
		lastClickTime = System.currentTimeMillis();

		if (diff < 10) // double fired event
			return;

		if (e.getButton() == MouseEvent.BUTTON1) {

			if(e.getX() > task.ADD_PHASE_BOX_X && e.getX() < task.ADD_PHASE_BOX_X + task.ADD_PHASE_BOX_W &&
					e.getY() > task.ADD_PHASE_BOX_Y && e.getY() < task.ADD_PHASE_BOX_Y + task.ADD_PHASE_BOX_H )
				task.addMotionPhase();

			if(e.getX() > task.ADD_CONSTR_BOX_X && e.getX() < task.ADD_CONSTR_BOX_X + task.ADD_CONSTR_BOX_W &&
					e.getY() > task.ADD_CONSTR_BOX_Y && e.getY() < task.ADD_CONSTR_BOX_Y + task.ADD_CONSTR_BOX_H )
				task.addMotionConstraint();
			

		}
	}
	

	public void writeToOWL(String filename) {
		

		OWLOntology ontology = null;

		// Create ontology manager and data factory
		OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
		OWLDataFactory factory = manager.getOWLDataFactory();
		DefaultPrefixManager pm = PREFIX_MANAGER;;

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

		task.writeToOWL(manager, factory, pm, ontology);
		
		OWLFileUtils.saveOntologyToFile(ontology, ontology.getOWLOntologyManager().getOntologyFormat(ontology), filename);
		
	}

	public void setFrame(Frame frame) {
		this.frame = frame;
	}


	public MotionTask getTask() {
		return task;
	}

	public void clearControlP5() {
		
		synchronized(controlP5) {
			for(ControllerInterface<?> c : controlP5.getAll()) {
				controlP5.remove(c.getName());
			}
		}

		controlP5.addButton("Save to OWL", 10.0f).setWidth(120).setColorForeground(0).setColorBackground(180).setPosition(40,15);

	}


	public void controlEvent(ControlEvent theEvent) {

		if (theEvent.isController() && theEvent.getController().getValue()==10f) {

			String savePath = selectOutput();  // opens file chooser
			if (savePath == null) {
				// no file selected
				return;
				
			} else {

				if(new File(savePath).isDirectory()) {
					savePath += File.separator + task.getName() + ".owl";
				}
				
				// export as file
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
				
				OWLFileUtils.saveOntologyToFile(ontology, savePath, manager.getOntologyFormat(ontology));
				
			}
		}

	}
	public void setTask(MotionTask task) {
		this.task = task;
	}
	public static void main(String args[]) {
		PApplet.main(new String[] { "org.knowrob.gui.ConstraintEditor" });
	}

}

