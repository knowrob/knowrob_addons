package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.knowrob.constr.util.RestrictionVisitor;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;


import processing.core.PApplet;
public class MotionTask {

	protected String name = "";
	protected String label = "";

	protected List<MotionPhase> phases;

	public float ADD_PHASE_BOX_X = 40;
	public float ADD_PHASE_BOX_Y = 100;
	public float ADD_PHASE_BOX_W = 120 + 40;
	public float ADD_PHASE_BOX_H = 40;

	public float ADD_CONSTR_BOX_X = 40 + 120;
	public float ADD_CONSTR_BOX_Y = 100;
	public float ADD_CONSTR_BOX_W = 40;
	public float ADD_CONSTR_BOX_H = 40;

	ControlP5 controlP5;



	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://knowrob.org/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for motion constraints ontology	
	public final static String CONSTR = "http://knowrob.org/kb/motion-constraints.owl#";

	// Base IRI for new ontology
	public final static String MOTION = "http://knowrob.org/kb/motion-def.owl#";


	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(CONSTR);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("constr:", CONSTR);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
		PREFIX_MANAGER.setPrefix("motion:", MOTION);
	}




	public MotionTask() {

		phases = Collections.synchronizedList(new ArrayList<MotionPhase>());

	}

	public MotionTask(ros.pkg.knowrob_motion_constraints.msg.MotionTask msg, ControlP5 controlP5) {

		this(msg.name, controlP5);
		this.label = msg.label;
		
		synchronized(phases) {
			for(ros.pkg.knowrob_motion_constraints.msg.MotionPhase phase : msg.phases) {
				this.phases.add(new MotionPhase(phase, controlP5));
			}
		}
	}

	public MotionTask(String name, ControlP5 controlP5) {

		this();

		this.name = name;
		this.controlP5 = controlP5;
		
		synchronized(controlP5) {
			controlP5.addTextfield(name + "_name").setText(name).setWidth(120).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).setPosition(40,60);
		}
	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {


		// draw 'add' boxes
		c.fill(230);
		c.rect(ADD_PHASE_BOX_X, ADD_PHASE_BOX_Y, ADD_PHASE_BOX_W, ADD_PHASE_BOX_H);
		c.rect(ADD_CONSTR_BOX_X, ADD_CONSTR_BOX_Y, ADD_CONSTR_BOX_W, ADD_CONSTR_BOX_H);


		c.pushMatrix();
		c.fill(255);
		c.textSize(32);
		c.text("+", ADD_PHASE_BOX_X + ADD_PHASE_BOX_W/2, 
				ADD_PHASE_BOX_Y + ADD_PHASE_BOX_H/2 + 10);

		c.text("+", ADD_CONSTR_BOX_X + 7, 
				ADD_CONSTR_BOX_Y + ADD_CONSTR_BOX_H/2 + 7);

		c.popMatrix();

		int curX = x;
		int curY = y;

//		// draw motion template headers
//		synchronized(templates) {
//			for(MotionConstraintTemplate t : templates) {
//				t.draw(c, curX + 120, curY, controlP5);
//				curX += MotionConstraintTemplate.TEMPLATE_BOX_WIDTH;
//			}
//		}
//		curX = x;
//		curY += MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;

		// draw motion phases and constraints
		synchronized(phases) {
			for(MotionPhase p : phases) {
				p.draw(c, curX, curY, controlP5);
				curY += MotionConstraint.CONSTRAINT_BOX_HEIGHT;
			}
			synchronized(controlP5) {
				controlP5.draw();
			}
		}

		c.fill(100);

	}

	public String getName() {
		return name;
	}


	public void setName(String name) {
		this.name = name;
	}


	public void recomputeBoxDimensions() {

		synchronized(phases) {
			
			if(phases.size()>0) {
				
				ADD_PHASE_BOX_X = 40;
				ADD_PHASE_BOX_Y = 60 + (phases.size()*MotionConstraint.CONSTRAINT_BOX_HEIGHT) + MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;
				ADD_PHASE_BOX_W = 120 + 40 + phases.get(0).getConstraints().size() * MotionConstraint.CONSTRAINT_BOX_WIDTH;
				ADD_PHASE_BOX_H = 40;

				ADD_CONSTR_BOX_X = 40 + 120+ phases.get(0).getConstraints().size() * MotionConstraint.CONSTRAINT_BOX_WIDTH ;
				ADD_CONSTR_BOX_Y = 60 + MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;
				ADD_CONSTR_BOX_W = 40;
				ADD_CONSTR_BOX_H = (phases.size()*MotionConstraint.CONSTRAINT_BOX_HEIGHT);
			}
		}
	}	


	// add new motion phase with empty constraints
	public void addMotionPhase() {

		synchronized(phases) {
			
			MotionPhase p = new MotionPhase(OWLThing.getUniqueID("").substring(1), controlP5);

			try { Thread.sleep(5); } catch (InterruptedException e) {
				e.printStackTrace();
			}

			if(phases.size()>0) {

				for(MotionConstraint c : phases.get(0).getConstraints()) {

					p.addConstraint(new MotionConstraint(OWLThing.getUniqueID("").substring(1), new ArrayList<String>(), "", "", "", 0f, 0f, controlP5));

					try { Thread.sleep(5); } catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			this.phases.add(p);

			recomputeBoxDimensions();
		}
	}


	// add new constraint to all motion phases
	public void addMotionConstraint() {

		synchronized(phases) {

			for(MotionPhase p : phases) {

				p.addConstraint(new MotionConstraint(OWLThing.getUniqueID("").substring(1), new ArrayList<String>(), "", "", "", 0f, 0f, controlP5));

				try { Thread.sleep(5); } catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			recomputeBoxDimensions();
		}
	}



	public void readFromOWL(String filename, String actionClassIRI) {

		try {
			synchronized(phases) {
				OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
				OWLDataFactory factory = manager.getOWLDataFactory();

				OWLOntology ont = OWLFileUtils.loadOntologyFromFile(filename);

				if(ont!=null) {

					OWLClass actioncls = factory.getOWLClass(IRI.create(actionClassIRI));
					OWLObjectProperty subAction = factory.getOWLObjectProperty(IRI.create(KNOWROB + "subAction"));


					// read sub-actions == motion phases
					RestrictionVisitor subActionVisitor = new RestrictionVisitor(Collections.singleton(ont), subAction);

					for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(actioncls)) {
						OWLClassExpression superCls = ax.getSuperClass();
						superCls.accept(subActionVisitor);
					}

					for (OWLClassExpression sub : subActionVisitor.getRestrictionFillers()) {

						// TODO: sort based on ordering constraints
						MotionPhase p = new MotionPhase(sub.toString().substring(1, sub.toString().length()-1), controlP5);
						p.readFromOWL((OWLClass) sub, ont, factory, controlP5);

						phases.add(p);
					}
				}
			}
		} catch (OWLOntologyCreationException e) {
			e.printStackTrace();
		}
	}



	public void writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {


		// create class identifier
		OWLClass motiontask = factory.getOWLClass(IRI.create(MOTION + this.name));

		// set label
		if(!this.label.isEmpty())
			manager.applyChange(new AddAxiom(ontology, 
					factory.getOWLAnnotationAssertionAxiom(
							factory.getRDFSLabel(), 
							IRI.create(MOTION + this.name), 
							factory.getOWLLiteral(this.label)))); 


		// write motion phases
		ArrayList<OWLClass> phasesCls = new ArrayList<OWLClass>();
		OWLObjectProperty subAction = factory.getOWLObjectProperty(IRI.create(KNOWROB + "subAction"));
		
		synchronized(phases) {
			
			for(MotionPhase p : phases) {

				OWLClass phaseCls = p.writeToOWL(manager, factory, pm, ontology);
				phasesCls.add(phaseCls);

				// add as sub-action
				OWLClassExpression subActionRestr = factory.getOWLObjectSomeValuesFrom(subAction, phaseCls);
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(motiontask, subActionRestr))); 
			}
			
		}

		// write ordering constraints
		OWLObjectProperty orderingConstraints = factory.getOWLObjectProperty(IRI.create(KNOWROB + "orderingConstraints"));
		OWLObjectProperty occursBeforeInOrdering = factory.getOWLObjectProperty(IRI.create(KNOWROB + "occursBeforeInOrdering"));
		OWLObjectProperty occursAfterInOrdering = factory.getOWLObjectProperty(IRI.create(KNOWROB + "occursAfterInOrdering"));

		OWLClass partialOrderingClass = factory.getOWLClass(IRI.create(KNOWROB + "PartialOrdering-Strict"));

		for(int i = 0; i<phasesCls.size(); i++) {
			for(int j=i+1; j<phasesCls.size(); j++) {

				// create ordering instance
				try{

					OWLNamedIndividual ordering = factory.getOWLNamedIndividual(IRI.create(MOTION + this.name + i + j + OWLThing.getUniqueID("")));
					manager.applyChanges(manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(partialOrderingClass, ordering)));

					OWLNamedIndividual pre = factory.getOWLNamedIndividual(IRI.create(phasesCls.get(i).toStringID()));
					OWLNamedIndividual post = factory.getOWLNamedIndividual(IRI.create(phasesCls.get(j).toStringID()));

					manager.applyChanges(manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(occursBeforeInOrdering, ordering, pre)));
					manager.applyChanges(manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(occursAfterInOrdering, ordering, post)));

					// add restriction to task class
					OWLClassExpression orderingRestr = factory.getOWLObjectHasValue(orderingConstraints, ordering);
					manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(motiontask, orderingRestr))); 

				} catch( Exception e) {
					e.printStackTrace();
				}
			}
		}
	}
}
