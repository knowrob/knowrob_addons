package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.AddImport;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyExpression;
import org.semanticweb.owlapi.model.OWLImportsDeclaration;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLLiteral;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyExpression;
import org.semanticweb.owlapi.model.OWLObjectSomeValuesFrom;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.util.OWLClassExpressionVisitorAdapter;

import controlP5.ControlP5;
import edu.tum.cs.ias.knowrob.owl.JointInstance;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;
import edu.tum.cs.ias.knowrob.owl.utils.OWLImportExport;


import processing.core.PApplet;
import ros.pkg.constraint_msgs.msg.Constraint;
import ros.pkg.constraint_msgs.msg.ConstraintCommand;
import ros.pkg.constraint_msgs.msg.ConstraintConfig;

public class MotionTask {

	protected String name = "";
	protected List<MotionPhase> phases;
	protected List<MotionConstraintTemplate> templates;

	public float ADD_PHASE_BOX_X = 40;
	public float ADD_PHASE_BOX_Y = 40;
	public float ADD_PHASE_BOX_W = 120 + 40;
	public float ADD_PHASE_BOX_H = 40;

	public float ADD_CONSTR_BOX_X = 40 + 120;
	public float ADD_CONSTR_BOX_Y = 40;
	public float ADD_CONSTR_BOX_W = 40;
	public float ADD_CONSTR_BOX_H = 40;

	ControlP5 controlP5;



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




	public MotionTask() {

		phases = Collections.synchronizedList(new ArrayList<MotionPhase>());
		templates = Collections.synchronizedList(new ArrayList<MotionConstraintTemplate>());

	}


	public MotionTask(String name, ControlP5 controlP5) {

		this();

		this.name = name;
		this.controlP5 = controlP5;

		controlP5.addTextfield(name + "_name").setText(name).setWidth(120).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).setPosition(40,15);

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

		synchronized(phases) {

			// draw motion template headers
			for(MotionConstraintTemplate t : templates) {
				t.draw(c, curX + 120, curY, controlP5);
				curX += MotionConstraintTemplate.TEMPLATE_BOX_WIDTH;
			}

			curX = x;
			curY += MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;

			// draw motion phases and constraints
			for(MotionPhase p : phases) {

				p.draw(c, curX, curY, controlP5);
				curY += MotionConstraint.CONSTRAINT_BOX_HEIGHT;

			}
			controlP5.draw();
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
				ADD_PHASE_BOX_Y = 40 + (phases.size()*MotionConstraint.CONSTRAINT_BOX_HEIGHT) + MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;
				ADD_PHASE_BOX_W = 120 + 40 + phases.get(0).getConstraints().size() * MotionConstraint.CONSTRAINT_BOX_WIDTH;
				ADD_PHASE_BOX_H = 40;

				ADD_CONSTR_BOX_X = 40 + 120+ phases.get(0).getConstraints().size() * MotionConstraint.CONSTRAINT_BOX_WIDTH ;
				ADD_CONSTR_BOX_Y = 40 + MotionConstraintTemplate.TEMPLATE_BOX_HEIGHT;
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

					p.addConstraint(new MotionConstraint(OWLThing.getUniqueID("").substring(1), new String[]{""}, true, 0f, 0f, c.getTemplate(), controlP5));

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

			MotionConstraintTemplate tmpl = new MotionConstraintTemplate(OWLThing.getUniqueID("").substring(1), new String[]{""}, "", "", controlP5);

			templates.add(tmpl);

			for(MotionPhase p : phases) {

				p.addConstraint(new MotionConstraint(OWLThing.getUniqueID("").substring(1), new String[]{""}, true, 0f, 0f, tmpl, controlP5));

				try { Thread.sleep(5); } catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			recomputeBoxDimensions();
		}
	}



	public void fillWithTestData() {

		synchronized(phases) {
			for(int i=0;i<5;i++) {

				MotionPhase p = new MotionPhase("phase " + i, controlP5);


				MotionConstraintTemplate tmpl = new MotionConstraintTemplate(OWLThing.getUniqueID("").substring(1), new String[]{""}, "handle", "pancake", controlP5);
				templates.add(tmpl);

				try { Thread.sleep(5); } catch (InterruptedException e) {
					e.printStackTrace();
				}

				for(int j=0;j<5;j++) {

					MotionConstraint c = new MotionConstraint(OWLThing.getUniqueID("").substring(1), 
							new String[]{"DirectionConstraint"}, 
							false, 0.2, 0.6, tmpl, controlP5);
					p.addConstraint(c);

					try { Thread.sleep(5); } catch (InterruptedException e) {
						e.printStackTrace();
					}
				}

				phases.add(p);
			}
		}
	}



	public void readFromOWL(String filename, String actionClassIRI) {

		try {

			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();

			OWLOntology ont = OWLFileUtils.loadOntologyFromFile(filename);

			if(ont!=null) {

				OWLClass actioncls = factory.getOWLClass(IRI.create(actionClassIRI));
				OWLObjectProperty subAction = factory.getOWLObjectProperty(IRI.create(KNOWROB + "subAction"));
				OWLObjectProperty constrainedBy = factory.getOWLObjectProperty(IRI.create(KNOWROB + "constrainedBy"));
				
				
				
				// read sub-actions == motion phases
				RestrictionVisitor subActionVisitor = new RestrictionVisitor(Collections.singleton(ont), subAction);
				
				for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(actioncls)) {
					OWLClassExpression superCls = ax.getSuperClass();
					superCls.accept(subActionVisitor);
				}
								
				for (OWLClassExpression sub : subActionVisitor.getRestrictionFillers()) {

					// TODO: sort based on ordering constraints
					System.out.println(sub.toString());	

					MotionPhase p = new MotionPhase(sub.toString().substring(1, sub.toString().length()-1), controlP5);
					phases.add(p);
					
					// TODO: read templates from OWL
					MotionConstraintTemplate tmpl = new MotionConstraintTemplate(OWLThing.getUniqueID("").substring(1), new String[]{""}, "handle", "pancake", controlP5);
					templates.add(tmpl);

					// read constraints for phases
					RestrictionVisitor constrainedByVisitor = new RestrictionVisitor(Collections.singleton(ont), constrainedBy);
					
					for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass((OWLClass)sub)) {
						OWLClassExpression superCls = ax.getSuperClass();
						superCls.accept(constrainedByVisitor);
					}
					
					// TODO: associate constraints to templates
					for (OWLClassExpression constr : constrainedByVisitor.getRestrictionFillers()) {

						MotionConstraint c = new MotionConstraint(constr.toString(), 
								new String[]{"DirectionConstraint"}, 
								false, 0.2, 0.6, tmpl, controlP5);
						p.addConstraint(c);
					}
				}

				

			}
		} catch (OWLOntologyCreationException e) {
			e.printStackTrace();
		}
	}



	public void writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {


		// TODO: adapt and check!!

		ArrayList<OWLClass> phases_cls = new ArrayList<OWLClass>();
		OWLClass itascmotion = factory.getOWLClass(IRI.create(KNOWROB + "ITaSCMotion"));

		for(MotionPhase p : phases) {

			OWLClass phaseCl = factory.getOWLClass(IRI.create(CONSTR + p.getName()));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCl, itascmotion)));

			phases_cls.add(phaseCl);

			p.writeToOWL(manager, factory, pm, ontology);
		}
	}



	/** Visits existential restrictions and collects the properties which are restricted
	 * 
	 *  (adapted from OWLAPI examples)
	 * 
	 */

	private static class RestrictionVisitor extends OWLClassExpressionVisitorAdapter {

		private Set<OWLClass> processedClasses;
//		private Set<OWLObjectPropertyExpression> restrictedProperties;
		private Set<OWLClassExpression> restrictionFillers;
		private Set<OWLOntology> onts;
		private OWLObjectProperty prop;

		public RestrictionVisitor(Set<OWLOntology> onts, OWLObjectProperty prop) {
			restrictionFillers = new HashSet<OWLClassExpression>();
			processedClasses = new HashSet<OWLClass>();
			this.onts = onts;
			this.prop = prop;
		}

		public Set<OWLClassExpression> getRestrictionFillers() {
			return restrictionFillers;
		}

		@Override
		public void visit(OWLClass desc) {

			if (!processedClasses.contains(desc)) {
				
				processedClasses.add(desc);
				
				for (OWLOntology ont : onts) {
					for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(desc)) {
						ax.getSuperClass().accept(this);
					}
				}
			}
		}

		@Override
		public void visit(OWLObjectSomeValuesFrom desc) {
			// This method gets called when a class expression is an existential
			// (someValuesFrom) restriction and it asks us to visit it

			if(desc.getProperty().equals(prop))
				restrictionFillers.add(desc.getFiller());
		}
	} 
}
