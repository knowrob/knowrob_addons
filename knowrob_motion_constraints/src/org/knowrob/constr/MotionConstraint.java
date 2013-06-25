package org.knowrob.constr;

import java.util.ArrayList;
import java.util.List;

import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;
import edu.tum.cs.ias.knowrob.owl.OWLThing;

import processing.core.PApplet;

public class MotionConstraint {

	protected String name = "";
	private String label = "";
	protected ArrayList<String> types;

	protected boolean active = true;

	protected double constrLowerLimit;
	protected double constrUpperLimit;

	protected MotionConstraintTemplate template;
	
	public static int CONSTRAINT_BOX_WIDTH  = 170;
	public static int CONSTRAINT_BOX_HEIGHT = 80;

	

	public MotionConstraint() {

		types = new ArrayList<String>();

	}


	public MotionConstraint(ros.pkg.knowrob_motion_constraints.msg.MotionConstraint msg, List<MotionConstraintTemplate> templates, ControlP5 controlP5) {

		this(msg.name, msg.types, msg.active, msg.constrLowerLimit, msg.constrUpperLimit, null, controlP5);
		this.label = msg.label;
		
		for(MotionConstraintTemplate t : templates) {
			if(t.getName().equals(msg.template.name)) {
				this.template = t;
				break;
			}
		}
	}
	
	public MotionConstraint(String name, List<String> types, boolean active, double constrLowerLimit, double constrUpperLimit, MotionConstraintTemplate template, ControlP5 controlP5) {

		this();

		this.name = name;
		this.active = active;
		this.types = new ArrayList<String>(types);
		
		this.constrLowerLimit = constrLowerLimit;
		this.constrUpperLimit = constrUpperLimit;

		this.template = template;
		
		synchronized(controlP5) {
			controlP5.addTextfield(name + "_name").setWidth(140).setText(name).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);

			controlP5.addTextfield(name + "_lower").setWidth(40).setText(""+constrUpperLimit).setCaptionLabel("lower").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
			controlP5.addTextfield(name + "_upper").setWidth(40).setText(""+constrLowerLimit).setCaptionLabel("upper").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);

			// create a toggle
			controlP5.addToggle(name + "_active").setSize(17,17).setValue(active).setCaptionLabel("active").setColorBackground(255).setColorForeground(130).setColorActive(130).getCaptionLabel().setColor(80);
		}
	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {

		synchronized(controlP5) {
			controlP5.get(name + "_name").setPosition(x+15, y+15);

			controlP5.get(name + "_lower").setPosition(x+15, y+35);
			controlP5.get(name + "_upper").setPosition(x+60, y+35);
			controlP5.get(name + "_active").setPosition(x+105, y+36);

			c.fill(255);
			c.rect(x,y,CONSTRAINT_BOX_WIDTH, CONSTRAINT_BOX_HEIGHT);

			c.fill(100);
		}

	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}
	
	public MotionConstraintTemplate getTemplate() {
		return template;
	}

	public void setTemplate(MotionConstraintTemplate tmpl) {
		this.template = tmpl;
	}

	public ArrayList<String> getTypes() {
		return types;
	}

	public void setTypes(ArrayList<String> types) {
		this.types = types;
	}

	public boolean isActive() {
		return active;
	}

	public void setActive(boolean active) {
		this.active = active;
	}

	public double getConstrLowerLimit() {
		return constrLowerLimit;
	}

	public void setConstrLowerLimit(double constrLowerLimit) {
		this.constrLowerLimit = constrLowerLimit;
	}

	public double getConstrUpperLimit() {
		return constrUpperLimit;
	}

	public void setConstrUpperLimit(double constrUpperLimit) {
		this.constrUpperLimit = constrUpperLimit;
	}


	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		// create constraint class
		String constrClsIRI = OWLThing.getUniqueID(MotionTask.MOTION + template.getName());
		OWLClass constrCls = factory.getOWLClass(IRI.create(constrClsIRI));

		// set constraint types 
		for(String t : types) {
			OWLClass constrType = factory.getOWLClass(IRI.create(MotionTask.CONSTR + t));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, constrType)));	
		}

		// set label
		if(!this.label.isEmpty())
			manager.applyChange(new AddAxiom(ontology, 
					factory.getOWLAnnotationAssertionAxiom(
							factory.getRDFSLabel(), 
							IRI.create(constrClsIRI), 
							factory.getOWLLiteral(this.label)))); 

		// set properties
		OWLDataProperty constrWeight = factory.getOWLDataProperty(IRI.create(MotionTask.CONSTR + "constrWeight"));
		OWLClassExpression weightRestr = factory.getOWLDataHasValue(constrWeight, factory.getOWLLiteral(1.0));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, weightRestr))); 

		OWLDataProperty constrLowerLimit  = factory.getOWLDataProperty(IRI.create(MotionTask.CONSTR + "constrLowerLimit"));
		OWLClassExpression lowerLimitRestr = factory.getOWLDataHasValue(constrLowerLimit, factory.getOWLLiteral(this.constrLowerLimit));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, lowerLimitRestr))); 
		
		OWLDataProperty constrUpperLimit  = factory.getOWLDataProperty(IRI.create(MotionTask.CONSTR + "constrUpperLimit"));
		OWLClassExpression upperLimitRestr = factory.getOWLDataHasValue(constrUpperLimit, factory.getOWLLiteral(this.constrUpperLimit));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, upperLimitRestr))); 

		return constrCls;

	}
	

	public void readFromOWL(OWLClass constrCls, OWLOntology ont, OWLDataFactory factory, ControlP5 controlP5) {

//		OWLObjectProperty constrainedBy   = factory.getOWLObjectProperty(IRI.create(KNOWROB + "constrainedBy"));
//
//		OWLObjectProperty toolFeature     = factory.getOWLObjectProperty(IRI.create(CONSTR + "toolFeature"));
//		OWLObjectProperty worldFeature    = factory.getOWLObjectProperty(IRI.create(CONSTR + "worldFeature"));
//
//		OWLDataProperty constrLowerLimit  = factory.getOWLDataProperty(IRI.create(CONSTR + "constrLowerLimit"));
//		OWLDataProperty constrUpperLimit  = factory.getOWLDataProperty(IRI.create(CONSTR + "constrUpperLimit"));
//		OWLDataProperty constrWeight      = factory.getOWLDataProperty(IRI.create(CONSTR + "constrWeight"));
//
//		Set<OWLClassExpression> sup1 = constrCls.getSuperClasses(ont);
//		Set<OWLClassExpression> sup2 = constrCls.getSubClasses(ont);
//		
//		Set<OWLSubClassOfAxiom> sup = ont.getSubClassAxiomsForSubClass(constrCls);
//		Set<OWLSubClassOfAxiom> sub = ont.getSubClassAxiomsForSuperClass(constrCls);
//		Set<OWLClassAxiom> all = ont.getAxioms(constrCls);

		for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass((OWLClass)constrCls)) {
			OWLClassExpression superCls = ax.getSuperClass();
			
			System.out.println(superCls.toString());
		}
		
	}

}
