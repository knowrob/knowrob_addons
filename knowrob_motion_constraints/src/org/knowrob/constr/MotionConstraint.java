package org.knowrob.constr;

import java.util.ArrayList;
import java.util.List;

import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLObjectProperty;
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

	protected String toolFeature = "";
	protected String worldFeature = "";
	protected String reference = "";

	protected double constrLowerLimit;
	protected double constrUpperLimit;

	public static int CONSTRAINT_BOX_WIDTH  = 170;
	public static int CONSTRAINT_BOX_HEIGHT = 80;


	public MotionConstraint() {
		types = new ArrayList<String>();
	}


	public MotionConstraint(ros.pkg.knowrob_motion_constraints.msg.MotionConstraint msg, ControlP5 controlP5) {

		this(msg.name, msg.types, msg.toolFeature, msg.worldFeature, msg.refFeature, 
				       msg.constrLowerLimit, msg.constrUpperLimit, controlP5);
		this.label = msg.label;
		
	}
	
	public MotionConstraint(String name, List<String> types, String toolFeature, String worldFeature, 
			String refFeature, double constrLowerLimit, double constrUpperLimit, ControlP5 controlP5) {

		this();

		this.name = name;
		this.types = new ArrayList<String>(types);
		
		this.toolFeature = toolFeature;
		this.worldFeature = worldFeature;
		this.reference = refFeature;

		this.constrLowerLimit = constrLowerLimit;
		this.constrUpperLimit = constrUpperLimit;
		
		synchronized(controlP5) {
			controlP5.addTextfield(name + "_name").setWidth(140).setText(name).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);

			controlP5.addTextfield(name + "_lower").setWidth(40).setText(""+constrUpperLimit).setCaptionLabel("lower").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
			controlP5.addTextfield(name + "_upper").setWidth(40).setText(""+constrLowerLimit).setCaptionLabel("upper").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
		}
	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {

		synchronized(controlP5) {
			controlP5.get(name + "_name").setPosition(x+15, y+15);

			controlP5.get(name + "_lower").setPosition(x+15, y+35);
			controlP5.get(name + "_upper").setPosition(x+60, y+35);
//			controlP5.get(name + "_active").setPosition(x+105, y+36);

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
	
	public ArrayList<String> getTypes() {
		return types;
	}

	public void setTypes(ArrayList<String> types) {
		this.types = types;
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
		OWLClass constrCls = factory.getOWLClass(IRI.create(OWLThing.getUniqueID(MotionTask.MOTION + types.get(0))));

		// set constraint types/superclasses
		for(String t : types) {
			OWLClass constrType = factory.getOWLClass(IRI.create(MotionTask.CONSTR + t));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, constrType)));	
		}

		// set label
		if(!this.label.isEmpty())
			manager.applyChange(new AddAxiom(ontology, 
					factory.getOWLAnnotationAssertionAxiom(
							factory.getRDFSLabel(), 
							constrCls.getIRI(), 
							factory.getOWLLiteral(this.label)))); 

		// set properties
		OWLDataProperty constrLowerLimit  = factory.getOWLDataProperty(IRI.create(MotionTask.CONSTR + "constrLowerLimit"));
		OWLClassExpression lowerLimitRestr = factory.getOWLDataHasValue(constrLowerLimit, factory.getOWLLiteral(this.constrLowerLimit));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, lowerLimitRestr))); 
		
		OWLDataProperty constrUpperLimit  = factory.getOWLDataProperty(IRI.create(MotionTask.CONSTR + "constrUpperLimit"));
		OWLClassExpression upperLimitRestr = factory.getOWLDataHasValue(constrUpperLimit, factory.getOWLLiteral(this.constrUpperLimit));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, upperLimitRestr))); 

		// annotate subclass with feature values
		OWLObjectProperty toolFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "toolFeature"));
		OWLClassExpression toolCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.toolFeature));
		OWLClassExpression toolFeatureRestr = factory.getOWLObjectSomeValuesFrom(toolFeatureProp, toolCls);
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, toolFeatureRestr))); 


		OWLObjectProperty worldFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "worldFeature"));
		OWLClassExpression worldCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.worldFeature));
		OWLClassExpression worldFeatureRestr = factory.getOWLObjectSomeValuesFrom(worldFeatureProp, worldCls);
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, worldFeatureRestr))); 

		if(reference!=null && !reference.isEmpty()) {
			OWLObjectProperty refFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "refFeature"));
			OWLClassExpression refCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.reference));
			OWLClassExpression refFeatureRestr = factory.getOWLObjectSomeValuesFrom(refFeatureProp, refCls);
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(constrCls, refFeatureRestr))); 
		}
		
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
