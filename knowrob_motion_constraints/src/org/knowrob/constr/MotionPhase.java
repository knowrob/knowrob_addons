package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.knowrob.constr.util.RestrictionVisitor;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;


import processing.core.PApplet;

public class MotionPhase {

	protected String name = "";
	private String label;
	protected ArrayList<MotionConstraint> constraints;


	public MotionPhase() {
		this.constraints = new ArrayList<MotionConstraint>();
	}

	public MotionPhase(ros.pkg.knowrob_motion_constraints.msg.MotionPhase msg, List<MotionConstraintTemplate> templates, ControlP5 controlP5) {
		
		this.name = msg.name;
		
		for(ros.pkg.knowrob_motion_constraints.msg.MotionConstraint constr : msg.constraints) {
			this.constraints.add(new MotionConstraint(constr, templates, controlP5));
		}
	}
	
	
	public MotionPhase(String name, ControlP5 controlP5) {

		this();

		this.name = name;

		controlP5.addTextfield(name + "_name").setText(name).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);

	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {

		int curX = x;
		int curY = y;


		c.fill(255);
		c.rect(curX,curY,120, MotionConstraint.CONSTRAINT_BOX_HEIGHT);
		controlP5.get(name + "_name").setPosition(curX+10, curY+MotionConstraint.CONSTRAINT_BOX_HEIGHT/2-10);

		curX += 120;

		for(MotionConstraint constr : constraints) {

			constr.draw(c, curX, curY, controlP5);
			curX += MotionConstraint.CONSTRAINT_BOX_WIDTH;
		}

	}

	public String getName() {
		return name;
	}


	public void setName(String name) {
		this.name = name;
	}

	public ArrayList<MotionConstraint> getConstraints() {
		return constraints;
	}


	public void setConstraints(ArrayList<MotionConstraint> constraints) {
		this.constraints = constraints;
	}


	public MotionConstraint getConstraint(String name) {
		for(MotionConstraint c : constraints) {
			if(c.getName().equals(name))
				return c;
		}
		return null;
	}

	public boolean addConstraint(MotionConstraint e) {
		return constraints.add(e);
	}


	public boolean addAllConstraint(Collection<? extends MotionConstraint> c) {
		return constraints.addAll(c);
	}


	public void clearConstraints() {
		constraints.clear();
	}


	public boolean containsConstraint(Object o) {
		return constraints.contains(o);
	}


	public MotionConstraint getConstraint(int index) {
		return constraints.get(index);
	}


	public boolean constraintsEmpty() {
		return constraints.isEmpty();
	}


	public MotionConstraint removeConstraint(int index) {
		return constraints.remove(index);
	}


	public boolean removeConstraint(Object o) {
		return constraints.remove(o);
	}


	public boolean removeAllConstraints(Collection<?> c) {
		return constraints.removeAll(c);
	}


	public int numConstraints() {
		return constraints.size();
	}



	public void readFromOWL(OWLClass phaseCls, MotionConstraintTemplate tmpl, OWLOntology ont, OWLDataFactory factory, ControlP5 controlP5) {

		OWLObjectProperty constrainedBy = factory.getOWLObjectProperty(IRI.create(MotionTask.KNOWROB + "constrainedBy"));
		
		// read constraints for phases
		RestrictionVisitor constrainedByVisitor = new RestrictionVisitor(Collections.singleton(ont), constrainedBy);
		
		for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass((OWLClass)phaseCls)) {
			OWLClassExpression superCls = ax.getSuperClass();
			superCls.accept(constrainedByVisitor);
		}
		
		// TODO: associate constraints to templates
		for (OWLClassExpression constr : constrainedByVisitor.getRestrictionFillers()) {

			
			MotionConstraint c = new MotionConstraint(constr.toString(), 
					new String[]{"DirectionConstraint"}, false, 0.0, 0.0, tmpl, controlP5);

			c.readFromOWL((OWLClass) constr, ont, factory, controlP5);
			
			this.addConstraint(c);
		}

	}



	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		OWLClass itascmotion = factory.getOWLClass(IRI.create(MotionTask.KNOWROB + "ITaSCMotion"));
		OWLClass phaseCl = factory.getOWLClass(IRI.create(MotionTask.MOTION + name));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCl, itascmotion)));

		// set label
		if(!this.label.isEmpty())
			manager.applyChange(new AddAxiom(ontology, 
					factory.getOWLAnnotationAssertionAxiom(
							factory.getRDFSLabel(), 
							IRI.create(MotionTask.MOTION + name), 
							factory.getOWLLiteral(this.label)))); 
		
		
		OWLObjectProperty constrainedBy = factory.getOWLObjectProperty(IRI.create(MotionTask.KNOWROB + "constrainedBy"));
		for(MotionConstraint constr : constraints) {
			
			if(constr.isActive()) {
				OWLClass constrCl = constr.writeToOWL(manager, factory, pm, ontology);
				
				OWLClassExpression constrainedByRestr = factory.getOWLObjectSomeValuesFrom(constrainedBy, constrCl);
				manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCl, constrainedByRestr))); 
			}
		}
		
		return phaseCl;
	}

}
