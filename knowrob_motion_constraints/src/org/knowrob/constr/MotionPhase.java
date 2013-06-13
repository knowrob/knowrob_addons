package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Collection;

import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;


import processing.core.PApplet;

public class MotionPhase {

	protected String name = "";
	protected ArrayList<MotionConstraint> constraints;

	public MotionPhase() {
		this.constraints = new ArrayList<MotionConstraint>();
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






	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {


		for(MotionConstraint constr : constraints) {

			// TODO: export phase name, trypes, properties

			constr.writeToOWL(manager, factory, pm, ontology);
		}
		
		return null;
	}

}
