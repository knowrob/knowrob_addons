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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

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
	private String label = "";
	protected ArrayList<MotionConstraint> constraints;


	public MotionPhase() {
		this.constraints = new ArrayList<MotionConstraint>();
	}

	public MotionPhase(ros.pkg.knowrob_motion_constraints.msg.MotionPhase msg, ControlP5 controlP5) {

		this(msg.name, controlP5);
		this.label = msg.label;

		for(ros.pkg.knowrob_motion_constraints.msg.MotionConstraint constr : msg.constraints) {
			this.constraints.add(new MotionConstraint(constr, controlP5));
		}
	}


	public MotionPhase(String name, ControlP5 controlP5) {

		this();

		this.name = name;

		synchronized(controlP5) {
			controlP5.addTextfield(name + "_name").setText(name).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);
		}
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



	public void readFromOWL(OWLClass phaseCls, OWLOntology ont, OWLDataFactory factory, ControlP5 controlP5) {

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
					new ArrayList<String>(Arrays.asList(new String[]{"DirectionConstraint"})), "", "", "", 0.0, 0.0, controlP5);

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

			OWLClass constrCl = constr.writeToOWL(manager, factory, pm, ontology);

			OWLClassExpression constrainedByRestr = factory.getOWLObjectSomeValuesFrom(constrainedBy, constrCl);
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCl, constrainedByRestr))); 
		}

		return phaseCl;
	}

}
