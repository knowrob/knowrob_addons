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
import java.util.List;

import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;

import processing.core.PApplet;


public class MotionConstraintTemplate {

	protected String name = "";
	private String label = "";
	protected ArrayList<String> types;

	protected String toolFeature = "";
	protected String worldFeature = "";
	protected String reference = "";
	
	public static int TEMPLATE_BOX_WIDTH  = 170;
	public static int TEMPLATE_BOX_HEIGHT = 80;

	

	public MotionConstraintTemplate() {

		types = new ArrayList<String>();

	}
	

	public MotionConstraintTemplate(ros.pkg.knowrob_motion_constraints.msg.MotionConstraintTemplate msg, ControlP5 controlP5) {
		//this(msg.name, msg.types, msg.toolFeature, msg.worldFeature, msg.reference, controlP5);
		this.label = msg.label;
	}

	public MotionConstraintTemplate(String name, List<String> types, String toolFeature, String worldFeature, String reference, ControlP5 controlP5) {

		this();

		this.name = name;
		this.types = new ArrayList<String>(types);
		
		this.toolFeature = toolFeature;
		this.worldFeature = worldFeature;
		this.reference = reference;

		synchronized(controlP5) {
			controlP5.addTextfield(name + "_name").setText(name).setWidth(140).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);

			controlP5.addTextlabel(name + "_tool_label").setText("tool").setColorValueLabel(80);
			controlP5.addTextfield(name + "_tool").setText(toolFeature).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
			controlP5.addTextlabel(name + "_world_label").setText("world").setColorValueLabel(80);
			controlP5.addTextfield(name + "_world").setText(worldFeature).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
			controlP5.addTextlabel(name + "_ref_label").setText("ref").setColorValueLabel(80);
			controlP5.addTextfield(name + "_ref").setText(worldFeature).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
		}
	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {

		synchronized(controlP5) {
			controlP5.get(name + "_name").setPosition(x+15, y+5);

			c.fill(255);
			c.rect(x,y,TEMPLATE_BOX_WIDTH, TEMPLATE_BOX_HEIGHT);

			c.fill(100);
			controlP5.get(name + "_tool_label").setPosition(x+15, y+30);
			controlP5.get(name + "_world_label").setPosition(x+15, y+50);

			controlP5.get(name + "_tool").setPosition(x+55, y+25);
			controlP5.get(name + "_world").setPosition(x+55, y+45);
			
			controlP5.get(name + "_tool").setPosition(x+95, y+25);
			controlP5.get(name + "_world").setPosition(x+95, y+45);

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


	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		
		// create template class
		OWLClass templateCl = factory.getOWLClass(IRI.create(MotionTask.MOTION + name));
		
		
		// set super classes
		for(String t : types) {
			OWLClass sup = factory.getOWLClass(IRI.create(MotionTask.CONSTR + t));
			manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(templateCl, sup)));
		}

		// set label
		if(!this.label.isEmpty())
			manager.applyChange(new AddAxiom(ontology, 
					factory.getOWLAnnotationAssertionAxiom(
							factory.getRDFSLabel(), 
							IRI.create(MotionTask.MOTION + name), 
							factory.getOWLLiteral(this.label)))); 
		
		
		// annotate subclass with feature values
		OWLObjectProperty toolFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "toolFeature"));
		OWLClassExpression toolCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.toolFeature));
		OWLClassExpression toolFeatureRestr = factory.getOWLObjectSomeValuesFrom(toolFeatureProp, toolCls);
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(templateCl, toolFeatureRestr))); 


		OWLObjectProperty worldFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "worldFeature"));
		OWLClassExpression worldCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.worldFeature));
		OWLClassExpression worldFeatureRestr = factory.getOWLObjectSomeValuesFrom(worldFeatureProp, worldCls);
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(templateCl, worldFeatureRestr))); 

		OWLObjectProperty refFeatureProp = factory.getOWLObjectProperty(IRI.create(MotionTask.CONSTR + "refFeature"));
		OWLClassExpression refCls = factory.getOWLClass(IRI.create(MotionTask.CONSTR + this.reference));
		OWLClassExpression refFeatureRestr = factory.getOWLObjectSomeValuesFrom(refFeatureProp, refCls);
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(templateCl, refFeatureRestr))); 

		return templateCl;

	}

}
