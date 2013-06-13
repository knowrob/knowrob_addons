package org.knowrob.constr;

import java.util.ArrayList;
import java.util.Arrays;

import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import controlP5.ControlP5;

import processing.core.PApplet;

public class MotionConstraintTemplate {

	protected String name = "";
	protected ArrayList<String> types;


	protected String toolFeature = "";
	protected String worldFeature = "";
	
	
	public static int TEMPLATE_BOX_WIDTH  = 150;
	public static int TEMPLATE_BOX_HEIGHT = 80;

	

	public MotionConstraintTemplate() {

		types = new ArrayList<String>();

	}

	public MotionConstraintTemplate(String name, String[] types, String toolFeature, String worldFeature, ControlP5 controlP5) {

		this();

		this.name = name;
		this.types = new ArrayList<String>(Arrays.asList(types));
		
		this.toolFeature = toolFeature;
		this.worldFeature = worldFeature;


		controlP5.addTextfield(name + "_name").setText(name).setWidth(140).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255);

		controlP5.addTextlabel(name + "_tool_label").setText("tool").setColorValueLabel(80);
		controlP5.addTextfield(name + "_tool").setText(toolFeature).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);
		controlP5.addTextlabel(name + "_world_label").setText("world").setColorValueLabel(80);
		controlP5.addTextfield(name + "_world").setText(worldFeature).setWidth(100).setCaptionLabel("").setColor(0).setColorForeground(0).setColorBackground(255).getCaptionLabel().setColor(80);

	}


	public void draw(PApplet c, int x, int y, ControlP5 controlP5) {

		controlP5.get(name + "_name").setPosition(x+5, y+5);


		c.fill(255);
		c.rect(x,y,TEMPLATE_BOX_WIDTH, TEMPLATE_BOX_HEIGHT);

		c.fill(100);
		controlP5.get(name + "_tool_label").setPosition(x+5, y+30);
		controlP5.get(name + "_world_label").setPosition(x+5, y+50);

		controlP5.get(name + "_tool").setPosition(x+45, y+25);
		controlP5.get(name + "_world").setPosition(x+45, y+45);

		c.fill(100);


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


		return null;

	}

}
