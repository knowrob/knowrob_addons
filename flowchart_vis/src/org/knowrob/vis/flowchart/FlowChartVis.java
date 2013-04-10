package org.knowrob.vis.flowchart;

import java.awt.event.MouseEvent;
import java.util.HashMap;
import java.util.Map;

import processing.core.PApplet;
import processing.core.PShape;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.List;

import org.yaml.snakeyaml.Yaml;


/**
 * Load and Display a Shape. 
 * Illustration by George Brower. 
 * 
 * The loadShape() command is used to read simple SVG (Scalable Vector Graphics)
 * files and OBJ (Object) files into a Processing sketch. This example loads an
 * SVG file of a monster robot face and displays it to the screen. 
 */

// The next line is needed if running in JavaScript Mode with Processing.js
/* @pjs preload="bot1.svg"; */ 

public class FlowChartVis extends PApplet {

	private static final long serialVersionUID = -1591411154652437400L;
	PShape chart;
	PShape bg;

	Map<String, PShape> shapes;
	Map<String, String> actiondefs;

	int bgX = 0;
	int bgY = 0;

	static String svg_file = "";
	

	public void setup() {

		size(1100, 600);
		// The file "bot1.svg" must be in the data folder
		// of the current sketch to load successfully

		chart = loadShape(svg_file);
		bg = chart.getChild("background");
		bgX = (int) bg.getParam(0);
		bgY = (int) bg.getParam(1);

		shapes = new HashMap<String, PShape> ();
		readAllShapes(chart, shapes);

		actiondefs = new HashMap<String, String>();
		try {
			loadActionDefs(svg_file.replaceAll("svg", "yaml"), actiondefs);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		hideMenus();

	}

	
	public void draw(){
		background(0xffffff);
		shape(chart, 0, 0);

	}



	/**
	 * disable all "menu" blocks
	 */
	private void hideMenus() {
		for(String name : shapes.keySet()) {
			if(name.endsWith("menu"))
				shapes.get(name).setVisible(false);
		}
	} 


	/**
	 * Handle mouse events
	 * 
	 * @param e
	 */
	public void mouseClicked(MouseEvent e) {

		PShape clicked = findClickedChild(chart, e.getX(), e.getY());

		if(clicked != null) {

			// open menu if non-menu block has been clicked
			if(!clicked.getName().endsWith("menu")) {

				hideMenus();

				PShape blk = shapes.get(clicked.getName() + "_menu");
				if(blk!=null) {
					blk.setVisible(true);
				}
				PShape txt = shapes.get("text_" + clicked.getName().substring(6) + "_menu");
				if(txt!=null) txt.setVisible(true);
			}

			// check if action has been defined for this element
			String act = actiondefs.get(clicked.getName().replace("block_", ""));

			if(act!=null) {

				// TODO: execute action

				if(act.startsWith("prolog")) {

					System.err.println("Executing Prolog query:");
					System.err.println("    " + act.substring(9));

				} else if (act.startsWith("java")) {

					System.err.println("Calling Java method:");
					System.err.println("    Class:  " + act.substring(7).split("#")[0]);
					System.err.println("    Method: " + act.substring(7).split("#")[1]);

				} else if (act.startsWith("service")) {
					
					System.err.println("Calling ROS service:");
					System.err.println("    service:  " + act.substring(10).split("#")[0]);
					System.err.println("    Args: " + act.substring(10).split("#")[1]);
				}

			}

		}
	}



	// 
	/**
	 * Recursively go through all children and check if 
	 * position is inside the respective shape.
	 * 
	 * @param p Current shape whose children are to be checked
	 * @param x Position to be checked
	 * @param y Position to be checked
	 * @return  First PShape that contains the (x,y) position 
	 */
	public PShape findClickedChild(PShape p, int x, int y) {

		//print(p.getName() + "\n");

		for(PShape c : p.getChildren()) {

			if(c.getName().equals("background")) {
				continue;
			}

			if(c.getFamily()==PShape.PRIMITIVE && contains(c, x, y)) {
				return c;

			} else {

				PShape cc = findClickedChild(c, x, y);
				if(cc!=null) return cc; 
			}
		}
		return null;  
	}


	/**
	 * Test if position is inside a rectangular shape
	 * 
	 * @param p Current shape to be checked
	 * @param x Position to be checked
	 * @param y Position to be checked
	 * @return True if position is inside
	 */
	public boolean contains(PShape p, int x, int y) {

		if(p.getFamily()==PShape.PRIMITIVE && p.isVisible()) {

			float[] par = p.getParams();
			float px = par[0] - bgX;
			float py = par[1] - bgY;
			float pw = par[2];
			float ph = par[3];

			if( (x>px) && (x<(px+pw)) &&
					(y>py) && (y<(py+ph)) ) {
				return true;
			}
		}
		return false;  
	}



	/**
	 * Read all sub-shapes into a map: name -> PShape
	 * 
	 * @param s
	 * @param shapes Map 
	 */
	private void readAllShapes(PShape s, Map<String, PShape> shapes) {

		for(PShape c : s.getChildren()) {
			if(!shapes.containsKey(c.getName()))
				shapes.put(c.getName(), c);

			readAllShapes(c, shapes);
		}

	}


	@SuppressWarnings("unchecked")
	private void loadActionDefs(String filename, Map<String, String> actiondefs) throws FileNotFoundException {

		InputStream input;
		input = new FileInputStream(new File(filename));

		Yaml yaml = new Yaml();
		try {
			Object mapping = (HashMap<String, List<String>>) yaml.load(input);
			actiondefs.putAll(((Map<String, Map<String, String>>) mapping).get("actiondefs"));	
		} catch (Exception e) {
			e.printStackTrace();
		}

	}



	public static void main(String args[]) {
		
		if(args.length < 1) {
			System.err.println("Usage: flowchart_vis <filename>.svg");
			return;
		}
		svg_file = args[0];
		
		PApplet.main(new String[] { "org.knowrob.vis.flowchart.FlowChartVis" });
	}
}
