package org.knowrob.vis.flowchart;

import instruction.gui.EHowInstructionPanel;

import java.awt.Color;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.HashMap;
import java.util.Map;

import processing.core.PApplet;
import processing.core.PShape;
import ros.NodeHandle;
import ros.Ros;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

import javax.swing.JFrame;

import org.yaml.snakeyaml.Yaml;

import edu.tum.cs.ias.knowrob.json_prolog.Prolog;


/**
 * Load and Display a Shape. 
 * Illustration by George Brower. 
 * 
 * The loadShape() command is used to read simple SVG (Scalable Vector Graphics)
 * files and OBJ (Object) files into a Processing sketch. This example loads an
 * SVG file of a monster robot face and displays it to the screen. 
 */

public class FlowChartVis extends PApplet {

	private static final long serialVersionUID = -1591411154652437400L;
	PShape chart;
	PShape bg;

	Map<String, PShape> shapes;
	Map<String, String> actiondefs;

	int bgX = 0;
	int bgY = 0;

	static String svg_file = "";

	static Ros ros;
	public static NodeHandle n;

	public static JFrame ehow_window;
	public static EHowInstructionPanel ehow_panel;

	public void setup() {

		size(1100, 600);

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
					System.err.println(act.substring(9));

					sendPrologQuery(act.substring(9));


				} else if (act.startsWith("java")) {

					System.err.println("Calling Java method:");
					System.err.println("    Class:  " + act.substring(7).split("#")[0]);
					System.err.println("    Method: " + act.substring(7).split("#")[1]);
					

					callJavaMethod(act.substring(7).split("#")[0], act.substring(7).split("#")[1]);

				} else if (act.startsWith("service")) {

					callROSService(act.substring(10).split("#")[0], 
							act.substring(10).split("#")[1].split("&")[0]);

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


	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// execute actions

	/**
	 * Send a Prolog query via the json_prolog ROS service
	 * 
	 * @param query Query to be sent (without trailing '.')
	 */
	public void sendPrologQuery(String query) {

		Prolog pl = new Prolog();
		pl.query(query);

	}


	/**
	 * Call one out of a predefined set of ROS services
	 * 
	 * @param service Service name
	 * @param arg Optional string argument
	 */
	public void callROSService(String service, String arg) {

		// TODO: implement code for calling the different kinds of services 
		
//		if(service.equals("comm_vis_set_left_img")) {
//
//			ServiceClient<CommVisSetLeftImg.Request, CommVisSetLeftImg.Response, CommVisSetLeftImg> sc =
//					n.serviceClient("add_two_ints" , new CommVisSetLeftImg(), false);
//
//			CommVisSetLeftImg.Request rq = new CommVisSetLeftImg.Request();
//			sc.call(rq);
//			
//			sc.shutdown();
//
//		}


	}

	/**
	 * Support for calling a static method without arguments
	 * 
	 * @param cls Fully qualified class name
	 * @param method Method name inside that class
	 */
	public void callJavaMethod(String cls, String method) {

		try {
			Class<?> clz = Class.forName(cls);
			
			for(Method m : clz.getMethods()) {
				
				if(m.getName().equals(method)) {
					m.invoke(clz, (Object[]) null);
				}
			}
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			e.printStackTrace();
		}
		
	}
	
	public static void importEhowInstruction() {
		
    // setup and pre-initialize the ehow window  	
	  	ehow_window = new JFrame();
	  	ehow_window.setVisible( false );
	  	ehow_window.setSize( 1100, 800 );
	  	ehow_window.setTitle( "Plan Importer GUI" );
	  	ehow_window.setLocation( 400, 300 );
	  	ehow_window.setBackground(new Color(20, 20, 20));
	  	
	  	ehow_window.addWindowListener( new WindowAdapter() {
	  		public void windowClosing( WindowEvent we ) {
	    	//System.exit( 0 );
	  			ehow_window.setVisible(false);
	    	} } );

	  	ehow_panel = new EHowInstructionPanel();
	  	ehow_window.add( ehow_panel );
	  	ehow_panel.init();
	  	ehow_panel.revalidate();
	  	ehow_panel.setVisible(true);
	
	}
	
	
	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_flowchart_vis");
		}
		n = ros.createNodeHandle();

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
