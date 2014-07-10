package edu.tum.cs.ias.knowrob.map;

import java.io.*;
import java.util.Date;
import java.util.ArrayList;
import java.util.HashMap;
import java.text.SimpleDateFormat;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.vocab.PrefixOWLOntologyFormat;
import ros.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;




//
//
// export trajectory points to OWL
//
//



public class PosesToOWL {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//
	
	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";
	
	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for semantic map ontology	
	public final static String IAS_MAP = "http://ias.cs.tum.edu/kb/ccrl2_semantic_map_addons.owl#";
	
	
	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(IAS_MAP);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("ias_map:", IAS_MAP);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}
	
	// mapping ROS-KnowRob identifiers
	protected static final HashMap<String, String> rosToKnowrob = new HashMap<String, String>();
	static {
		rosToKnowrob.put("cupboard",     "knowrob:Cupboard");
		rosToKnowrob.put("drawer",       "knowrob:Drawer");
		rosToKnowrob.put("oven",         "knowrob:Oven");
		rosToKnowrob.put("refrigerator", "knowrob:Refrigerator");
		rosToKnowrob.put("dishwasher",    "knowrob:Dishwasher");
		rosToKnowrob.put("table",        "knowrob:Table");
		rosToKnowrob.put("countertop",   "knowrob:CounterTop");
		rosToKnowrob.put("sink",         "knowrob:Sink");
		rosToKnowrob.put("door",         "knowrob:Door");
		rosToKnowrob.put("hinge",        "knowrob:HingedJoint");
		rosToKnowrob.put("handle",       "knowrob:Handle");
		rosToKnowrob.put("knob",         "knowrob:ControlKnob");
		
		// TODO: add a concept for horizontal planes to knowrob
		rosToKnowrob.put("horizontal_plane", "knowrob:CounterTop");
	}
	
	HashMap<String, ArrayList<OWLNamedIndividual>> pointsInTrajectories; 
	
	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;
	
	int inst_counter=150;	// counter to create unique instance identifiers
	
	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff
	
	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	
	////////////////////////////////////////////////////////////////////////////////

		
	/**
	 * Initialize the ROS environment if it has not yet been initialized
	 * 
	 * @param node_name A unique node name
	 */
	protected static void initRos(String node_name) {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init(node_name);
		}
		n = ros.createNodeHandle();

	}

	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public PosesToOWL() {

		pointsInTrajectories = new HashMap<String, ArrayList<OWLNamedIndividual>>();
	
		initRos("bla");
		
	}
	
	
	/**
	 * 
	 * @param ros_map
	 * @return
	 */
	public OWLOntology buildOWLDescription(String filename) {

		OWLOntology ontology = null;
		pointsInTrajectories.clear();

		
	    
	    try{
	    	
	    	BufferedReader reader = new BufferedReader(new FileReader(filename));
	    	
			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();
			
			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;
			
			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(IAS_MAP));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());
			
			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);
			
			

			// create time point 
			OWLNamedIndividual time_inst = createTimePointInst(ros.now(), ontology);
			

			
			// iterate over all objects and create the respective OWL representations
			String line;
			String objname="";
			
			while(true) {
			        
		        line = reader.readLine();
		        if(line==null){break;}
		        if(line.equals("")){continue;}
		        
	
		        // new object: line is object name
		        if(line.matches("[a-zA-Z].*")) {
		          
		        	objname=line;
		        	this.pointsInTrajectories.put(objname, new ArrayList<OWLNamedIndividual>());
		        	continue;
		        	
		        	
		        } else { 
		          
		        	String[] qpose = line.split(",");
		        	Point p = new Point();
		        	p.x = Float.valueOf(qpose[0]);
		        	p.y = Float.valueOf(qpose[1]);
		        	p.z = Float.valueOf(qpose[2]);
		        	
		        	Quaternion q = new Quaternion();
		        	q.x = Float.valueOf(qpose[3]);
		        	q.y = Float.valueOf(qpose[4]);
		        	q.z = Float.valueOf(qpose[5]);
		        	q.w = Float.valueOf(qpose[6]);
		        	
		        	double[] matrix = quaternionToMatrix(p,q);
		        	createPointDescription(objname, matrix, time_inst, ontology);
		        	
		        	continue;
		        }
		        
			}

			for(String obj : pointsInTrajectories.keySet()) {
				
				OWLIndividual owl_traj = createTrajInst(ontology);
				
				OWLNamedIndividual owl_obj = factory.getOWLNamedIndividual("knowrob:"+obj, pm);
				OWLObjectProperty prop = factory.getOWLObjectProperty("knowrob:openingTrajectory", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(prop,  owl_obj, owl_traj));

				
				prop = factory.getOWLObjectProperty("knowrob:pointOnTrajectory", pm);
				
				for(OWLIndividual point : pointsInTrajectories.get(obj)) {
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(prop,  owl_traj, point));
				}
				
			}
			
			
			
			SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
			String outfile = "ias_semantic_map_"+sdf.format(new Date())+".owl";
			saveOntologyToFile(ontology, outfile);

	      
		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}
		
		return ontology;
	}
	
	
	
	

	
	protected void createPointDescription(String objname, double[] matrix, OWLNamedIndividual timestamp, OWLOntology ontology) {
				
		// create object instance
		OWLNamedIndividual obj_inst = createPointInst(ontology);

		// create pose matrix instance
		OWLNamedIndividual pose_inst = createPoseInst(matrix, ontology);
		
		// create perception instance
		createPerceptionInst(obj_inst, pose_inst, timestamp, ontology);
		
		// remember mapping of ROS object ID and OWL instance (for being able to link to parents)
		this.pointsInTrajectories.get(objname).add(obj_inst);
				
	}
	


	protected OWLNamedIndividual createTimePointInst(ros.communication.Time stamp, OWLOntology ontology) {
		
		OWLNamedIndividual time_inst = factory.getOWLNamedIndividual("ias_map:timepoint_"+stamp.secs, pm);
		OWLClass time_class = factory.getOWLClass("knowrob:TimePoint", pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(time_class, time_inst));
		
		return time_inst;
	}
	
	

	protected OWLNamedIndividual createPointInst(OWLOntology ontology) {
		
		// create object instance
		OWLClass obj_class = factory.getOWLClass("knowrob:Point3D", pm);
		OWLNamedIndividual obj_inst = factory.getOWLNamedIndividual(
										      instForClass("knowrob:Point3D"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(obj_class, obj_inst));
		
		return obj_inst;
	}
	

	protected OWLNamedIndividual createTrajInst(OWLOntology ontology) {
		
		// create object instance
		OWLClass traj_class = factory.getOWLClass("knowrob:ArmTrajectory", pm);
		OWLNamedIndividual traj_inst = factory.getOWLNamedIndividual(
										      instForClass("knowrob:ArmTrajectory"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(traj_class, traj_inst));
		
		return traj_inst;
	}
	
	

	protected OWLNamedIndividual createPoseInst(double[] ros_obj, OWLOntology ontology) { 
	
		// create pose matrix instance
		OWLClass pose_class = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
		OWLNamedIndividual pose_inst = factory.getOWLNamedIndividual(
										      instForClass("ias_map:RotationMatrix3D"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(pose_class, pose_inst));
		
		// set pose properties
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {
				
				System.out.println("m"+i+j+"="+ros_obj[4*i+j]);
				
				OWLDataProperty prop = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
				manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  pose_inst, ros_obj[4*i+j]));
			}
		}
		
		return pose_inst;
	}	

	
	

	protected OWLNamedIndividual createPerceptionInst(OWLNamedIndividual obj_inst, OWLNamedIndividual pose_inst, OWLNamedIndividual timestamp, OWLOntology ontology) {
		
		// create perception instance
		OWLClass perc_class = factory.getOWLClass("knowrob:TouchPerception", pm);
		OWLNamedIndividual perc_inst = factory.getOWLNamedIndividual(
										      instForClass("knowrob:TouchPerception"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(perc_class, perc_inst));
		
		// link to the object instance and the pose instance
		OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
		OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);

		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objectActedOn, perc_inst, obj_inst));
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(eventOccursAt, perc_inst, pose_inst));
		
		// set time stamp
		OWLObjectProperty startTime = factory.getOWLObjectProperty("knowrob:startTime", pm);
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(startTime,  perc_inst, timestamp));
		
		return perc_inst;
	}
	
	
	

	
	
	
	
	protected String instForClass(String cl) {
		cl.replaceAll("knowrob", "ias_map");
		return cl+(inst_counter++);
	}
	

	protected static double[] quaternionToMatrix(Point p, Quaternion q) {

		double[] m = new double[16];
		
	    double xx = q.x * q.x;
	    double xy = q.x * q.y;
	    double xz = q.x * q.z;
	    double xw = q.x * q.w;

	    double yy = q.y * q.y;
	    double yz = q.y * q.z;
	    double yw = q.y * q.w;

	    double zz = q.z * q.z;
	    double zw = q.z * q.w;

	    m[0]  = 1 - 2 * ( yy + zz );
	    m[1]  =     2 * ( xy - zw );
	    m[2]  =     2 * ( xz + yw );
	    m[3]  = p.x;
	    
	    m[4]  =     2 * ( xy + zw );
	    m[5]  = 1 - 2 * ( xx + zz );
	    m[6]  =     2 * ( yz - xw );
	    m[7]  = p.y;
	    
	    m[8]  =     2 * ( xz - yw );
	    m[9]  =     2 * ( yz + xw );
	    m[10] = 1 - 2 * ( xx + yy );
	    m[11]=p.z;
	    
	    m[12]=0;
	    m[13]=0;
	    m[14]=0;
	    m[15]=1;
	    return m;
	}
	
	
	
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// TODO: copied from RoboEarth -- find a nicer solution, e.g. with a dedicated OWL library package 
//

	/**
	 * Saves an OWL ontology to an output stream in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param stream output stream
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToStream(OWLOntology ontology, OutputStream stream, OWLOntologyFormat format) {
		
		boolean ok = false;
		
		if (stream != null) {

			try {
				
				// Get hold of the ontology manager
				OWLOntologyManager manager = ontology.getOWLOntologyManager();

				// By default ontologies are saved in the format from which they were loaded.
				// We can get information about the format of an ontology from its manager
				OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

				if (currFormat.equals(format)) {
					
					// Save a copy of the ontology to the given stream.
					manager.saveOntology(ontology, stream);
					
				} else {

					// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
					// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
					// the new ontology document
					if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
						((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
					}
					manager.saveOntology(ontology, format, stream);
					
				}

				ok = true;
				
			} catch (Exception e) {
				System.out.println("Could not save ontology: " + e.getMessage());
			}
			
		}
		
		return ok;
	}
	
	
	/**
	 * Saves an OWL ontology to a String object.
	 * @param ontology ontology to be saved
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return String object containing the string representation of the ontology
	 */
	public static String saveOntologytoString(OWLOntology ontology, OWLOntologyFormat format) {
		
		String s = null;
		ByteArrayOutputStream os = new ByteArrayOutputStream(4096);
		
		if (saveOntologyToStream(ontology, os, format)) {
			try {
				s = new String(os.toByteArray(), "UTF-8");
			} catch (UnsupportedEncodingException e) {
				System.out.println("UTF-8 encoding is unsupported: " + e.getMessage());
			}			
		}

		return s;
		
	}
	
	

	/**
	 * Saves an OWL ontology to a file.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file) {

		boolean ok = false;

		try {
			OWLOntologyFormat format = ontology.getOWLOntologyManager().getOntologyFormat(ontology);
			ok = saveOntologyToFile(ontology, file, format);			
		} catch (NullPointerException e) {
			System.out.println("Could not save ontology: null pointer argument found\n" + e.getMessage());
		}
		
		return ok; 
		
	}

	
	/**
	 * Saves an OWL ontology to a file in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file, OWLOntologyFormat format) {
		
		boolean ok = false;
		
		try {
			
			// Build File object
			File f = new File(file);
			
			// Get hold of the ontology manager
			OWLOntologyManager manager = ontology.getOWLOntologyManager();

			// By default ontologies are saved in the format from which they were loaded.
			// We can get information about the format of an ontology from its manager
			OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

			// The Document IRI, where the file should be saved
			IRI documentIRI = IRI.create(f.toURI());
			
			if (currFormat.equals(format)) {
				
				// Save a local copy of the ontology.
				manager.saveOntology(ontology, documentIRI);
				
			} else {

				// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
				// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
				// the new ontology document
				if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
					((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
				}
				manager.saveOntology(ontology, format, documentIRI);
				
			}

			ok = true;
			
		} catch (Exception e) {
			System.out.println("Could not save ontology: " + e.getMessage());
		}
		
		return ok;
	}

	
	
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	
	public static void main(String[] args) {

		PosesToOWL p2o = new PosesToOWL();
		p2o.buildOWLDescription("poses.txt");
		
	}

	
}
