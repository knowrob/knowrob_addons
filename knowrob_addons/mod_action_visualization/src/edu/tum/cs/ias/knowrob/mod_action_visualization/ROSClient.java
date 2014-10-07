package edu.tum.cs.ias.knowrob.mod_action_visualization;

import java.io.*;
import java.util.ArrayList;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.io.SystemOutDocumentTarget;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import edu.tum.cs.ias.knowrob.owl.OWLIndividual;

//java stuff
import java.io.* ;
import java.io.File;
import java.io.StringReader;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.util.*;
import jpl.*;
import jpl.Query;                // empirically, we need this, but I don't know why...
import jpl.PrologException;
import jpl.fli.Prolog;
import jpl.*;
import java.util.Hashtable;

//ros stuff
import ros.*;
import ros.communication.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;
//import ros.pkg.pr2_msgs.msg.*;

//knowrob stuff
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;

/** 
 * @author Asil Kaan Bozcuoglu, asil@cs.uni-bremen.de
 *
 */

public class ROSClient 
{
	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//
	
	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://knowrob.org/kb/knowrob.owl#";
	public final static String TRACK = "http://knowrob.org/kb/track.owl#";
	
	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";
	
	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";
	
	// Base IRI for semantic map ontology	
	public final static String IAS_MAP = "http://knowrob.org/kb/ias_semantic_map.owl#";
	
	// ROS package name for KnowRob
	public final static String KNOWROB_PKG = "ias_knowledge_base";
	
	// OWL file of the KnowRob ontology (relative to KNOWROB_PKG)
	public final static String KNOWROB_OWL = "owl/knowrob.owl";
	
	// Namespace of OWL File
	public static String NAMESPACE = "http://knowrob.org/kb/knowrob2.owl#";
	
	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(KNOWROB);
	static 
	{
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("track:", TRACK);
		PREFIX_MANAGER.setPrefix("owl:",    OWL);	
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}
	
	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n1;
	static double r;
        public ArrayList store;	
        
        public ROSClient(String node_name) 
	{
	        initRos(node_name);
		store = new ArrayList();

		PrologInterface.initJPLProlog("mod_vis");
		PrologInterface.executeQuery("register_ros_package('mod_execution_trace')");

		manager = OWLManager.createOWLOntologyManager();
		factory = manager.getOWLDataFactory();		
 
        }

        protected static void initRos(String node_name) 
	{

		ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n1 = ros.createNodeHandle();
		             
        }

	public void writeToFile(int objectNumbers,
			String path, String matrixFilePrefix, String indexFilePrefix, int startframe, int endframe) throws FileNotFoundException, IOException
	{
		OWLOntology ontology = null;
		try 
		{
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();

			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(NAMESPACE));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create("knowrob"));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);
			

			OWLClass map_class = factory.getOWLClass("knowrob:SemanticEnvironmentMap", pm);
			OWLNamedIndividual map_inst = factory.getOWLNamedIndividual("track:SemanticEnvironmentMap0", pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(map_class, map_inst));
			
			double currentMatrix[][] = new double[4][4];
			
			for(int x = startframe; x <= endframe; x++)
			{
				File mFile  = new File(path + matrixFilePrefix + x + ".txt");
				BufferedReader reader = new BufferedReader(new FileReader(mFile));
				
				File iFile  = new File(path + indexFilePrefix + x + ".txt");
				BufferedReader iReader = new BufferedReader(new FileReader(iFile));
				
				ArrayList<java.lang.Integer> listOfIndices = new ArrayList<java.lang.Integer>();
				
				String s =iReader.readLine();

				while(s!=null)
				{
				    listOfIndices.add(new java.lang.Integer(java.lang.Integer.parseInt(s))); 
				    s = iReader.readLine();
				}
				
				for(int l = 0; l < listOfIndices.size(); l = l + 4)
				{
					if(l != 0 && listOfIndices.get(l) == listOfIndices.get(l - 4))
						continue;
					
					if(listOfIndices.get(l) != 2)
						continue;
					
					String prefix = "";
					
					if(x / 1000 == 0)
						prefix = prefix.concat("0");
					if(x / 100 == 0)
						prefix = prefix.concat("0");
					if(x / 10 == 0)
						prefix = prefix.concat("0");
					
					//add object instance
					OWLClass object_class = factory.getOWLClass("knowrob:Door", pm);
					OWLNamedIndividual object_inst = factory.getOWLNamedIndividual("track:object" + listOfIndices.get(l) + "_" + prefix + x, pm);
					
					OWLDatatype doubleDatatype2 = factory.getDoubleOWLDatatype();
					OWLLiteral literal0 = factory.getOWLLiteral(0.01);
					OWLDataProperty m001_ = factory.getOWLDataProperty(":depthOfObject", pm);
			        OWLDataPropertyRangeAxiom rangeAxiom0 = factory.getOWLDataPropertyRangeAxiom(
			                m001_, doubleDatatype2);
					
					OWLDataPropertyExpression m001 = rangeAxiom0.getProperty(); 
					manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(m001, object_inst, literal0));
					
					OWLLiteral literal1 = factory.getOWLLiteral(0.01);
					OWLDataProperty m01_ = factory.getOWLDataProperty(":heightOfObject", pm);
			        OWLDataPropertyRangeAxiom rangeAxiom1 = factory.getOWLDataPropertyRangeAxiom(
			                m01_, doubleDatatype2);
					
					OWLDataPropertyExpression m01 = rangeAxiom1.getProperty(); 
					manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(m01, object_inst, literal1));
					
					OWLLiteral literal2 = factory.getOWLLiteral(0.01);
					OWLDataProperty m02_ = factory.getOWLDataProperty(":widthOfObject", pm);
			        OWLDataPropertyRangeAxiom rangeAxiom2 = factory.getOWLDataPropertyRangeAxiom(
			                m02_, doubleDatatype2);
					
					OWLDataPropertyExpression m02 = rangeAxiom2.getProperty(); 
					manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(m02, object_inst, literal2));
					
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(object_class, object_inst));
					
					// add to semantic map
					OWLObjectProperty semantic_m = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(semantic_m,  object_inst, map_inst));
					
					//add timing instance
					OWLNamedIndividual timestamp1 = factory.getOWLNamedIndividual("track:timepoint_" + x, pm);
					OWLClass time_class = factory.getOWLClass("knowrob:TimePoint", pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(time_class, timestamp1));
					
					//load matrix elements
					for(int i = 0; i < 4; i++)
						for(int j = 0; j < 4; j++)
						{
							try
							{
								if(i == 3 && j != 3)
									currentMatrix[j][i] = Double.parseDouble(reader.readLine()) / 100;
								else
									currentMatrix[j][i] = Double.parseDouble(reader.readLine());
							}
							catch (NumberFormatException e)
							{
								currentMatrix[j][i] = 0;
							}
						}
					
					// add matrix instance
					OWLIndividual matrix_inst_ind =  OWLIndividual.getOWLIndividualOfClass("RotationMatrix3D");
					OWLNamedIndividual matrix_inst = factory.getOWLNamedIndividual(matrix_inst_ind.getLabel(), pm);
					OWLClass matrix_class = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
					
					//insert matrix elements
					for(int i = 0; i < 4; i++)
						for(int j = 0; j < 4; j++)
						{
					
							OWLDatatype doubleDatatype = factory.getDoubleOWLDatatype();
							OWLLiteral literal = factory.getOWLLiteral(currentMatrix[i][j]);
							OWLDataProperty m00_ = factory.getOWLDataProperty(":m"+i+j, pm);
					        OWLDataPropertyRangeAxiom rangeAxiom = factory.getOWLDataPropertyRangeAxiom(
					                m00_, doubleDatatype);
							
							OWLDataPropertyExpression m00 = rangeAxiom.getProperty(); 
							manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(matrix_class, matrix_inst));
							manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(m00, matrix_inst, literal));
						}
					
					//insert perception instance
					OWLIndividual perception_inst_ind =  OWLIndividual.getOWLIndividualOfClass("SemanticMapPerception");
					OWLNamedIndividual perception_inst = factory.getOWLNamedIndividual(perception_inst_ind.getLabel(), pm);
					OWLClass perception_class = factory.getOWLClass("knowrob:SemanticMapPerception", pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(perception_class, perception_inst));
					
					// add time relation to perception instance
					OWLObjectProperty startTime = factory.getOWLObjectProperty("knowrob:startTime", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(startTime,  perception_inst, timestamp1));
					
					// add object information to perception instance
					OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objectActedOn, perception_inst, object_inst));
					
					// add pose information to perception instance
					OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(eventOccursAt, perception_inst, matrix_inst));
				
				
				}
			}
			File output = new File("/home/asil/Desktop/deneme/deneme1.owl");
			IRI documentIRI2 = IRI.create(output);
			manager.saveOntology(ontology, new RDFXMLOntologyFormat(), documentIRI2);
			manager.saveOntology(ontology, new SystemOutDocumentTarget());
		}
		catch (Exception e) 
		{
			e.printStackTrace();
		}
		
	}

        public void getVisual(int x, int y) 
	{
		try 
		{
			this.writeToFile(5,
					"/home/asil/Desktop/shellgame/", "deneme", 
					"indices", x, y);


		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		}
	}

}
