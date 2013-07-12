package edu.tum.cs.ias.knowrob.mod_execution_trace;

// java staff
import java.io.*;
import java.io.File;
import java.io.StringReader;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.util.*;
import java.util.HashMap;
import java.util.StringTokenizer;
import java.util.Vector;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.w3c.dom.Node;
import org.w3c.dom.Element;
//import org.xml.sax.helpers.InputSource;

// owl stuff
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.OWLXMLOntologyFormat;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.io.SystemOutDocumentTarget;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
//import edu.tum.cs.ias.labeling.labels.readers.LabelsDatFileReader;
//import edu.tum.cs.ias.labeling.labels.readers.YamlConfigReader;

//ros stuff
import ros.*;
import ros.communication.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;

//knowrob stuff
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.owl.OWLIndividual;


/**
 * Create label file that describes the tasks in execution trace 
 * in terms of instances of the respective OWL classes.
 * 
 * The output is an ABOX representation of the action sequence
 * that can be loaded e.g. in the KnowRob knowledge base.
 * 
 * @author Asil Kaan Bozcuoglu, asil@cs.uni-bremen.de
 *
 */


public class ROSClient 
{
	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";
	public final static String EXECUTIONTRACE = "http://ias.cs.tum.edu/kb/executiontrace.owl#";
	public final static String MODEXECUTIONTRACE = "http://ias.cs.tum.edu/kb/modexecutiontrace.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for semantic map ontology	
	public final static String IAS_MAP = "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#";

	// ROS package name for KnowRob
	public final static String KNOWROB_PKG = "ias_knowledge_base";

	// OWL file of the KnowRob ontology (relative to KNOWROB_PKG)
	public final static String KNOWROB_OWL = "owl/knowrob.owl";

	// Namespace of OWL File
	public static String NAMESPACE = "http://ias.cs.tum.edu/kb/knowrob2.owl#";

	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(KNOWROB);
	static 
	{
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("executiontrace:", EXECUTIONTRACE);
		PREFIX_MANAGER.setPrefix("modexecutiontrace:", MODEXECUTIONTRACE);
		PREFIX_MANAGER.setPrefix("owl:",    OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}



        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n1;
	static double r;
        public ArrayList store;	


	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;
        OWLOntology ontology;

	ArrayList<Node> listOfAddedGoalContext;
        
	public ROSClient(String node_name) 
	{
		manager = OWLManager.createOWLOntologyManager();
		factory = manager.getOWLDataFactory();

	        initRos(node_name);
		store = new ArrayList();

		ontology = null;

		listOfAddedGoalContext = new ArrayList<Node>();
        }

        protected static void initRos(String node_name) 
	{

		ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n1 = ros.createNodeHandle();
		             
        }

        public void getTrace() 
	{
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
				
		DocumentBuilder dBuilder = null; // new DocumentBuilder();
		try
		{		
			dBuilder = dbFactory.newDocumentBuilder();
		}
		catch(javax.xml.parsers.ParserConfigurationException e)
		{
			System.out.println("Parsing failed!!!");
		}
		
		File inputFile = new File("/home/asil/Desktop/exec-trace.xml");
		//InputSource newSource = new InputSource(inputFile); 		

		Document doc = null; // new Document();
		try
		{
			doc = dBuilder.parse(inputFile);
 		}
		catch(Exception e)
		{
			System.out.println("Parsing failed!!!");
		}

		doc.getDocumentElement().normalize();
		
		// OWL PART
		try
		{
			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();
	
			// Get prefix manager using the base IRI as default namespace
			pm = PREFIX_MANAGER;

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(NAMESPACE));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create("knowrob"));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			// Action instances 
			OWLNamedIndividual action_inst = null;
			OWLNamedIndividual prev_action_inst = null;

			//System.out.println("Root element :" + doc.getDocumentElement().getNodeName());
 			//NodeList nList = doc.getElementsByTagName("goal-context");
			//NodeList nList = doc.getDocumentElement();

			NodeList nList = doc.getElementsByTagName("task");

			for(int i = 1; i < nList.getLength(); i++)
			{
				Node current = nList.item(i);
				Node directParent = current.getParentNode();				
				Node currentDummy = current;

				while(currentDummy.getParentNode() != null)
				{
					currentDummy = currentDummy.getParentNode();
					checkAndAddNewGoalContext(currentDummy);
				}

				OWLNamedIndividual goal_context_inst = factory.getOWLNamedIndividual("executiontrace:" + directParent.getAttributes().getNamedItem("name").getNodeValue().replaceAll(" ", "_").toLowerCase(), pm);

				OWLNamedIndividual task_inst = factory.getOWLNamedIndividual("executiontrace:" + current.getAttributes().getNamedItem("name").getNodeValue().replaceAll(" ", "_").toLowerCase(), pm);
				OWLClass task_class = factory.getOWLClass("modexecutiontrace:Task", pm);
				manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(task_class, task_inst));

				OWLObjectProperty goalContextOfTask = factory.getOWLObjectProperty("modexecutiontrace:TaskGoalContext", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(goalContextOfTask, task_inst, goal_context_inst));

				NodeList taskChildNodes = current.getChildNodes();

				String created, running, failed, succeeded;

				for(int x = 0; x < nList.getLength(); x++)
				{				
					if(taskChildNodes != null && taskChildNodes.item(x) != null && taskChildNodes.item(x).getAttributes().getNamedItem("name") != null && 
						taskChildNodes.item(x).getAttributes().getNamedItem("name").getNodeValue().equals("CREATED"))
					{
						created = taskChildNodes.item(x).getTextContent();
						System.out.println(created);
					}
					else if(taskChildNodes != null && taskChildNodes.item(x) != null && taskChildNodes.item(x).getAttributes().getNamedItem("name") != null &&
						taskChildNodes.item(x).getAttributes().getNamedItem("name").getNodeValue().equals("RUNNING"))
					{
						running = taskChildNodes.item(x).getTextContent();
						System.out.println(running);
					}
					else if(taskChildNodes != null && taskChildNodes.item(x) != null && taskChildNodes.item(x).getAttributes().getNamedItem("name") != null && 
						taskChildNodes.item(x).getAttributes().getNamedItem("name").getNodeValue().equals("FAILED"))
					{
						failed = taskChildNodes.item(x).getTextContent();
						System.out.println(failed);
					}
					else if(taskChildNodes != null && taskChildNodes.item(x) != null && taskChildNodes.item(x).getAttributes().getNamedItem("name") != null &&
						taskChildNodes.item(x).getAttributes().getNamedItem("name").getNodeValue().equals("SUCCEEDED"))
					{
						succeeded = taskChildNodes.item(x).getTextContent();
						System.out.println(succeeded);
					}
		
				}
				

			}

			File output = new File("/home/asil/Desktop/executiontrace.owl");
			IRI documentIRI2 = IRI.create(output);
			manager.saveOntology(ontology, new RDFXMLOntologyFormat(), documentIRI2);
//			manager.saveOntology(ontology, new SystemOutDocumentTarget());		
		}
		catch (Exception e) 
		{
			e.printStackTrace();
		}		
	}

	void checkAndAddNewGoalContext(Node currentDummy)
	{
		boolean doesExist = false;
		for(int i = 0; i < listOfAddedGoalContext.size(); i++)
		{
			//System.out.println("Abc: " + currentDummy.getNodeName());

			if(!(currentDummy.getNodeName().equals("#document")) && listOfAddedGoalContext.get(i).getAttributes().getNamedItem("name").getNodeValue().replaceAll(" ", "_").equals(currentDummy.getAttributes().getNamedItem("name").getNodeValue().replaceAll(" ", "_")))
			{
				//System.out.println(currentDummy.getAttributes().getNamedItem("name").getNodeValue().replaceAll(" ", "_"));
				doesExist = true;
				break;
			}
			else if (currentDummy.getNodeName().equals("#document"))
				doesExist = true;
		}

		if(!doesExist)
		{
			listOfAddedGoalContext.add(currentDummy);

			OWLNamedIndividual goal_context_inst = factory.getOWLNamedIndividual("executiontrace:" + currentDummy.getAttributes().getNamedItem("name").getNodeValue().toLowerCase().replaceAll(" ", "_"), pm);
			OWLClass goal_context_class = factory.getOWLClass("modexecutiontrace:GoalContext", pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(goal_context_class, goal_context_inst));
		}
	}
       
        public static void main(String[] args) 
        {

                ROSClient d = new ROSClient("knowrob_execution_trace_test_123");
		int i = 1;
		while (i == 1)
		{
			/*Reply answer = */ d.getTrace();
			//String res = answer.result;
                	//System.out.println(x);
			i++;
                }
        }

}

