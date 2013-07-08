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
import ros.pkg.pr2_msgs.msg.*;
import ros.pkg.cram_et_prolog.msg.*;
import ros.pkg.cram_et_prolog.srv.*;

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


public class ROSClient2 
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
        
	public ROSClient2(String node_name) 
	{
		manager = OWLManager.createOWLOntologyManager();
		factory = manager.getOWLDataFactory();

	        initRos(node_name);
		store = new ArrayList();

		ontology = null;
        }

        protected static void initRos(String node_name) 
	{

		ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n1 = ros.createNodeHandle();
		             
        }

        public String[] getTrace() 
	{
		Reply result=null;
		try 
                {

			ServiceClient<QueryExecutionTrace.Request, QueryExecutionTrace.Response, QueryExecutionTrace> sc =
                                n1.serviceClient("/query_execution_trace", new QueryExecutionTrace());

                        QueryExecutionTrace.Request req = new QueryExecutionTrace.Request();
			req.filename = "/home/asil/Desktop/traces-4/exectrace3";
			req.request = new Req();
			req.request.predicate = "(task ?tsk)";			
                        result = sc.call(req).response;
                        sc.shutdown();

                } 
                catch (RosException e) 
                {
			ros.logError("ROSClient: Call to service failed");
                }
		
		String parseString = result.result.substring(5);
		StringTokenizer stringTokenizer = new StringTokenizer(parseString, "#");
		
		int tokenCount = stringTokenizer.countTokens();
		String[] interimElements = new String[tokenCount];
		String[][][] allElements = new String[tokenCount][][];
		int i = 0;

		while (stringTokenizer.hasMoreElements()) 
		{
			interimElements[i] = stringTokenizer.nextElement().toString();
			i++;
		}

		for(int x = 1; x < tokenCount; x++)
		{		
			int type = checkType(interimElements[x]);

			ArrayList<String[]> dataPrologReady = convertToPrologReady(type, interimElements[x]);

			if(dataPrologReady != null)
			{
				allElements[x] = new String[dataPrologReady.size()][];

				for(int y = 0; y < dataPrologReady.size(); y++)
				{
					allElements[x][y] = new String[dataPrologReady.get(y).length];
					String[] curr = dataPrologReady.get(y);
										

					for(int z = 0; z < dataPrologReady.get(y).length; z++)
					{
						allElements[x][y][z] = curr[z];
	
					}
				}

			}
 			
		}

		int length = 0;

		for (int x = 1; x < allElements.length; x++)
		{
			if( allElements[x] != null)
			{
				for(int y = 0; y < allElements[x].length; y++)
				{
					length += allElements[x][y].length;
				}
			}
		}

		String[] allElementsOneDimensional = new String[length];
		int count = 0;
		for (int x = 1; x < allElements.length; x++)
		{
			if( allElements[x] != null)
			{
				for(int y = 0; y < allElements[x].length; y++)
				{
					for(int z = 0; z < allElements[x][y].length; z++)
					{
						allElementsOneDimensional[count] = allElements[x][y][z];
						count++;
					}
				}
			}
		}



                // OWL PART
		try
		{
			// Create ontology manager and data factory
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

			//Action instances 
			OWLNamedIndividual action_inst = null;
			OWLNamedIndividual prev_action_inst = null;


			count = 0;
			boolean isNewTask = true;
	
			String task = "Xxx";
			//OWLNamedIndividual action_inst = null;

			for(int x = 0; x < allElementsOneDimensional.length; x++)
			{
				String current = allElementsOneDimensional[x];

				if(isNewTask)
				{
					task = "task_" + count;
				
					action_inst = factory.getOWLNamedIndividual("executiontrace:" + task.toLowerCase(), pm);
					OWLClass action_class = factory.getOWLClass("modexecutiontrace:task", pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(action_class, action_inst));	
					
					isNewTask = false;
					count++;
				}
			
				if(current.equals("GOAL"))
				{
					String prefix = "goal_";
					prefix = prefix.concat(allElementsOneDimensional[x+1].toLowerCase());
					prefix = prefix.concat("_");
					prefix = prefix.concat(allElementsOneDimensional[x+2].toLowerCase());
					prefix = prefix.concat("_");
					prefix = prefix.concat(allElementsOneDimensional[x+3].toLowerCase());
					prefix = prefix.replace("-", "_");
					prefix = prefix.replace(":", "_");
					prefix = prefix.replace("?", "");

					String name = prefix + count;

					OWLNamedIndividual goal_inst = factory.getOWLNamedIndividual("executiontrace:" + name.toLowerCase(), pm);
					OWLClass goal_class = factory.getOWLClass("modexecutiontrace:" + prefix.toLowerCase(), pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(goal_class, goal_inst));
	
					OWLObjectProperty goalOfTask = factory.getOWLObjectProperty("modexecutiontrace:taskgoal", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(goalOfTask, action_inst, goal_inst));

					x = x + 3;
				}
				else if(current.equals("GOAL-CONTEXT"))
				{
					String prefix = "goalcontext_";
					prefix = prefix.concat(allElementsOneDimensional[x+1].toLowerCase());
					if(!(allElementsOneDimensional[x+1].equals("'WITH-DESIGNATORS")))					
					{
						prefix = prefix.concat("_");
						prefix = prefix.concat(allElementsOneDimensional[x+2].toLowerCase());
					}
					prefix = prefix.replace("-", "_");
					prefix = prefix.replace(":", "_");
					prefix = prefix.replace("?", "");
					prefix = prefix.replace("'", "");
					
					// System.out.println(prefix);	

					String name = prefix + count;

					OWLNamedIndividual goal_context_inst = factory.getOWLNamedIndividual("executiontrace:" + name.toLowerCase(), pm);
					OWLClass goal_context_class = factory.getOWLClass("modexecutiontrace:" + prefix.toLowerCase(), pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(goal_context_class, goal_context_inst));

					OWLObjectProperty goalContextOfTask = factory.getOWLObjectProperty("modexecutiontrace:taskgoalcontext", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(goalContextOfTask, action_inst, goal_context_inst));				

					if(!(allElementsOneDimensional[x+1].equals("'WITH-DESIGNATORS")))					
						x = x + 2;
					else x = x + 1;
				
				}
				else if(current.equals("TOP-LEVEL"))
				{
					String prefix = "toplevel_";
					prefix = prefix.concat(allElementsOneDimensional[x+1]);
					prefix = prefix.replace("-", "_");
					prefix = prefix.replace(":", "_");
					prefix = prefix.replace("?", "");

					String name = prefix + count;

					OWLNamedIndividual toplevel_inst = factory.getOWLNamedIndividual("executiontrace:" + name.toLowerCase(), pm);
					OWLClass toplevel_class = factory.getOWLClass("modexecutiontrace:" + prefix.toLowerCase(), pm);
					manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(toplevel_class, toplevel_inst));

					OWLObjectProperty topLevelOfTask = factory.getOWLObjectProperty("modexecutiontrace:tasktoplevel", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(topLevelOfTask, action_inst, toplevel_inst ));


					isNewTask = true;
					x = x + 1;

				}		

			
			
			}

			File output = new File("/home/asil/Desktop/plans.owl");
			IRI documentIRI2 = IRI.create(output);
			manager.saveOntology(ontology, new RDFXMLOntologyFormat(), documentIRI2);
			manager.saveOntology(ontology, new SystemOutDocumentTarget());		
		}
		catch (Exception e) 
		{
			e.printStackTrace();
		}		
		
		

		return allElementsOneDimensional;
	}

	int checkType(String data)
	{
		if(data.length() > 16 && data.substring(1,15).equals("TASK-TREE-NODE"))
			return 0;
		return -1;	
	}

	ArrayList<String[]> convertToPrologReady(int type, String data)
	{
		if (type == 0)
		{
			String dataToBeUsed = data.substring(31);
			
			ArrayList<String[]> components = new ArrayList<String[]>();
			
			int paranthesisCount = 0;
			for(int i = 0; i < dataToBeUsed.length(); i++)
			{
				String currentChar = dataToBeUsed.substring(i, i+1);				
				if (currentChar.equals("(")) paranthesisCount++;
				else if (currentChar.equals(")")) paranthesisCount--;

				
				
				if (paranthesisCount > 0 && !(currentChar.equals(")")) && !(currentChar.equals("(")))
				{
					if((currentChar.equals("\n")) || (currentChar.equals("\t")))
						store.add(" ");
					else store.add(currentChar);
				}
				else if (paranthesisCount == 0)
				{
					String subData = "";
					for(int ind = 0; ind < store.size(); ind++)
					{
						String xx = store.get(ind).toString();
						subData = subData + xx;
					}
					store.clear();

					StringTokenizer divider = new StringTokenizer(subData, " ");
		
					int elementCount = divider.countTokens();
					String[] coreElements = new String[elementCount];
					int j = 0;
					while (divider.hasMoreElements()) 
					{
						coreElements[j] = divider.nextElement().toString();
						j++;
					}
	
					if (elementCount != 0)
						components.add(coreElements);

				} 
			}
			return components;
						
		}
		return null;
	}
       
        public static void main(String[] args) 
        {

                ROSClient2 d = new ROSClient2("knowrob_execution_trace_test_123");
		int i =1;
		while (i == 1)
		{
			/*Reply answer = */ d.getTrace();
			//String res = answer.result;
                	//System.out.println(x);
			
                }
        }

}

