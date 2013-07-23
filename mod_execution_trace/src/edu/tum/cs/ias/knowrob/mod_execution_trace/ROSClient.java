package edu.tum.cs.ias.knowrob.mod_execution_trace;

//java stuff
import java.io.* ;
import java.io.File;
import java.io.StringReader;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.util.*;
/*import java.util.StringTokenizer;
import java.util.Stack;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;*/


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

//knowrob stuff
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;

public class ROSClient 
{

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
			req.filename = "/home/asil/Desktop/traces-4/a.ek";
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



		count = 0;
		boolean isNewTask = true;
		//Variable task = new Variable("Xxx");
		String task = "Xxx";

		for(int x = 0; x < allElementsOneDimensional.length; x++)
		{
			String current = allElementsOneDimensional[x];

			if(isNewTask)
			{
				task = "task_" + count;
				/*Compound assert_goal = new Compound(
 					"rdf_has",
 					new Term[] {
 						task,
						new Atom("rdf:type"),
 						new Atom("execution_trace:'Task'")
 					}
 				);*/

				//System.out.println("deneme");
				//PrologInterface.executeQuery("Ta = " + task + "");
				//PrologInterface.executeQuery("term_to_atom(Ta, Tsk)");

				PrologInterface.executeQuery("rdf_assert(" + task + ", rdf:type, execution_trace:'Task')"); 		
				
				isNewTask = false;
				count++;
			}
			
			if(current.equals("GOAL"))
			{
				String prefix = "goal_";
				prefix = prefix.concat(allElementsOneDimensional[x+1]);
				prefix = prefix.concat("_");
				prefix = prefix.concat(allElementsOneDimensional[x+2]);
				prefix = prefix.concat("_");
				prefix = prefix.concat(allElementsOneDimensional[x+3]);
				prefix = prefix.replace("-", "_");
				prefix = prefix.replace(":", "_");
				prefix = prefix.replace("?", "");

				//PrologInterface.executeQuery("Gl = '" + prefix + "'");
				//PrologInterface.executeQuery("term_to_atom(Gl, goal)");

				PrologInterface.executeQuery("rdf_assert(" + prefix + ", rdf:type, execution_trace:'Goal')");

				PrologInterface.executeQuery("rdf_assert(" + task + ", execution_trace:'taskGoal'," + prefix + ")");


				x = x + 3;
			}
			else if(current.equals("GOAL-CONTEXT"))
			{
				String prefix = "goalcontext_";
				prefix = prefix.concat(allElementsOneDimensional[x+1]);
				prefix = prefix.concat("_");
				prefix = prefix.concat(allElementsOneDimensional[x+2]);
				prefix = prefix.replace("-", "_");
				prefix = prefix.replace(":", "_");
				prefix = prefix.replace("?", "");

				//PrologInterface.executeQuery("GlC = '" + prefix + "'");
				//PrologInterface.executeQuery("term_to_atom(GlC, goalcontext)");

				PrologInterface.executeQuery("rdf_assert(" + prefix + ", rdf:type, execution_trace:'GoalContext')");

				PrologInterface.executeQuery("rdf_assert(" + task + ", execution_trace:'taskGoalContext'," + prefix + ")");				


				x = x + 2;
			}
			else if(current.equals("TOP-LEVEL"))
			{
				String prefix = "toplevel_";
				prefix = prefix.concat(allElementsOneDimensional[x+1]);
				prefix = prefix.replace("-", "_");
				prefix = prefix.replace(":", "_");
				prefix = prefix.replace("?", "");

				//PrologInterface.executeQuery("ToL = '" + prefix + "'");
				//PrologInterface.executeQuery("term_to_atom(ToL, toplevel)");

				PrologInterface.executeQuery("rdf_assert(" + prefix + ", rdf:type, execution_trace:'TopLevel')");

				PrologInterface.executeQuery("rdf_assert(" + task + ", execution_trace:'taskTopLevel'," + prefix + ")");



				isNewTask = true;
				x = x + 1;

			}		


			
		}

		InputStreamReader istream = new InputStreamReader(System.in);
                BufferedReader bufRead = new BufferedReader(istream);

		String prologQuery = "";

		try {
               		System.out.println("Enter prolog query: ");
               		prologQuery = bufRead.readLine();
          	}
          	catch (IOException err) {
               		System.out.println("Error reading line");
          	}

		HashMap<String, Vector<String>> results = PrologInterface.executeQuery(prologQuery);

		/*Set set = results.entrySet(); 
		// Get an iterator 
		Iterator it = set.iterator(); 
		// Display elements 
		while(it.hasNext()) 
		{ 
			Map.Entry me = (Map.Entry)it.next(); 
			System.out.print(me.getKey() + ": "); 
			System.out.println(me.getValue()); 
		} */

		System.out.println(results);

		System.out.println(allElementsOneDimensional.length);

		/*for(int x = 0; x < allElementsOneDimensional.length; x++)
			System.out.println(allElementsOneDimensional[x]);*/

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

                ROSClient d = new ROSClient("knowrob_execution_trace_test_123");
		int i =1;
		while (i == 1)
		{
			/*Reply answer = */ d.getTrace();
			//String res = answer.result;
                	//System.out.println(x);
			
                }
        }

}
