package edu.tum.cs.ias.knowrob.map;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import org.semanticweb.owlapi.model.*;

import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;
import edu.tum.cs.ias.knowrob.owl.utils.OWLImportExport;

import ros.*;
import ros.pkg.mod_semantic_map.srv.*;
import ros.pkg.mod_semantic_map.msg.*;


/**
 * ROS service to convert a mod_semantic_map/SemMap message into an OWL description
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */

public class SemanticMapToOWL {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public SemanticMapToOWL() {

		try {

			initRos();

			n.advertiseService("/knowrob_semantic_map_to_owl/generate_owl_map", 
					new GenerateSemanticMapOWL(), 
					new ConvertToOwlCallback());
			ros.spin();

		} catch (RosException e) {
			e.printStackTrace();	
		}

	}



	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_semantic_map_to_owl");
		}
		n = ros.createNodeHandle();

	}

	/**
	 * 
	 * Callback class for querying the Web for the object type of a barcode
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceServer.Callback<GenerateSemanticMapOWL.Request, GenerateSemanticMapOWL.Response> {

		@Override
		public GenerateSemanticMapOWL.Response call(GenerateSemanticMapOWL.Request req) {

			GenerateSemanticMapOWL.Response res = new GenerateSemanticMapOWL.Response();
			res.owlmap="";

			if (req.map != null && req.map.objects.size()>0) {

				OWLImportExport export = new OWLImportExport();

				// get address from ROS parameters
				ArrayList<String[]> address = new ArrayList<String[]>();
				String namespace = OWLImportExport.IAS_MAP; // use IAS_MAP as default, PREFIX_MANAGER is set by default

				try {

					if(n.hasParam("map_address_room_nr") && n.getStringParam("map_address_room_nr")!=null)
						address.add(new String[]{"knowrob:RoomInAConstruction", "knowrob:roomNumber", n.getStringParam("map_address_room_nr")});

					if(n.hasParam("map_address_floor_nr") && n.getStringParam("map_address_floor_nr")!=null)
						address.add(new String[]{"knowrob:LevelOfAConstruction", "knowrob:floorNumber", n.getStringParam("map_address_floor_nr")});
					
					if(n.hasParam("map_address_street_nr") && n.getStringParam("map_address_street_nr")!=null)
						address.add(new String[]{"knowrob:Building", "knowrob:streetNumber", n.getStringParam("map_address_street_nr")});

					if(n.hasParam("map_address_street_name") && n.getStringParam("map_address_street_name")!=null)
						address.add(new String[]{"knowrob:Street", "rdfs:label", n.getStringParam("map_address_street_name")});
					
					if(n.hasParam("map_address_city_name") && n.getStringParam("map_address_city_name")!=null)
						address.add(new String[]{"knowrob:City", "rdfs:label", n.getStringParam("map_address_city_name")});
					

					if (n.hasParam("map_owl_namespace") && n.getStringParam("map_owl_namespace")!=null) {
						namespace = n.getStringParam("map_owl_namespace");
						
						if(!namespace.endsWith("#"))
							namespace += "#";
					}

					System.err.println("Using map id: " + namespace);
					
				} catch (RosException e) {
					e.printStackTrace();
				}
				

				OWLOntology owlmap = export.createOWLMapDescription(namespace, 
							"SemanticEnvironmentMap" + new SimpleDateFormat("yyyyMMddHHmmss").format(Calendar.getInstance().getTime()), 
							semMapObj2MapObj(namespace, req.map.objects), address);
				res.owlmap = OWLFileUtils.saveOntologytoString(owlmap, owlmap.getOWLOntologyManager().getOntologyFormat(owlmap));

			}

			return res;
		}
	}

	
	private ArrayList<ObjectInstance> semMapObj2MapObj(String map_id, ArrayList<SemMapObject> smos) {
		
		HashMap<Integer, ObjectInstance> intIdToID = new HashMap<Integer, ObjectInstance>();
		ArrayList<ObjectInstance> mos = new ArrayList<ObjectInstance>();
		
		for(SemMapObject smo : smos) {
			
			ObjectInstance mo = ObjectInstance.getObjectInstance(smo.type + smo.id);
			intIdToID.put(smo.id, mo);
			
			mo.addType(OWLClass.getOWLClass(smo.type));
			
			mo.getDimensions().x=smo.width;
			mo.getDimensions().y=smo.depth;
			mo.getDimensions().z=smo.height;

			for(int i=0;i<4;i++) {
				for(int j=0;j<4;j++) {
					mo.getPoseMatrix().setElement(i, j, smo.pose[4*i+j]);
				}
			}

			if(intIdToID.get(smo.partOf) != null)
			    intIdToID.get(smo.partOf).addPhysicalPart(mo);

			mos.add(mo);
		}
				
		return mos;
	}


	
	public static void main(String[] args) {


//		if (args.length == 0) {
			// run service
			new SemanticMapToOWL();
			
//		} else {
//			System.out.println("usage: rosrun mod_semantic_map SemanticMapToOWL");
//			System.out.println("Commands:");
//			System.out.println("        rosrun mod_semantic_map SemanticMapToOWL       Runs the service");
//			System.out.println();
//		}

	}


}
