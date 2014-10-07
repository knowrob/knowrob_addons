package org.knowrob.map;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.List;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseBuilder;
import org.semanticweb.owlapi.model.*;

import org.knowrob.owl.OWLClass;
import org.knowrob.owl.ObjectInstance;
import org.knowrob.owl.utils.OWLFileUtils;
import org.knowrob.owl.utils.OWLImportExport;

import semantic_map_to_owl.SemMapObject;


/**
 * ROS service to convert a mod_semantic_map/SemMap message into an OWL description
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */

public class SemanticMapToOWL  extends AbstractNodeMain {

	ConnectedNode node;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("semantic_map_to_owl");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {

		this.node = connectedNode;
		connectedNode.newServiceServer("knowrob_semantic_map_to_owl/generate_owl_map", 
				semantic_map_to_owl.GenerateSemanticMapOWL._TYPE,
				new ConvertToOwlCallback());
	}


	/**
	 * 
	 * Callback class for querying the Web for the object type of a barcode
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceResponseBuilder<semantic_map_to_owl.GenerateSemanticMapOWLRequest, semantic_map_to_owl.GenerateSemanticMapOWLResponse> {

		@Override
		public void build(semantic_map_to_owl.GenerateSemanticMapOWLRequest req, semantic_map_to_owl.GenerateSemanticMapOWLResponse res) {

			res.setOwlmap("");

			if (req.getMap() != null && req.getMap().getObjects().size()>0) {

				OWLImportExport export = new OWLImportExport();

				// get address from ROS parameters
				ArrayList<String[]> address = new ArrayList<String[]>();
				String namespace = OWLImportExport.IAS_MAP; // use IAS_MAP as default, PREFIX_MANAGER is set by default

				ParameterTree params = node.getParameterTree();

				if(params.has("map_address_room_nr") && params.getString("map_address_room_nr")!=null)
					address.add(new String[]{"knowrob:RoomInAConstruction", "knowrob:roomNumber", params.getString("map_address_room_nr")});

				if(params.has("map_address_floor_nr") && params.getString("map_address_floor_nr")!=null)
					address.add(new String[]{"knowrob:LevelOfAConstruction", "knowrob:floorNumber", params.getString("map_address_floor_nr")});

				if(params.has("map_address_street_nr") && params.getString("map_address_street_nr")!=null)
					address.add(new String[]{"knowrob:Building", "knowrob:streetNumber", params.getString("map_address_street_nr")});

				if(params.has("map_address_street_name") && params.getString("map_address_street_name")!=null)
					address.add(new String[]{"knowrob:Street", "rdfs:label", params.getString("map_address_street_name")});

				if(params.has("map_address_city_name") && params.getString("map_address_city_name")!=null)
					address.add(new String[]{"knowrob:City", "rdfs:label", params.getString("map_address_city_name")});


				if (params.has("map_owl_namespace") && params.getString("map_owl_namespace")!=null) {
					namespace = params.getString("map_owl_namespace");

					if(!namespace.endsWith("#"))
						namespace += "#";
				}

				System.err.println("Using map id: " + namespace);


				OWLOntology owlmap = export.createOWLMapDescription(namespace, 
						"SemanticEnvironmentMap" + new SimpleDateFormat("yyyyMMddHHmmss").format(Calendar.getInstance().getTime()), 
						semMapObj2MapObj(namespace, req.getMap().getObjects()), address);
				res.setOwlmap(OWLFileUtils.saveOntologytoString(owlmap, owlmap.getOWLOntologyManager().getOntologyFormat(owlmap)));

			}
		}
	}


	private ArrayList<ObjectInstance> semMapObj2MapObj(String map_id, List<SemMapObject> smos) {

		HashMap<Integer, ObjectInstance> intIdToID = new HashMap<Integer, ObjectInstance>();
		ArrayList<ObjectInstance> mos = new ArrayList<ObjectInstance>();

		for(SemMapObject smo : smos) {

			ObjectInstance mo = ObjectInstance.getObjectInstance(smo.getType() + smo.getId());
			intIdToID.put(smo.getId(), mo);

			mo.addType(OWLClass.getOWLClass(smo.getType()));

			mo.getDimensions().x=smo.getWidth();
			mo.getDimensions().y=smo.getDepth();
			mo.getDimensions().z=smo.getHeight();

			for(int i=0;i<4;i++) {
				for(int j=0;j<4;j++) {
					mo.getPoseMatrix().setElement(i, j, smo.getPose()[4*i+j]);
				}
			}

			if(intIdToID.get(smo.getPartOf()) != null)
				intIdToID.get(smo.getPartOf()).addPhysicalPart(mo);

			mos.add(mo);
		}

		return mos;
	}
}
