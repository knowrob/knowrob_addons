package org.knowrob.map;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import semantic_map_to_owl.SemMapObject;


public class SemanticMapToOWLTestClient extends AbstractNodeMain {

	ConnectedNode node;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("semantic_map_to_owl_test_client");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		
		this.node = connectedNode;
		
		ServiceClient<semantic_map_to_owl.GenerateSemanticMapOWLRequest, semantic_map_to_owl.GenerateSemanticMapOWLResponse> serviceClient;
		
		try {
			serviceClient = connectedNode.newServiceClient("knowrob_semantic_map_to_owl/generate_owl_map", semantic_map_to_owl.GenerateSemanticMapOWL._TYPE);
			
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}


		// create semantic map message
		final semantic_map_to_owl.GenerateSemanticMapOWLRequest req = serviceClient.newMessage();
		
		// Set the IRI for the map that will be created
		req.getMap().getHeader().setFrameId("http://www.example.com/foo.owl#");
		req.getMap().getHeader().setStamp(node.getCurrentTime());


		// create cupboard
		SemMapObject cupboard = node.getTopicMessageFactory().newFromType(SemMapObject._TYPE);
		
		cupboard.setId(1);
		cupboard.setPartOf(0);
		cupboard.setType("cupboard");

		cupboard.setDepth(55.6f);
		cupboard.setWidth(60.7f);
		cupboard.setHeight(70.6f);

		cupboard.setPose(new float[] { 	1.0f,0.0f,0.0f,2.3f,
										0.0f,1.0f,0.0f,1.2f,
										0.0f,0.0f,1.0f,0.4f,
										0.0f,0.0f,0.0f,1.0f});
		req.getMap().getObjects().add(cupboard);
		
		// create door
		SemMapObject door = node.getTopicMessageFactory().newFromType(SemMapObject._TYPE);
		door.setId(2);
		door.setPartOf(1);
		door.setType("Door");

		door.setDepth(0.6f);
		door.setWidth(60.7f);
		door.setHeight(70.6f);

		door.setPose(new float[] { 1.0f,0.0f,0.0f,2.4f,
													0.0f,1.0f,0.0f,1.3f,
													0.0f,0.0f,1.0f,0.4f,
													0.0f,0.0f,0.0f,1.0f});
		req.getMap().getObjects().add(door);
		
		
		
		// create hinge
		SemMapObject hinge = node.getTopicMessageFactory().newFromType(SemMapObject._TYPE);
		hinge.setId(3);
		hinge.setPartOf(2);
		hinge.setType("HINGEDJOINT");

		hinge.setDepth(5.6f);
		hinge.setWidth(0.7f);
		hinge.setHeight(0.6f);

		hinge.setPose(new float[] { 1.0f,0.0f,0.0f,2.5f,
			0.0f,1.0f,0.0f,1.4f,
			0.0f,0.0f,1.0f,0.4f,
			0.0f,0.0f,0.0f,1.0f});
		req.getMap().getObjects().add(hinge);

		// create handle
		SemMapObject handle = node.getTopicMessageFactory().newFromType(SemMapObject._TYPE);
		handle.setId(4);
		handle.setPartOf(2);
		handle.setType("handle");

		handle.setDepth(5.6f);
		handle.setWidth(6.7f);
		handle.setHeight(7.6f);

		handle.setPose(new float[] { 1.0f,0.0f,0.0f,2.6f,
			0.0f,1.0f,0.0f,1.5f,
			0.0f,0.0f,1.0f,0.4f,
			0.0f,0.0f,0.0f,1.0f});
		req.getMap().getObjects().add(handle);

		
		serviceClient.call(req, new ServiceResponseListener<semantic_map_to_owl.GenerateSemanticMapOWLResponse>() {
			@Override
			public void onSuccess(semantic_map_to_owl.GenerateSemanticMapOWLResponse response) {
				
				connectedNode.getLog().info(
						String.format("%s", response.getOwlmap()));
			}

			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
		});
	}
}

