/*
 * Copyright (c) 2010-14 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
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

