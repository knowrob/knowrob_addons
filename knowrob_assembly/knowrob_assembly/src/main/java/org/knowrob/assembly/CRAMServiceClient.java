package org.knowrob.assembly;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class CRAMServiceClient extends AbstractNodeMain {
	private static CRAMServiceClient instance = null;
	
	//private ServiceClient<std_msgs.String, std_msgs.String> client = null;
	private ConnectedNode node = null;
	
	
	public static CRAMServiceClient get() {
		if(instance==null) instance = new CRAMServiceClient();
		return instance;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("perform_assembly_designator");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.node = connectedNode;
		//try {
		//	client = connectedNode.newServiceClient("/cram/perform-action-designator", std_msgs.String._TYPE);
		//} catch (ServiceNotFoundException e) {
		//	throw new RosRuntimeException(e);
		//}
	}
	
	public void performDesignator(final String designator) {
		waitForNode();
		System.out.println("    [CRAM] perform-designator");
		System.out.println(designator);
		/*
		final std_msgs.String request = client.newMessage();
		request.setData(designator);
		client.call(request, new ServiceResponseListener<std_msgs.String>() {
			@Override
			public void onSuccess(std_msgs.String response) {
			}
			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
		});
		*/
	}
	
	private void waitForNode() {
		try {
			while(this.node == null) Thread.sleep(200);
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
