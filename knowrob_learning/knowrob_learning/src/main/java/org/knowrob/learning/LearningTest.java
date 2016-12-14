package org.knowrob.learning;

import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.io.PrintStream;
import java.io.Serializable;


import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;


import weka.classifiers.Classifier;
import weka.core.Attribute;
import weka.core.FastVector;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Utils;
import weka.attributeSelection.*;

public class LearningTest extends AbstractNodeMain 
{
	ConnectedNode node;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("learning_test_client");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode)
	{
		String[] keys = new String[1];
		keys[0] = "header.frame_id";

		Object[] vals = new Object[1];
		vals[0] = new String("/head_mount_kinect_rgb_optical_frame");

		String[] rels = new String[1];
		rels[0] = "==";

		String[] fields = new String[1];
		fields[0] = "header.frame_id";

		String[] labels = new String[1];
		labels[0] = "A";

		ArrayList<String[]> testers = MongoFeaturizer.getInstancesFromMongo("kinect_head_rgb_image_color_compressed", keys, rels, vals, fields);

		System.out.println(testers.size());

	}
		
}
