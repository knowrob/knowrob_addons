/*
 * ROSClient_low_level.java
 * Copyright (c) 2013, Asil Kaan Bozcuoglu, Institute for Artifical Intelligence, Universitaet Bremen
 * asil@cs.uni-bremen.de
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Universitaet Bremen nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

package org.knowrob.cram;

import geometry_msgs.PoseStamped;

import java.util.*;
import java.util.Map.*;
import java.lang.Integer;
import java.nio.ByteOrder;
import java.awt.image.BufferedImage;
import javax.vecmath.Matrix4d;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.awt.image.DataBufferByte;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import designator_integration_msgs.KeyValuePair;


public class LogdataPublisher extends AbstractNodeMain {

	MongoDBInterface mdb;

	ConnectedNode node;
	Publisher<designator_integration_msgs.Designator> pub;
	Publisher<sensor_msgs.Image> pub_image;


	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_log_publisher");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		node = connectedNode;
		mdb = new MongoDBInterface();

		pub = connectedNode.newPublisher("logged_designators", designator_integration_msgs.Designator._TYPE);
		pub_image = connectedNode.newPublisher("logged_images", sensor_msgs.Image._TYPE); 

	}


	public boolean publishImage(String image_path)
	{
		BufferedImage image = null;
		try {
			image = ImageIO.read(new File(image_path.replace("'", "")));
		} catch (IOException e) {
			e.printStackTrace();
		}

		// wait for publisher to be ready
		try {
			while(pub_image ==null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		// create image message from file
		sensor_msgs.Image image_msg = pub_image.newMessage();

		image_msg.getHeader().setStamp(node.getCurrentTime());
		image_msg.setEncoding("bgr8");
		image_msg.setHeight(image.getHeight());
		image_msg.setWidth(image.getWidth());
		image_msg.setStep(image_msg.getWidth() * 3);


		byte[] bytes = ((DataBufferByte)image.getRaster().getDataBuffer()).getData();
		ChannelBuffer cb = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, bytes);

		image_msg.setData(cb);
		pub_image.publish(image_msg);
		return true;

	}

	public boolean publishDesignator(org.knowrob.interfaces.mongo.types.Designator designator) {

		// wait for publisher to be ready
		try {
			while(pub ==null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		designator_integration_msgs.Designator designator_msg = pub.newMessage();

		try {
			if(designator.getType().toString().toLowerCase() == "action")
				designator_msg.setType(1);
			else if(designator.getType().toString().toLowerCase() == "object")
				designator_msg.setType(2);
			else if(designator.getType().toString().toLowerCase() == "location")
				designator_msg.setType(0);			
		} catch (java.lang.NullPointerException exc) {
			designator_msg.setType(0);
		}

		this.publishDesignator2(designator, designator_msg, 0);

		return true;

	}

	public int publishDesignator2(Designator designator, designator_integration_msgs.Designator designator_msg, int parentId) {

		Set<Entry<String, Object>> values = designator.entrySet();
		Object[] pairs = values.toArray();

		int difference_in_x = 0;
		for(int x = parentId; x < pairs.length + parentId + difference_in_x; x++)
		{

			Entry<String, Object> currentEntry = (Entry<String, Object>) pairs[x - parentId - difference_in_x];
			String key = currentEntry.getKey();
			if( !(key.substring(0,1).equals("_"))) // check if publishable key
			{

				KeyValuePair k1 = node.getTopicMessageFactory().newFromType(designator_integration_msgs.KeyValuePair._TYPE);
				designator_msg.getDescription().add(k1);

				int c = designator_msg.getDescription().size()-1;

				designator_msg.getDescription().get(c).setId(x + 1);
				designator_msg.getDescription().get(c).setKey(key);
				if(parentId == 0)
				{
					designator_msg.getDescription().get(c).setParent(parentId);
				}
				else 
				{
					designator_msg.getDescription().get(c).setParent(parentId +1);
				}


				if (currentEntry.getValue() instanceof Double) 
				{
					designator_msg.getDescription().get(c).setType(1);
					Double current_value = (Double)currentEntry.getValue();
					designator_msg.getDescription().get(c).setValueFloat((float)current_value.doubleValue());						
				}
				else if (currentEntry.getValue() instanceof Integer) 
				{
					designator_msg.getDescription().get(c).setType(1);
					Double current_value = (Double)currentEntry.getValue();
					designator_msg.getDescription().get(c).setValueFloat((float)current_value.doubleValue());
				}
				else if (currentEntry.getValue() instanceof ISODate) 
				{
					designator_msg.getDescription().get(c).setType(0);
					ISODate date = (ISODate)currentEntry.getValue();
					designator_msg.getDescription().get(c).setValueString(date.toString());
				}
				else if (currentEntry.getValue() instanceof Designator) 
				{
					Designator inner_designator = (Designator)currentEntry.getValue();					
					try
					{
						if(inner_designator.getType().toString().toLowerCase() == "action")
							designator_msg.getDescription().get(c).setType(6);
						else if(inner_designator.getType().toString().toLowerCase() == "object")
							designator_msg.getDescription().get(c).setType(7);
						else if(inner_designator.getType().toString().toLowerCase() == "location")
							designator_msg.getDescription().get(c).setType(8);				
					}
					catch (java.lang.NullPointerException exc)
					{
						designator_msg.getDescription().get(c).setType(6);
					}
					int inner_size = this.publishDesignator2(inner_designator, designator_msg, x);
					x += inner_size;
					difference_in_x += inner_size;
				}
				else if (currentEntry.getValue() instanceof PoseStamped) 
				{
					designator_msg.getDescription().get(c).setType(4);
					PoseStamped pose = (PoseStamped) currentEntry.getValue();
					designator_msg.getDescription().get(c).setValuePosestamped(pose);
				}
				else if (currentEntry.getValue() instanceof geometry_msgs.Pose) 
				{
					designator_msg.getDescription().get(c).setType(5);
					geometry_msgs.Pose pose = (geometry_msgs.Pose) currentEntry.getValue();
					designator_msg.getDescription().get(c).setValuePose(pose);
				}
				else if (currentEntry.getValue().getClass().equals(String.class)) 
				{
					designator_msg.getDescription().get(c).setType(0);
					designator_msg.getDescription().get(c).setValueString((String)currentEntry.getValue());
				}
			}
		}

		if(parentId == 0)
		{
			pub.publish(designator_msg);
		}
		return pairs.length + difference_in_x;

	}

	public boolean publishDesignator(String designatorId) 
	{
		StringTokenizer s1 = new StringTokenizer(designatorId, "#");
		s1.nextToken();
		designatorId= s1.nextToken();

		org.knowrob.interfaces.mongo.types.Designator d1 = mdb.getDesignatorByID(designatorId);
		publishDesignator(d1);
		return true;
	}

	public double[] getBeliefByDesignator(String designatorId) 
	{

		StringTokenizer s1 = new StringTokenizer(designatorId, "#");
		s1.nextToken();
		designatorId= s1.nextToken();

		// wait for node to be ready
		try {
			while(mdb == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		org.knowrob.interfaces.mongo.types.Designator d1 = mdb.getDesignatorByID(designatorId);

		if(d1!=null) {
			publishDesignator(d1);
			Matrix4d poseMatrix;
			try 
			{
				poseMatrix = mdb.getDesignatorLocation(designatorId);
			}
			catch (java.lang.NullPointerException e)
			{
				poseMatrix = null;
			}

			double[] dummy = new double[16];
			if(poseMatrix != null)
			{
				dummy[0] = poseMatrix.getElement(0, 0); 
				dummy[1] = poseMatrix.getElement(0, 1);
				dummy[2] = poseMatrix.getElement(0, 2);
				dummy[3] = poseMatrix.getElement(0, 3);
				dummy[4] = poseMatrix.getElement(1, 0);
				dummy[5] = poseMatrix.getElement(1, 1);
				dummy[6] = poseMatrix.getElement(1, 2);
				dummy[7] = poseMatrix.getElement(1, 3);
				dummy[8] = poseMatrix.getElement(2, 0); 
				dummy[9] = poseMatrix.getElement(2, 1);
				dummy[10] = poseMatrix.getElement(2, 2);
				dummy[11] = poseMatrix.getElement(2, 3);
				dummy[12] = poseMatrix.getElement(3, 0);
				dummy[13] = poseMatrix.getElement(3, 1);
				dummy[14] = poseMatrix.getElement(3, 2);
				dummy[15] = poseMatrix.getElement(3, 3);
			}
			else dummy[15] = -1;

			return dummy;
		} else return new double[16];
	}

	public String getArmLink(String designatorId) 
	{
		StringTokenizer s1 = new StringTokenizer(designatorId, "#");
		s1.nextToken();
		designatorId= s1.nextToken();

		// wait for node to be ready
		try {
			while(mdb == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		org.knowrob.interfaces.mongo.types.Designator d1 = mdb.getDesignatorByID(designatorId);

		if(d1 != null) {
			publishDesignator(d1);
			String link_name = (String)d1.get("LINK");
			link_name = "/" + link_name;
			return	link_name;
		}
		return "";
	}
}
