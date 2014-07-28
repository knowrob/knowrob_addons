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

package edu.tum.cs.ias.knowrob.mod_execution_trace;

import geometry_msgs.PoseStamped;

import java.util.*;
import java.util.Map.*;
import java.lang.Integer;
import java.awt.image.BufferedImage;
import javax.vecmath.Matrix4d;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.awt.image.DataBufferByte;
import java.awt.Graphics2D;

import org.jboss.netty.buffer.ChannelBuffers;
import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import designator_integration_msgs.KeyValuePair;


public class ROSClient_low_level extends AbstractNodeMain {

	MongoDBInterface mdb;
	
	ConnectedNode node;
	Publisher<designator_integration_msgs.Designator> pub;
	Publisher<sensor_msgs.Image> pub_image;
	
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava_tutorial_pubsub/talker");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		node = connectedNode;
		mdb = new MongoDBInterface();
		
		pub = connectedNode.newPublisher("logged_designators", designator_integration_msgs.Designator._TYPE);
		pub_image = connectedNode.newPublisher("logged_images", sensor_msgs.Image._TYPE); 
		
		
//		final Publisher<std_msgs.String> publisher =
//				connectedNode.newPublisher("chatter", std_msgs.String._TYPE);
//		// This CancellableLoop will be canceled automatically when the node shuts
//		// down.
//		connectedNode.executeCancellableLoop(new CancellableLoop() {
//			private int sequenceNumber;
//
//			@Override
//			protected void setup() {
//				sequenceNumber = 0;
//			}
//
//			@Override
//			protected void loop() throws InterruptedException {
//				std_msgs.String str = publisher.newMessage();
//				str.setData("Hello world! " + sequenceNumber);
//				publisher.publish(str);
//				sequenceNumber++;
//				Thread.sleep(1000);
//			}
//		});
	}
	

	public int getDuration (int start, int end)
	{
		int duration = end - start;

		return duration;		
	} 



	public boolean publishImage(String image_path)
	{
		BufferedImage image = null;
		try 
		{
			image = ImageIO.read(new File(image_path.replace("'", "")));
		} 
		catch (IOException e) 
		{
			System.out.println("Exception thrown");
		}

		BufferedImage newImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
		Graphics2D g = newImage.createGraphics();
		g.drawImage(image, 0, 0, null);
		g.dispose();

		sensor_msgs.Image image_msg = pub_image.newMessage();

		image_msg.getHeader().setStamp(node.getCurrentTime()); 
		image_msg.setEncoding("bgr8");
		image_msg.setHeight(image.getHeight());
		image_msg.setWidth(image.getWidth());
		image_msg.setStep(image_msg.getWidth() * 3);
		
		byte[] bytes = ((DataBufferByte)newImage.getRaster().getDataBuffer()).getData();
//		for(int x = 0; x < image_msg.getHeight() * (int)image_msg.getWidth(); x = x + 1)
//		{
//			image_msg.data[3 * x] = (short) bytes[3 * x];
//			image_msg.data[3 * x + 1] = (short) bytes[3 * x + 1];
//			image_msg.data[3 * x + 2] = (short) bytes[3 * x + 2];
//		}
		
		image_msg.setData(ChannelBuffers.copiedBuffer(bytes));

		pub_image.publish(image_msg);
		return true;

	}

	public boolean publishDesignator(org.knowrob.interfaces.mongo.types.Designator designator)
	{
		designator_integration_msgs.Designator designator_msg = pub.newMessage();

		try
		{
			if(designator.getType().toString().toLowerCase() == "action")
				designator_msg.setType(1);
			else if(designator.getType().toString().toLowerCase() == "object")
				designator_msg.setType(2);
			else if(designator.getType().toString().toLowerCase() == "location")
				designator_msg.setType(0);			
		}
		catch (java.lang.NullPointerException exc)
		{
			designator_msg.setType(0);
		}
//		designator_msg.setDescription(new ArrayList<ros.pkg.designator_integration_msgs.msg.KeyValuePair>());

		this.publishDesignator2(designator, designator_msg, 0);

		return true;

	}

	public int publishDesignator2(Designator designator, designator_integration_msgs.Designator designator_msg, 
			int parentId)
	{

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

		org.knowrob.interfaces.mongo.types.Designator d1 = mdb.getDesignatorByID(designatorId);
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
		/*double o_x, o_y, o_z, o_w;
		o_x = Double.parseDouble((String)d.get("pose.pose.orientation.x"));
		o_y = Double.parseDouble((String)d.get("pose.pose.orientation.y"));
		o_z = Double.parseDouble((String)d.get("pose.pose.orientation.z"));
		o_w = Double.parseDouble((String)d.get("pose.pose.orientation.w"));

		double x, y, z, w;
		x = Double.parseDouble((String)d.get("pose.pose.position.x"));
		y = Double.parseDouble((String)d.get("pose.pose.position.y"));
		z = Double.parseDouble((String)d.get("pose.pose.position.z"));
		w = Double.parseDouble((String)d.get("pose.pose.position.w"));*/

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

	}

	public int timeComparison(String time1, String time2)
	{
		StringTokenizer s1 = new StringTokenizer(time1, "_");
		StringTokenizer s2 = new StringTokenizer(time2, "_");

		String time_value1 = "0";
		String time_value2 = "0";

		while (s1.hasMoreTokens())
		{
			time_value1 = s1.nextToken();
			time_value2 = s2.nextToken();
		}

		int time_value_integer_1 = Integer.parseInt(time_value1);
		int time_value_integer_2 = Integer.parseInt(time_value2);

		if( time_value_integer_1 > time_value_integer_2)
			return 2;
		else if( time_value_integer_1 == time_value_integer_2)
			return 0;
		else if( time_value_integer_1 < time_value_integer_2)
			return 1; 

		return -2;	
	}

	public String timeComparison2(String[] timeList, String time)
	{
		time = time.replace("'", "");
		StringTokenizer s1 = new StringTokenizer(time, "_");
		String time_value1 = "0";

		while (s1.hasMoreTokens())
			time_value1 = s1.nextToken();
		int time_value_integer_1 = Integer.parseInt(time_value1);		

		String latestTime = "";
		int min = 0;
		int count = 0;
		for(int x = 0; x < timeList.length; x++)
		{
			timeList[x] = timeList[x].replace("'", "");			
			StringTokenizer s2 = new StringTokenizer(timeList[x], "_");
			String time_value2 = "0";

			while (s2.hasMoreTokens())
				time_value2 = s2.nextToken();

			int time_value_integer_2 = Integer.parseInt(time_value2);

			if( count == 0 && time_value_integer_1 >= time_value_integer_2)
			{
				min = time_value_integer_1 - time_value_integer_2;
				count++;
				latestTime = timeList[x]; 
			}
			else if( time_value_integer_1 - time_value_integer_2 < min && time_value_integer_1 >= time_value_integer_2)
			{
				min = time_value_integer_1 - time_value_integer_2;
				count++;
				latestTime = timeList[x]; 
			} 
		}
		return latestTime;	
	}

	public int locationComparison(String location1, String location2)
	{
		StringTokenizer s1 = new StringTokenizer(location1, "_");
		StringTokenizer s2 = new StringTokenizer(location2, "_");

		String element_value1 = s1.nextToken();
		String element_value2 = s2.nextToken();

		double element_value_d1;
		double element_value_d2;
		while (s1.hasMoreTokens())
		{
			element_value1 = s1.nextToken();
			element_value2 = s2.nextToken();

			element_value_d1 = Double.parseDouble(element_value1);
			element_value_d2 = Double.parseDouble(element_value2);

			if(element_value_d1 != element_value_d2)
				return 1;

		}

		return 0;	
	}

	public String getArmLink(String designatorId) 
	{
		StringTokenizer s1 = new StringTokenizer(designatorId, "#");
		s1.nextToken();
		designatorId= s1.nextToken();

		org.knowrob.interfaces.mongo.types.Designator d1 = mdb.getDesignatorByID(designatorId);
		publishDesignator(d1);
		String link_name = (String)d1.get("LINK");
		link_name = "/" + link_name;
		return	link_name;	


	}
}
