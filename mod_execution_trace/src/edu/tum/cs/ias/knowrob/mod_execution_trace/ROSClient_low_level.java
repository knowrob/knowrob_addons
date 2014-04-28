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

import java.util.*;
import java.util.Map.*;
import java.lang.Integer;
import java.sql.Timestamp;
import java.lang.*;
import java.awt.image.BufferedImage;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Date;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.ByteArrayOutputStream;
import java.awt.image.DataBufferByte;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import ros.*;
import ros.communication.*;
import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;
import ros.pkg.designator_integration_msgs.msg.*;
import ros.pkg.sensor_msgs.msg.Image;

public class ROSClient_low_level 
{

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;
	static Publisher<ros.pkg.designator_integration_msgs.msg.Designator> pub;
	static Publisher<ros.pkg.sensor_msgs.msg.Image> pub_image;

	MongoDBInterface mdb;



	/**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient_low_level(String node_name) 
	{
                initRos(node_name);

		mdb = new MongoDBInterface();
        }



	/**
         * Initialize the ROS environment if it has not yet been initialized
         *
         * @param node_name A unique node name
         */
        protected static void initRos(String node_name) 
	{

                ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) 
		{
                        ros.init(node_name);
                }
                n = ros.createNodeHandle();
		try 
		{ 
			pub = n.advertise("/logged_designators", new ros.pkg.designator_integration_msgs.msg.Designator(), 100);
			pub_image = n.advertise("/logged_images", new ros.pkg.sensor_msgs.msg.Image(), 100); 
		}
		catch (ros.RosException r1)
		{
			System.out.println("Exception thrown");
		}
                n.spinOnce();
               
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


		ros.pkg.sensor_msgs.msg.Image image_msg = new ros.pkg.sensor_msgs.msg.Image();	
		image_msg.header.stamp = Time.now(); 
		image_msg.encoding = "bgr8";
		image_msg.height = image.getHeight();
		image_msg.width = image.getWidth();
		image_msg.step = image_msg.width * 3;
    		image_msg.data = new short[(int)image_msg.height * (int)image_msg.step];
		byte[] bytes = ((DataBufferByte)newImage.getRaster().getDataBuffer()).getData();
		for(int x = 0; x < image_msg.height * (int)image_msg.width; x = x + 1)
		{
			image_msg.data[3 * x] = (short) bytes[3 * x];
			image_msg.data[3 * x + 1] = (short) bytes[3 * x + 1];
			image_msg.data[3 * x + 2] = (short) bytes[3 * x + 2];
		}

		pub_image.publish(image_msg);
		return true;

	}

	public boolean publishDesignator(org.knowrob.interfaces.mongo.types.Designator designator)
	{
		ros.pkg.designator_integration_msgs.msg.Designator designator_msg = new ros.pkg.designator_integration_msgs.msg.Designator();

		try
		{
			if(designator.getType().toString().toLowerCase() == "action")
				designator_msg.type = 1;
			else if(designator.getType().toString().toLowerCase() == "object")
				designator_msg.type = 2;
			else if(designator.getType().toString().toLowerCase() == "location")
				designator_msg.type = 0;			
		}
		catch (java.lang.NullPointerException exc)
		{
			designator_msg.type = 0;
		}
		designator_msg.description = new ArrayList<ros.pkg.designator_integration_msgs.msg.KeyValuePair>();

		this.publishDesignator2(designator, designator_msg, 0);

		return true;

	}

	public int publishDesignator2(org.knowrob.interfaces.mongo.types.Designator designator, ros.pkg.designator_integration_msgs.msg.Designator designator_msg, 
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
				KeyValuePair k1 = new KeyValuePair();
				designator_msg.description.add(k1);
				
				int c = designator_msg.description.size()-1;

				designator_msg.description.get(c).id = x + 1;
				designator_msg.description.get(c).key = key;
				if(parentId == 0)
				{
					designator_msg.description.get(c).parent = parentId;
				}
				else 
				{
					designator_msg.description.get(c).parent = parentId +1;
				}
			

				if (currentEntry.getValue().getClass().equals(Double.TYPE)) 
				{
					designator_msg.description.get(c).type = 1;
					Double current_value = (Double)currentEntry.getValue();
					designator_msg.description.get(c).value_float = (float)current_value.doubleValue();						
	    			}
				else if (currentEntry.getValue().getClass().equals(Integer.TYPE)) 
				{
					designator_msg.description.get(c).type = 1;
					Double current_value = (Double)currentEntry.getValue();
					designator_msg.description.get(c).value_float = (float)current_value.doubleValue();
	    			}
				else if (currentEntry.getValue().getClass().equals(org.knowrob.interfaces.mongo.types.ISODate.class)) 
				{
	       				designator_msg.description.get(c).type = 0;
					org.knowrob.interfaces.mongo.types.ISODate date = (ISODate)currentEntry.getValue();
					designator_msg.description.get(c).value_string = date.toString();
	   			}
				else if (currentEntry.getValue().getClass().equals(org.knowrob.interfaces.mongo.types.Designator.class)) 
				{
	       				org.knowrob.interfaces.mongo.types.Designator inner_designator = (org.knowrob.interfaces.mongo.types.Designator)currentEntry.getValue();					
					try
					{
						if(inner_designator.getType().toString().toLowerCase() == "action")
							designator_msg.description.get(c).type = 6;
						else if(inner_designator.getType().toString().toLowerCase() == "object")
							designator_msg.description.get(c).type = 7;
						else if(inner_designator.getType().toString().toLowerCase() == "location")
							designator_msg.description.get(c).type = 8;				
					}
					catch (java.lang.NullPointerException exc)
					{
						designator_msg.description.get(c).type = 6;
					}
					int inner_size = this.publishDesignator2(inner_designator, designator_msg, x);
					x += inner_size;
					difference_in_x += inner_size;
	   			}
				else if (currentEntry.getValue().getClass().equals(org.knowrob.interfaces.mongo.types.PoseStamped.class)) 
				{
					designator_msg.description.get(c).type = 4;
	       				ros.pkg.geometry_msgs.msg.PoseStamped pose = (org.knowrob.interfaces.mongo.types.PoseStamped)currentEntry.getValue();
					designator_msg.description.get(c).value_posestamped = pose;
	   			}
				else if (currentEntry.getValue().getClass().equals(org.knowrob.interfaces.mongo.types.Pose.class)) 
				{
					designator_msg.description.get(c).type = 5;
	       				ros.pkg.geometry_msgs.msg.Pose pose = (org.knowrob.interfaces.mongo.types.Pose)currentEntry.getValue();
					designator_msg.description.get(c).value_pose = pose;
	   			}
				else if (currentEntry.getValue().getClass().equals(String.class)) 
				{
	       				designator_msg.description.get(c).type = 0;
					designator_msg.description.get(c).value_string = (String)currentEntry.getValue();
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

	public static void main(String[] args) {

		MongoDBInterface mdb2 = new MongoDBInterface();

		ROSClient_low_level deneme = new ROSClient_low_level("deneme");
		org.knowrob.interfaces.mongo.types.Designator d1 = mdb2.getDesignatorByID("designator_d1QZprbsST3yO6");
		deneme.publishDesignator(d1);
		/*Timestamp timestamp = Timestamp.valueOf("2013-08-05 15:32:35.0");
		long d = timestamp.getTime();
		System.out.println(d);
		deneme.getBelief("51ffa963106a029da6b91a32", "" + d/1000);*/
	}
}
