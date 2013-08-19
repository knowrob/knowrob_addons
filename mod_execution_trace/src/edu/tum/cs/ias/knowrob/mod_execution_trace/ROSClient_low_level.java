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
import java.lang.Integer;
import java.sql.Timestamp;
import java.lang.*;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import ros.*;
import ros.communication.*;
import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;

public class ROSClient_low_level 
{

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;

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

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n = ros.createNodeHandle();

                n.spinOnce();
               
        }

	public double[] getBelief(String object, String date) 
	{
		int date_converted = Integer.parseInt(date);

		Designator d;
	
		d = mdb.getUIMAPerception(object, date_converted);
		
		PoseStamped pose = (PoseStamped)d.get("pose");
		Matrix4d poseMatrix = pose.getMatrix4d();

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
		
		return dummy;
	}

	public double[] getReal(/*String object, String date*/) 
	{
		double [][] dummy = new double[1][3];

		dummy[0][0] = 0;
		dummy[0][1] = 0;
		dummy[0][2] = 0;

		System.out.println("info not available");

		return dummy[0];

	}

        public long[] getPerceptionTimeStamps(String object) 
	{
		List<Date> listOfTimes = mdb.getUIMAPerceptionTimes(object);

		long[] timeStamps = new long[listOfTimes.size()];

		for(int i = 0; i < listOfTimes.size(); i++)		
			timeStamps[i] = listOfTimes.get(i).getTime();

		return timeStamps;

	}

	public String[] getPerceptionObjects(String date) 
	{
		int date_converted = Integer.parseInt(date);
		List<String> listOfObjects = mdb.getUIMAPerceptionObjects(date_converted);

		String[] objects = new String[listOfObjects.size()];

		for(int i = 0; i < listOfObjects.size(); i++)		
			objects[i] = listOfObjects.get(i);

		return objects;

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

	public static void main(String[] args) {

		ROSClient_low_level deneme = new ROSClient_low_level("deneme");

		Timestamp timestamp = Timestamp.valueOf("2013-08-05 15:32:35.0");
		long d = timestamp.getTime();
		System.out.println(d);
		deneme.getBelief("51ffa963106a029da6b91a32", "" + d/1000);
	}
}
