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

import java.util.Date;

import ros.*;
import ros.communication.*;
import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;

public class ROSClient_low_level 
{

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;

	static MongoDBInterface mdb;



	/**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient_low_level(String node_name) 
	{
                initRos(node_name);
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

		mdb = new MongoDBInterface();

                n.spinOnce();
               
        }
	
	
        public double[] getObserved(/*String object, String date*/) 
	{
		/*if(object != null)
		{
			
			Designator d = mdb.latestUIMAPerceptionBefore(date);
			
		}
		else
		{
			Designator d = mdb.latestUIMAPerceptionBefore(date);
		}*/

		double [][] dummy = new double[1][3];

		dummy[0][0] = 0;
		dummy[0][1] = 0;
		dummy[0][2] = 0;

		return dummy[0];

        }

	public double[] getBelief(/*String object, String date*/) 
	{
		double [][] dummy = new double[1][3];

		dummy[0][0] = 0;
		dummy[0][1] = 0;
		dummy[0][2] = 0;

		System.out.println("info not available");

		return dummy[0];
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

}
