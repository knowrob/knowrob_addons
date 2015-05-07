/*
 * Copyright (c) 2012-13 Asil Kaan Bozcuoglu
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

package edu.tum.cs.ias.knowrob.mod_self_info;

import ros.*;
import ros.communication.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.pr2_msgs.msg.*;

public class ROSClient_battery_state {

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;
	static Subscriber.QueueingCallback<ros.pkg.pr2_msgs.msg.PowerState> callback;
	static Subscriber<ros.pkg.pr2_msgs.msg.PowerState> sub;
	static double r;


        /**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient_battery_state(String node_name) 
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


		callback = new Subscriber.QueueingCallback<ros.pkg.pr2_msgs.msg.PowerState>();
	  	
		try 
		{
			sub = n.subscribe("/power_state", new ros.pkg.pr2_msgs.msg.PowerState(), callback, 10);
		}
		catch (RosException e) 
		{
                        ros.logError("ROSClient: Error while subscribing power_state topicc");
                }
		r = 0;	
                n.spinOnce();
               
        }


        /**
         * Call the table_objects service and return the result
         * @return An array of CollisionObjects referring to the objects detected on the table
         */
        public double getPower() 
	{
		
		try
		{	
			while (!callback.isEmpty()) r = callback.pop().relative_capacity;
		}
		catch (java.lang.InterruptedException e)
		{
                        ros.logError("ROSClient: Stack is empty");
		}
		n.spinOnce();
                return r;
        }


	public double getPowerByPooling() 
	{
		double state = 0;
		
		while (state == 0)
		{
			state = getPower();
                	// System.out.println(res);
                }
                return state;
        }

        /*
         * Test method: call the tabletop_object_detector and print results
         */
        public static void main(String[] args) 
        {

                ROSClient_battery_state d = new ROSClient_battery_state("knowrob_battery_state_test_123");
		int i =1;
		while (i == 1)
		{
			double res = d.getPowerByPooling();
                	System.out.println(res);
                }
        }

}
