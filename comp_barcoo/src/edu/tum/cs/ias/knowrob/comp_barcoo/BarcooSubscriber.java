/*
 * Copyright (c) 2010 Nacer Khalil
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
package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;
import ros.roscpp.*;
import ros.pkg.comp_barcoo.srv.*;


public class BarcooSubscriber{
	
	//private String topic = "/barcode_image_node/barcode";
	//private Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	private BarcooImporter barcooImporter;
	private BarcooKnowrobLoader knowrobLoader;
	
	public BarcooSubscriber(final String barcooOWLPath) throws Exception
	{		
		
		final Ros ros = Ros.getInstance();		
		ros.init("BarcooService");				
		NodeHandle n = ros.createNodeHandle();
		barcooImporter = new BarcooImporter(barcooOWLPath);
		knowrobLoader = new BarcooKnowrobLoader(n,"knowrobTopic");
		
		ServiceServer.Callback<send_barcode.Request,send_barcode.Response> scb = new ServiceServer.Callback<send_barcode.Request,send_barcode.Response>() 
		{
            public send_barcode.Response call(send_barcode.Request request) {
            	send_barcode.Response res = new send_barcode.Response();
            	if(request.knowrob == 1)
            	{
            		String indName = knowrobLoader.passToKnowrob(request.barcode.data);
            		res.recieved.data = indName;                 
            		return res;
            	}
            	
            	else if(request.knowrob == 0)
            	{
            		String indName = barcooImporter.createNewIndividual(request.barcode.data);
            		res.recieved.data = indName;                 
            		return res;
            	}
            	else
            	{
            		res.recieved.data = "Wrong knowrob attribute value";                 
            		return res;
            	}
            }
		};            
		
        ServiceServer<send_barcode.Request,send_barcode.Response,send_barcode> srv = n.advertiseService("send_barcode", new send_barcode(), scb);
        
        ros.logInfo("Ready to receive barcode and create new individuals"); 
        System.out.println("LOADING COMPLETED.");
		n.spin();	
	}	
	
	public static void main(String[] args)
	{		
		try
		{
			if(args == null || args.length != 1 || args[0].equalsIgnoreCase("help"))
			{
				System.out.println("Usage: barcoo_subscriber <barcoo_ontology_file>.owl");				
				System.exit(0);
			}			
			System.out.println("PLEASE WAIT.\n PROGRAM LOADING...");
			
			BarcooSubscriber subscriber = new BarcooSubscriber(args[0]);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
}
