/*
 * ROSClient_low_level.java
 * Copyright (c) 2013 Asil Kaan Bozcuoglu, 2015 Daniel Beßler
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
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
import java.lang.Integer;

import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.*;
import org.knowrob.interfaces.mongo.types.*;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.mongodb.QueryBuilder;

import javax.vecmath.Vector3d;

import designator_integration_msgs.KeyValuePair;

import java.io.File;
import java.util.LinkedList;


public class LogdataPublisher extends AbstractNodeMain {
	MongoDBInterface mdb;

	ConnectedNode node;
	Publisher<designator_integration_msgs.Designator> pub;
	Publisher<std_msgs.String> pub_image;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_log_publisher");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		mdb = new MongoDBInterface();

		pub = connectedNode.newPublisher("logged_designators", designator_integration_msgs.Designator._TYPE);
		pub_image = connectedNode.newPublisher("logged_images", std_msgs.String._TYPE); 
	}
	
	public boolean waitOnPublisher() {
		try {
			while(pub_image ==null) {
				Thread.sleep(200);
			}
			return true;
		} catch (InterruptedException e) {
			e.printStackTrace();
			return false;
		}
	}
	
	public boolean publishImage(String image_path) {
		// wait for publisher to be ready
		if(!waitOnPublisher()) return false;

		// create image message from file
		final std_msgs.String image_msg = pub_image.newMessage();

		image_msg.setData(image_path);
		pub_image.publish(image_msg);
		return true;
	}

	public boolean publishDesignator(String designatorId) {
		try {
			final StringTokenizer s1 = new StringTokenizer(designatorId, "#");
			s1.nextToken();
			designatorId= s1.nextToken();
			
			QueryBuilder query = QueryBuilder.start("designator._id").is(designatorId);
			org.knowrob.interfaces.mongo.types.Designator d1 = mdb.designator(mdb.one(
					mdb.query(MongoDBInterface.COLLECTION_LOGGED_DESIGNATORS, query)));
			publishDesignator(d1);
			return true;
		}
		catch (Exception e) {
			System.err.println("Failed to publish designator: " + e.getMessage());
			e.printStackTrace();
		}
		return false;
	}

	public boolean publishDesignator(org.knowrob.interfaces.mongo.types.Designator designator) {
		try {
			// wait for publisher to be ready
			if(!waitOnPublisher()) return false;
	
			final designator_integration_msgs.Designator designator_msg = pub.newMessage();

			designator_msg.setType(0);
			try {
				if(designator.getType().toString().toLowerCase() == "action")
					designator_msg.setType(1);
				else if(designator.getType().toString().toLowerCase() == "object")
					designator_msg.setType(2);
				else if(designator.getType().toString().toLowerCase() == "location")
					designator_msg.setType(0);			
			}
			catch (java.lang.NullPointerException exc) {}
	
			publishDesignator(designator, designator_msg, 0);
			
			pub.publish(designator_msg);
	
			return true;
		}
		catch (Exception e) {
			System.err.println("Failed to publish designator: " + e.getMessage());
			e.printStackTrace();
		}
		return false;
	}
	
	private void publishDesignator(Designator designator, designator_integration_msgs.Designator designator_msg, int level) {
		for(String key : designator.keySet()) {
			// check if publishable key
			if(key.isEmpty()) continue;
			if(key.substring(0,1).equals("_")) continue;
			Object value = designator.get(key);
			if(value==null) {
				System.err.println("Designator null value for key: " + key);
				continue;
			}

			KeyValuePair kv = node.getTopicMessageFactory().newFromType(
					designator_integration_msgs.KeyValuePair._TYPE);
			designator_msg.getDescription().add(kv);
			int c = designator_msg.getDescription().size()-1;
			
			designator_msg.getDescription().get(c).setId(c);
			designator_msg.getDescription().get(c).setKey(key);
			designator_msg.getDescription().get(c).setParent(level);

			if (value instanceof Double) 
			{
				designator_msg.getDescription().get(c).setType(1);
				Double current_value = (Double)value;
				designator_msg.getDescription().get(c).setValueFloat((float)current_value.doubleValue());						
			}
			else if (value instanceof Integer) 
			{
				designator_msg.getDescription().get(c).setType(1);
				Double current_value = (Double)value;
				designator_msg.getDescription().get(c).setValueFloat((float)current_value.doubleValue());
			}
			else if (value instanceof ISODate) 
			{
				designator_msg.getDescription().get(c).setType(0);
				ISODate date = (ISODate)value;
				designator_msg.getDescription().get(c).setValueString(date.toString());
			}
			else if (value instanceof Designator) 
			{
				Designator inner_designator = (Designator)value;
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
				publishDesignator(inner_designator, designator_msg, level+1);
			}
			else if (value instanceof PoseStamped) 
			{
				designator_msg.getDescription().get(c).setType(4);
				PoseStamped pose = (PoseStamped) value;
				designator_msg.getDescription().get(c).setValuePosestamped(pose);
			}
			else if (value instanceof geometry_msgs.Pose) 
			{
				designator_msg.getDescription().get(c).setType(5);
				geometry_msgs.Pose pose = (geometry_msgs.Pose) value;
				designator_msg.getDescription().get(c).setValuePose(pose);
			}
			else if (value instanceof Matrix4d)
			{
				handleMatrixValue(designator_msg, c, (Matrix4d) value);
			}
			else if (value instanceof Vector3d)
			{
				handleVectorValue(designator_msg, c, (Vector3d) value);
			}
			else if (value instanceof tfjava.Stamped<?>)
			{
				Object innerVal = ((tfjava.Stamped<?>)value).getData();
				if (innerVal instanceof Matrix4d)
					handleMatrixValue(designator_msg, c, (Matrix4d) innerVal);
				else if (innerVal instanceof Vector3d)
					handleVectorValue(designator_msg, c, (Vector3d) innerVal);
			}
			else
			{
				designator_msg.getDescription().get(c).setType(0);
				designator_msg.getDescription().get(c).setValueString(value.toString());
			}
		}
	}

	private void handleVectorValue(designator_integration_msgs.Designator designator_msg, int c, Vector3d vec) {
		double val[] = new double[3];
		vec.get(val);
		designator_msg.getDescription().get(c).setType(13);
		designator_msg.getDescription().get(c).setValueArray(val);
	}

	private void handleMatrixValue(designator_integration_msgs.Designator designator_msg, int c, Matrix4d mat) {
		double val[] = {
			mat.m00, mat.m01, mat.m02, mat.m03,
			mat.m10, mat.m11, mat.m12, mat.m13,
			mat.m20, mat.m21, mat.m22, mat.m23,
			mat.m30, mat.m31, mat.m32, mat.m33
		};
		designator_msg.getDescription().get(c).setType(12);
		designator_msg.getDescription().get(c).setValueArray(val);
	}
	
	// TODO: move to openease package
	public static String[] getVideoURLs(String cat, String exp)
	{
		LinkedList<String> urls = new LinkedList<String>();
		File expDir = new File("/episodes/"+cat+"/"+exp);
		if(expDir.exists()) {
			for (final File episodeDir : expDir.listFiles()) {
				if(!episodeDir.isDirectory()) continue;
				File videoDir = new File(episodeDir, "videos");
				if(videoDir.exists()) {
					for (final File vidFile : videoDir.listFiles()) {
						urls.add("/knowrob/knowrob_data/"+cat+"/"+exp+"/"+episodeDir.getName()+"/videos/"+vidFile.getName());
					}
				}
				for (final File vidFile : episodeDir.listFiles()) {
					if(vidFile.isDirectory()) continue;
					String ext = vidFile.getName().substring(vidFile.getName().indexOf(".") + 1);
					if (ext.equalsIgnoreCase("avi") ||
					    ext.equalsIgnoreCase("mpg") ||
					    ext.equalsIgnoreCase("mp4") ||
					    ext.equalsIgnoreCase("mpeg") ||
					    ext.equalsIgnoreCase("flv") ||
					    ext.equalsIgnoreCase("mov") ||
					    ext.equalsIgnoreCase("mkv")) {
						urls.add("/knowrob/knowrob_data/"+cat+"/"+exp+"/"+episodeDir.getName()+"/" + vidFile.getName());
					}
				}
			}
		}
		if(urls.isEmpty())
			return null;
		else
			return urls.toArray(new String[urls.size()]);
	}

}
