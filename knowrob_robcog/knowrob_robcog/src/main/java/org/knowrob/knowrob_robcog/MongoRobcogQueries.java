/*
  Copyright (C) 2014-16 by Andrei Haidu

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Andrei Haidu
  @license BSD
*/

package org.knowrob.knowrob_robcog;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

import java.security.SecureRandom;
import java.lang.StringBuilder;
import javax.vecmath.Vector3d;

import java.io.IOException;
import java.io.Writer;
import java.io.FileWriter;
import java.io.FileOutputStream;
import java.io.BufferedWriter;
import java.io.OutputStreamWriter;
import java.io.File;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.nio.file.Files;

import com.mongodb.DBObject;
import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBList;
import com.mongodb.Cursor;
import com.mongodb.AggregationOptions;

import org.knowrob.vis.MarkerObject;
import org.knowrob.vis.MarkerPublisher;

import visualization_msgs.Marker;
import geometry_msgs.Point;

public class MongoRobcogQueries {
	
	// marker unique ids
	private Deque<String> markerIDs;
	
	// unreal connection to mongodb
	private MongoRobcogConn MongoRobcogConn;

	
	////////////////////////////////////////////////////////////////
	///// MONGO CONNECTION
	/**
	 * MongoRobcogQueries constructor with new mongo connection
	 */	
	public MongoRobcogQueries() {
		// set the connection to unreal
		MongoRobcogConn MongoRobcogConn = new MongoRobcogConn();
		
		// init marker array ids
		this.markerIDs = new ArrayDeque<String>();
	}
	
	/**
	 * MongoRobcogQueries constructor with copied mongo connection
	 */	
	public MongoRobcogQueries(MongoRobcogConn MongoRobcogConn) {
		// set the connection to unreal
		this.MongoRobcogConn = MongoRobcogConn;
		
		// init marker array ids
		this.markerIDs = new ArrayDeque<String>();
	}
	
	
	////////////////////////////////////////////////////////////////
	///// HELPER FUNCTIONS	
	/**
	 * Generate a random string
	 */    
	public String randString(int length) {
		final String str_coll = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
		SecureRandom sec_rand =  new SecureRandom();		
		StringBuilder str_builder = new StringBuilder(length);
		for(int i = 0; i < length; ++i){
			str_builder.append(str_coll.charAt(sec_rand.nextInt(str_coll.length())));
		}
		return str_builder.toString();		
	}
	
	/**
	 * Get color vector from string
	 */    
	public float[] colorFromString (String color) {
	     if(color.equals("blue")){
	    	return new float[] {0.0f, 0.0f, 1.0f, 1.0f};	    	
	     }
	     else if(color.equals("lime")){
	    	 return new float[] {0.0f, 1.0f, 0.0f, 1.0f};
	     }
	     else if(color.equals("green")){
	    	 return new float[] {0.0f, 0.5f, 0.0f, 1.0f};
	     }
	     else if(color.equals("yellow")){
	    	 return new float[] {1.0f, 1.0f, 0.0f, 1.0f};
	     }
	     else if(color.equals("orange")){
	    	 return new float[] {1.0f, 0.65f, 0.0f, 1.0f};
	     }
	     else if(color.equals("cyan")){
	    	 return new float[] {0.0f, 1.0f, 1.0f, 1.0f};
	     }
	     else if(color.equals("purple")){
	    	 return new float[] {0.5f, 0.0f, 0.5f, 1.0f};
	     }
	     else if(color.equals("teal")){
	    	 return new float[] {0.0f, 0.5f, 0.5f, 1.0f};
	     }
	     else if(color.equals("magenta")){
	    	 return new float[] {1.0f, 0.0f, 1.0f, 1.0f};
	     }
	     else if(color.equals("brown")){
	    	 return new float[] {0.65f, 0.16f, 0.16f, 1.0f};
	     }
	     else if(color.equals("gray") || color.equals("grey")){
	    	 return new float[] {0.5f, 0.5f, 0.5f, 1.0f};
	     }
	     else if(color.equals("white")){
	    	 return new float[] {1.0f, 1.0f, 1.0f, 1.0f};
	     }
	     else if(color.equals("black")){
	    	 return new float[] {0.0f, 0.0f, 0.0f, 1.0f};
	     }
	     else if(color.equals("red")){
	    	 return new float[] {1.0f, 0.0f, 0.0f, 1.0f};
	     }
	     else{
	    	 return new float[] {1.0f, 0.0f, 0.0f, 1.0f}; // default red       
	     }
	}
	
	/**
	 * Get marker type vector from string
	 */    
	public byte markerFromString (String type) {
	     if(type.equals("sphere")){
	    	return Marker.SPHERE_LIST;	    	
	     }
	     else if(type.equals("cube")){
	    	 return Marker.CUBE_LIST;	
	     }
	     else if(type.equals("point")){
	    	 return Marker.POINTS;
	     }
	     else{
	    	 return Marker.POINTS; // default points      
	     }
	}
	
	/**
	 * Helper function wich parses String with knowrob time format 'timepoint_%d'
	 */
	private double parseTime_d(String timepoint) {
		String x[] = timepoint.split("timepoint_");
		// Also allow input strings without 'timepoint_' prefix
		String ts = (x.length==1 ? x[0] : x[1]);
		return Double.valueOf(ts.replaceAll("[^0-9.]", ""));
	}
	
	/**
	 * Helper function to create a geometry_msgs.Point
	 */
	private Point msgPoint(double x, double y, double z){
		// TODO make node a class attribute?
		Point p = MarkerPublisher.get().getNode().getTopicMessageFactory().newFromType(Point._TYPE);
		p.setX(x);
		p.setY(y);
		p.setZ(z);
		return p;
	}
	
	/**
	 * Helper function to create a geometry_msgs.Point
	 */
	private Point msgPoint(Vector3d v){
		// TODO make node a class attribute?
		Point p = MarkerPublisher.get().getNode().getTopicMessageFactory().newFromType(Point._TYPE);
		p.setX(v.x);
		p.setY(v.y);
		p.setZ(v.z);
		return p;
	}
	
	
	////////////////////////////////////////////////////////////////
	///// MARKER FUNCTIONS	
	/**
	 * Create the rviz markers
	 */
	public void CreateMarkers(ArrayList<Vector3d> pointsArr, String markerID, String markerType, String color, float scale){
		// List of marker points
		List<Point> marker_points = new ArrayList<Point>();
		
		// iterate the 3d vector to create the marker points
		for (Vector3d p_iter : pointsArr){
			marker_points.add(this.msgPoint(p_iter));
		}

		// check if marker already exists
		MarkerObject m = MarkerPublisher.get().getMarker(markerID);
		if(m==null) {			
			// create marker
			m = MarkerPublisher.get().createMarker(markerID);
			// set the type of the marker
			m.setType(markerFromString(markerType));
			// set the positions of the list markers
			m.getMessage().setPoints(marker_points);
			// transform string color to float[] array
			m.setColor(this.colorFromString(color));
			// set the scale of the marker
			m.setScale(new float[] {scale, scale, scale});
		}
		// add ID to the marker container
		this.markerIDs.add(markerID);
	}
	
	/**
	 * Create the rviz mesh marker
	 */
	public void CreateMeshMarker(double pose[], String markerID, String meshPath){		
		// split pose into translation and orientation
		final double[] translation = new double[] {pose[0], pose[1], pose[2]};
		final double[] orientation = new double[] {pose[3], pose[4], pose[5], pose[6]};		
		
		// check if marker already exists
		MarkerObject m = MarkerPublisher.get().getMarker(markerID);
		if(m==null) {
			// create marker
			m = MarkerPublisher.get().createMarker(markerID);
			// set the type of the marker
			m.setType(Marker.MESH_RESOURCE);
			// set the path to the mesh
			m.setMeshResource(meshPath);
			// set pos and rotation
			m.setTranslation(translation);
			m.setOrientation(orientation);
			// set scale
			m.setScale(new float[] {1.0f,1.0f,1.0f});
		}
		// add ID to the marker container
		this.markerIDs.add(markerID);
	}

	/**
	 * Create the bones rviz mesh marker
	 */
	public void CreateBonesMeshMarkers(
			double[][] poses,
			String[] names,
			String markerID,
			String meshFolderPath){
		// create marker for every link mesh
		for (int i = 0; i < names.length; ++i)
		{
			// split pose into translation and orientation
			final double[] translation = new double[] {poses[i][0], poses[i][1], poses[i][2]};
			final double[] orientation = new double[] {poses[i][3], poses[i][4], poses[i][5], poses[i][6]};
			
			final String curr_name = names[i];
			final String curr_id = markerID + curr_name;
			// check if marker already exists (ID + link names)
			MarkerObject m = MarkerPublisher.get().getMarker(curr_id);
			if(m==null) {
				// create marker
				m = MarkerPublisher.get().createMarker(curr_id);
				// set the type of the marker
				m.setType(Marker.MESH_RESOURCE);
				// set the path to the mesh
				m.setMeshResource(meshFolderPath + curr_name + ".dae");
				// set pos and rotation
				m.setTranslation(translation);
				m.setOrientation(orientation);
				// set scale
				m.setScale(new float[] {1.0f,1.0f,1.0f});
			}
			// add ID to the marker container
			this.markerIDs.add(curr_id);		
		}
	}
	
	/**
	 * Remove the rviz marker with the given ID
	 */
	public void RemoveMarker(String markerID){
		MarkerPublisher.get().eraseMarker(markerID);		
	}
	
	/**
	 * Remove all rviz markers created form sg
	 */
	public void RemoveAllMarkers(){
		// Iterate and remove all markers
        Iterator m_itr = this.markerIDs.iterator();
        while (m_itr.hasNext()) {
                this.RemoveMarker((String)m_itr.next());
        }
	}

	
	////////////////////////////////////////////////////////////////
	///// GET QUERY FUNCTIONS
	/**
	 * Query the Pose of the actor at the given timepoint (or the most recent one)
	 */
	public double[] GetActorPoseAt(String actorName,  String timestampStr){
		// transform the knowrob time to double with 3 decimal precision
		final double timestamp = (double) Math.round(parseTime_d(timestampStr) * 1000) / 1000;

		return GetActorPoseAt(actorName, timestamp);
	}
	
	public double[] GetActorPoseAt(String actorName, double timestamp){
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the actor name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("actors.name", actorName));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind actors in order to output only the queried actor
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$actors.pos");
		proj_fields.put("rot", "$actors.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_actors, match_actor, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);

		// if query has a response, return the pose
		if(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject first_doc = (BasicDBObject) cursor.next();			
			// close cursor
			cursor.close();
			// get the pose
			return new double[] {
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z"),
					((BasicDBObject) first_doc.get("rot")).getDouble("w"),
					((BasicDBObject) first_doc.get("rot")).getDouble("x"),
					((BasicDBObject) first_doc.get("rot")).getDouble("y"),
					((BasicDBObject) first_doc.get("rot")).getDouble("z")};
		}
		else
		{
			System.out.println("Java - GetActorPose - No results found, returning empty list..");			
			return new double[0];
		}
	}

	/**
	 * Query the Traj of the actor between the given timepoints
	 */
	public double[][] GetActorTraj(String actorName,
			String start,
			String end,
			double deltaT){
		// transform the knowrob time to double with 3 decimal precision
		final double start_ts = (double) Math.round(parseTime_d(start) * 1000) / 1000;
		final double end_ts = (double) Math.round(parseTime_d(end) * 1000) / 1000;
		
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the actors
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$actors.pos");
		proj_fields.put("rot", "$actors.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_actors, match_actor, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<double[]> traj_list = new ArrayList<double[]>();
		
		// if the query returned nothing, get the most recent pose
		if(!cursor.hasNext())
		{
			System.out.println("Java - GetActorTraj - No results found, returning most recent pose..");
			// get the most recent pose
			traj_list.add(this.GetActorPoseAt(actorName, start));
			
			// cast from dynamic array to standard array
			return traj_list.toArray(new double[traj_list.size()][7]);	
		}
		
		// timestamp used for deltaT
		double prev_ts = 0;
				
		// while query has a response, return the pose
		while(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			// get the curr timestamp
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{			
				// get the current pose
				traj_list.add(new double[] {
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("w"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("x"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("y"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("z")});
				prev_ts = curr_ts;
				//System.out.println(curr_doc.toString());
			}
		}
		// close cursor
		cursor.close();		
		
		// cast from dynamic array to standard array
		return traj_list.toArray(new double[traj_list.size()][7]);
	}
	
	/**
	 * Query the Pose of the actors bone at the given timepoint (or the most recent one)
	 */
	public double[] GetBonePoseAt(String actorName, String boneName, String timestampStr){
		final double timestamp = (double) Math.round(parseTime_d(timestampStr) * 1000) / 1000;
		
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the actor name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("actors.name", actorName));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind actors in order to output only the queried actor
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_bone_fields = new BasicDBObject("_id", 0);
		proj_bone_fields.put("timestamp", 1);
		proj_bone_fields.put("actors.bones", 1);
		DBObject project_bones = new BasicDBObject("$project", proj_bone_fields);

		
		// $unwind the bones
		DBObject unwind_bones = new BasicDBObject("$unwind", "$actors.bones");

		// $match for the given bone name from the unwinded bones
		DBObject match_bone = new BasicDBObject(
				"$match", new BasicDBObject("actors.bones.name", boneName));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$actors.bones.pos");
		proj_fields.put("rot", "$actors.bones.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_actors, match_actor,
				project_bones, unwind_bones, match_bone, project);
				

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);

		// if query has a response, return the pose
		if(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject first_doc = (BasicDBObject) cursor.next();	
			// close cursor since we only care about one value
			cursor.close();
			// get the pose
			return new double[] {
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z"),
					((BasicDBObject) first_doc.get("rot")).getDouble("w"),
					((BasicDBObject) first_doc.get("rot")).getDouble("x"),
					((BasicDBObject) first_doc.get("rot")).getDouble("y"),
					((BasicDBObject) first_doc.get("rot")).getDouble("z")};
		}
		else
		{
			System.out.println("Java - GetBonePose - No results found, returning empty list..");
			return new double[0];
		}
	}
	
	/**
	 * Query the Traj of the actors bone between the given timepoints
	 */
	public double [][] GetBoneTraj(String actorName,
			String boneName,
			String start,
			String end,
			double deltaT){		
		// transform the knowrob time to double with 3 decimal precision
		final double start_ts = (double) Math.round(parseTime_d(start) * 1000) / 1000;
		final double end_ts = (double) Math.round(parseTime_d(end) * 1000) / 1000;
		
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the actors
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$actors.pos");
		proj_fields.put("rot", "$actors.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_actors, match_actor, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<double[]> traj_list = new ArrayList<double[]>();	
		
		// if the query returned nothing, get the most recent pose
		if(!cursor.hasNext())
		{
			System.out.println("Java - GetBoneTraj - No results found, returning most recent pose..");
			// get the most recent pose
			traj_list.add(this.GetBonePoseAt(actorName, boneName, start));
			
			// cast from dynamic array to standard array
			return traj_list.toArray(new double[traj_list.size()][7]);	
		}
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// if query has a response, return the pose
		while(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();			
			
			// get the curr timestamp
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				// get the current pose
				traj_list.add(new double[] {
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("w"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("x"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("y"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("z")});
				prev_ts = curr_ts;
				//System.out.println(curr_doc.toString());
			}
		}
		// close cursor
		cursor.close();		
		
		// cast from dynamic array to standard array
		return traj_list.toArray(new double[traj_list.size()][7]);
	}
	
	/**
	 * Query the Names of the actor bones
	 */
	public String[] GetBonesNames(String actorName){
		// create the pipeline operations, first the $match
		DBObject match_name = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName)); 

		// $limit the result to 1
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind actors in order to output only the queried actor
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("bones_names", "$actors.bones.name");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_name, limit_result, unwind_actors, match_actor, project);
				

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);
		
		// if query has a response, return the names
		if(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject first_doc = (BasicDBObject) cursor.next();	
			// close cursor since we only care about one value
			cursor.close();
			// get the bone names as list			
			BasicDBList names = (BasicDBList) first_doc.get("bones_names");			
			// return as array of string
			return names.toArray(new String[names.size()]);
		}
		else // else return empty list
		{
			System.out.println("Java - GetBonesNames - No results found, returning empty list..");
			return new String[0];
		}
	}
	
	/**
	 * Query the Poses of the actor bones at the given timepoint (or the most recent one)
	 */
	public double[][] GetBonesPosesAt(String actorName, String timestampStr){
		// transform the knowrob time to double with 3 decimal precision
		final double timestamp = (double) Math.round(parseTime_d(timestampStr) * 1000) / 1000;
		
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the actor name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("actors.name", actorName));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind actors in order to output only the queried actor
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("bones_pos", "$actors.bones.pos");
		proj_fields.put("bones_rot", "$actors.bones.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_actors, match_actor, project);
				

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);

		// Poses as dynamic array
		ArrayList<double[]> pose_list = new ArrayList<double[]>();	
		
		// if query has a response, return the pose
		if(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject first_doc = (BasicDBObject) cursor.next();	
			// close cursor since we only care about one value
			cursor.close();
			// get the pose
			
			BasicDBList pos_list = (BasicDBList) first_doc.get("bones_pos");
			BasicDBList rot_list = (BasicDBList) first_doc.get("bones_rot");
			
			// pos_list and rot_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{				
				pose_list.add(new double[]{
					((BasicDBObject) pos_list.get(i)).getDouble("x"),
					((BasicDBObject) pos_list.get(i)).getDouble("y"),
					((BasicDBObject) pos_list.get(i)).getDouble("z"),
					((BasicDBObject) rot_list.get(i)).getDouble("w"),
					((BasicDBObject) rot_list.get(i)).getDouble("x"),
					((BasicDBObject) rot_list.get(i)).getDouble("y"),
					((BasicDBObject) rot_list.get(i)).getDouble("z")});		
			}
			// cast from dynamic array to standard array
			return pose_list.toArray(new double[pose_list.size()][7]);
		}
		else
		{
			System.out.println("Java - GetBonesPoses - No results found, returning empty list..");
			return new double[0][0];
		}
	}
	
	/**
	 * Query the Trajectories of the actor bones at the given timepoint (or the most recent one)
	 */
	public double[][][] GetBonesTrajs(String actorName,
			String start,
			String end,
			double deltaT){	
		// transform the knowrob time to double with 3 decimal precision
		final double start_ts = (double) Math.round(parseTime_d(start) * 1000) / 1000;
		final double end_ts = (double) Math.round(parseTime_d(end) * 1000) / 1000;
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind actors in order to output only the queried actor
		DBObject unwind_actors = new BasicDBObject("$unwind", "$actors");

		// $match for the given actor name from the unwinded actors
		DBObject match_actor = new BasicDBObject(
				"$match", new BasicDBObject("actors.name", actorName));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("bones_pos", "$actors.bones.pos");
		proj_fields.put("bones_rot", "$actors.bones.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_actors, match_actor, project);				

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		// get results
		Cursor cursor = this.MongoRobcogConn.coll.aggregate(pipeline, aggregationOptions);

		// Trajectories as dynamic array (dynamic on the time part)
		ArrayList<double[][]> bone_trajs = new ArrayList<double[][]>();
		
		// the number of bones
		int nr_bones = 0;
		
		// if the query returned nothing, get the most recent pose
		if(!cursor.hasNext())
		{
			System.out.println("Java - GetBonesTrajs - No results found, returning most recent poses..");
			// get the most recent pose
			bone_trajs.add(this.GetBonesPosesAt(actorName, start));
			
			// set the nr of bones
			nr_bones = bone_trajs.get(0).length;
			
			// cast from dynamic array to standard array
			return bone_trajs.toArray(new double[bone_trajs.size()][nr_bones][7]);
		}
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// if query has a response, return the pose
		while(cursor.hasNext())
		{
			// get the first document as the next cursor and append the metadata to it
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();

			// get the curr timestamp
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				// get the list of bones pos and rot
				BasicDBList pos_list = (BasicDBList) curr_doc.get("bones_pos");
				BasicDBList rot_list = (BasicDBList) curr_doc.get("bones_rot");
				
				// set the nr of bones
				nr_bones = pos_list.size();
				
				// Poses as dynamic array (dynamic on the nr of bones part)
				ArrayList<double[]> pose_list = new ArrayList<double[]>();
				
				// pos_list and rot_list length should be always the same
				for (int i = 0; i < nr_bones; ++i)
				{				
					pose_list.add(new double[]{
							((BasicDBObject) pos_list.get(i)).getDouble("x"),
							((BasicDBObject) pos_list.get(i)).getDouble("y"),
							((BasicDBObject) pos_list.get(i)).getDouble("z"),
							((BasicDBObject) rot_list.get(i)).getDouble("w"),
							((BasicDBObject) rot_list.get(i)).getDouble("x"),
							((BasicDBObject) rot_list.get(i)).getDouble("y"),
							((BasicDBObject) rot_list.get(i)).getDouble("z")});
				}
				// cast from dynamic array to standard array
				bone_trajs.add(pose_list.toArray(new double[nr_bones][7]));				
				prev_ts = curr_ts;
			}
		}
		// close cursor
		cursor.close();	
		
		// return the actor bones trajectories as float[][][] 
		return bone_trajs.toArray(new double[bone_trajs.size()][nr_bones][7]);
	}

	
	////////////////////////////////////////////////////////////////
	///// VIS QUERY FUNCTIONS	
	/**
	 * View and return the Pose of the actor at the given timepoint (or the most recent one)
	 */
	public void ViewActorPoseAt(String actorName, 
			String timestampStr,
			String markerType,
			String color,
			float scale){
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewActorPoseAt(actorName, timestampStr, marker_id, markerType, color, scale);
	}
	
	public void ViewActorPoseAt(String actorName, 
			String timestampStr, 
			String markerID,
			String markerType,
			String color,
			float scale){
		// get the pose of the actor
		final double[] pose = this.GetActorPoseAt(actorName, timestampStr);
	
		// return position as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
		pos.add(new Vector3d(pose[0],pose[1],pose[2]));
		
		// create the marker
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}
	
	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * view results as rviz mesh marker 
	 */
	public void ViewActorMeshAt(String actorName,			
			String timestampStr,
			String meshPath){
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewActorMeshAt(actorName, timestampStr, marker_id, meshPath);
	}
	
	public void ViewActorMeshAt(String actorName,			
			String timestampStr, 
			String markerID,
			String meshPath){
		// get the pose of the actor
		final double[] pose = this.GetActorPoseAt(actorName, timestampStr);
		
		// create mesh marker
		this.CreateMeshMarker(pose, markerID, meshPath);
	}

	/**
	 * Get the poses of the actorss bones meshes at the given timestamp
	 * view results as rviz markers
	 */
	public void ViewBonesMeshesAt(String actorName,
			String timestampStr,
			String meshFolderPath){	
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewBonesMeshesAt(actorName, timestampStr, marker_id, meshFolderPath);
	}
	
	public void ViewBonesMeshesAt(String actorName,
			String timestampStr,
			String markerID,
			String meshFolderPath){		
		// get the names of the bones
		final String[] names = this.GetBonesNames(actorName); 
	
		// pos xyz rot wxyz for every bone
		final double[][] bone_poses = this.GetBonesPosesAt(actorName, timestampStr);

		// create the bones mesh markers
		this.CreateBonesMeshMarkers(bone_poses, names, markerID, meshFolderPath);
	}

	/**
	 * View and return the Pose of the actors bone at the given timepoint (or the most recent one)
	 */
	public void ViewBonePoseAt(String actorName,
			String boneName,
			String timestampStr,
			String markerType,
			String color,
			float scale){	
		// gen id
		final String marker_id = boneName + "_" + this.randString(4);
		// create the marker
		this.ViewBonePoseAt(actorName, boneName, timestampStr, marker_id, markerType, color, scale);
	}
	
	public void ViewBonePoseAt(String actorName,
			String boneName,
			String timestampStr, 
			String markerID,
			String markerType,
			String color,
			float scale){	
		// get the pose of the actor
		final double[] pose = this.GetBonePoseAt(actorName, boneName, timestampStr);
	
		// return position as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
		pos.add(new Vector3d(pose[0],pose[1],pose[2]));
		
		// create the markers
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}
	
	/**
	 * View and return the Traj of the actor between the given timepoints
	 */
	public void ViewActorTraj(String actorName,
			String start,
			String end,
			String markerType,
			String color,
			float scale,
			double deltaT){	
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewActorTraj(actorName, start, end, marker_id, markerType, color, scale, deltaT);
	}
	
	public void ViewActorTraj(String actorName,
			String start,
			String end,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){		
		// get the trajectory
		final double[][] traj = this.GetActorTraj(actorName, start, end, deltaT);

		// positions as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();

		// add trajectory points
		for (int i = 0; i < traj.length; i++){
			pos.add(new Vector3d(traj[i][0], traj[i][1], traj[i][2]));
		}
		// create the markers
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}

	/**
	 * View and return the Pose of the actor between the given timepoints
	 */
	public void ViewBoneTraj(String actorName,
			String boneName,
			String start,
			String end,
			String markerType,
			String color,
			float scale,
			double deltaT){
		// gen id
		final String marker_id = boneName + "_" + this.randString(4);
		// create the marker
		this.ViewBoneTraj(actorName, boneName, start, end, marker_id, markerType, color, scale, deltaT);
	}
	
	public void ViewBoneTraj(String actorName,
			String boneName,
			String start,
			String end,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){
		// get the trajectory
		final double[][] traj = this.GetBoneTraj(actorName, boneName, start, end, deltaT);
		
		// positions as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
		
		// add trajectory points
		for (int i = 0; i < traj.length; i++){
			pos.add(new Vector3d(traj[i][0], traj[i][1], traj[i][2]));
		}	
	
		// create the markers
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}

	/**
	 * View and return the Poses of the actor bones at the given timepoint (or the most recent one)
	 */
	public void ViewBonesPoses(String actorName,
			String timestampStr,
			String markerType,
			String color,
			float scale){
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewBonesPoses(actorName, timestampStr, marker_id, markerType, color, scale);
	}

	public void ViewBonesPoses(String actorName,
			String timestampStr,
			String markerID,
			String markerType,
			String color,
			float scale){
		// get the bones poses
		final double[][] poses = this.GetBonesPosesAt(actorName, timestampStr);
		
		// positions as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
		
		// add bones points
		for (int i = 0; i < poses.length; i++){
			pos.add(new Vector3d(poses[i][0], poses[i][1], poses[i][2]));
		}
	
		// create the markers
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}

	/**
	 * View and return the Trajectories of the actor bones at the given timepoint (or the most recent one)
	 */
	public void ViewBonesTrajs(String actorName,
			String start,
			String end,
			String markerType,
			String color,
			float scale,
			double deltaT){
		// gen id
		final String marker_id = actorName + "_" + this.randString(4);
		// create the marker
		this.ViewBonesTrajs(actorName, start, end, marker_id, markerType, color, scale, deltaT);
	}
	
	public void ViewBonesTrajs(String actorName,
			String start,
			String end,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){
		// call further using double for timestamps
		final double[][][] trajs = this.GetBonesTrajs(actorName, start, end, deltaT);
		
		// positions as arraylist
		ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
		
		// add trajectory points to the marker
		for (int i = 0; i < trajs.length; i++){
			for(int j = 0; j < trajs[i].length; j++){				
				pos.add(new Vector3d(trajs[i][j][0], trajs[i][j][1], trajs[i][j][2]));
			}
		}
		
		// create the markers
		this.CreateMarkers(pos, markerID, markerType, color, scale);
	}

	////////////////////////////////////////////////////////////////
	///// ADD RATING
	public void AddRating(String RatingInst,
			String RatingType,
			String Score,
			String FilePath){
		// split strings from namspace / add unique hash
		final String rating_inst = RatingInst.split("#")[1];
		final String rating_type = RatingType.split("#")[1];
		final String rating_type_inst = rating_type + "_" + this.randString(4);		
		final String rating_str = 
				"\t<!-- Object: " + rating_type_inst + "-->\n" + 
				"\t<owl:NamedIndividual rdf:about=\"&log;" + rating_type_inst + "\">\n" +
				"\t\t<rdf:type rdf:resource=\"&knowrob_u;" + rating_type +"\"/>\n" +
				"\t\t<knowrob_u:ratingScore rdf:datatype=\"&xsd; float\">" + Score + "</knowrob_u:ratingScore>\n" +
				"\t\t<knowrob_u:ratingOf rdf:resource=\"&log;" + rating_inst +"\"/>\n" +
				"\t</owl:NamedIndividual>";

		// append rating to file
		try {
			// remove last line of the file
			RandomAccessFile randomAccessFile = new RandomAccessFile(FilePath, "rw");
			byte b;
			long length = randomAccessFile.length() ;
			if (length != 0) {
				do {
					length -= 1;
					randomAccessFile.seek(length);
					b = randomAccessFile.readByte();
				} while (b != 10 && length > 0);
				randomAccessFile.setLength(length);
				randomAccessFile.close();
			}		
			// append rating string to file
			PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(FilePath, true)));
			out.println(rating_str);
			out.print("\n</rdf:RDF>");
			out.close();
		} catch (IOException e) {
		}
	}
}

