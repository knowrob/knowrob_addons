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

package org.knowrob.knowrob_sim_games;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.Deque;
import java.util.ArrayDeque;
import java.util.Map;
import java.util.Set;
import java.util.Iterator;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import java.net.UnknownHostException;
import com.mongodb.MongoClient;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBObject;
import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBList;
import com.mongodb.Cursor;
import com.mongodb.AggregationOptions;

import weka.attributeSelection.AttributeSelection;
import weka.attributeSelection.PrincipalComponents;
import weka.attributeSelection.Ranker;
import weka.core.FastVector;
import weka.core.SparseInstance;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Attribute;
import weka.core.Utils;

// import org.knowrob.vis.MarkerObject;
// import org.knowrob.vis.MarkerPublisher;
import visualization_msgs.Marker;
import geometry_msgs.Point;

public class MongoSimGames {

	private static final int TIME_OFFSET = 0;

	private MongoClient mongoClient;
	private DB db;
	private DBCollection coll;	
	private String dbHost;
	private Deque<String> markerIDs;
	
	/**
	 * MongoSimGames constructor
	 */
	public MongoSimGames() {		
		
		// init marker array ids
		this.markerIDs = new ArrayDeque<String>();
		
		// set the host name (default)
		this.dbHost = "localhost";
		int port = 27017;
		
		// check if MONGO_PORT_27017_TCP_ADDR and MONGO_PORT_27017_TCP_PORT 
		// environment variables are set
		
        Map<String, String> env = System.getenv();
        if(env.containsKey("MONGO_PORT_27017_TCP_ADDR")) {
        	this.dbHost = env.get("MONGO_PORT_27017_TCP_ADDR");
        }
        
        if(env.containsKey("MONGO_PORT_27017_TCP_PORT")) {
        	port = Integer.valueOf(env.get("MONGO_PORT_27017_TCP_PORT"));
        }
	
		try {
			// create a new DB client
			this.mongoClient = new MongoClient(this.dbHost , port);

		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}

	
	////////////////////////////////////////////////////////////////
	///// HELPER FUNCTIONS	
	/**
	 * Get quaternion from euler angles
	 */    
	public double[] quatFromEulerAngles (double x, double y, double z) {
		final double pi_180 = Math.PI / 180;				
		return this.quatFromEulerRad((x*pi_180), (y*pi_180), (z*pi_180));
	}
	
	/**
	 * Get quaternion from euler radians
	 */    
	public double[] quatFromEulerRad (double x, double y, double z) {
		final double phi = x / 2.0;
		final double the = y / 2.0;
		final double psi = z / 2.0;

		// save the sin/cos
		final double cos_phi = Math.cos(phi);
		final double sin_phi = Math.sin(phi);
		final double cos_the = Math.cos(the);
		final double sin_the = Math.sin(the);
		final double cos_psi = Math.cos(psi);
		final double sin_psi = Math.sin(psi);

		// save repeated computation
		final double c_phi_c_the = cos_phi * cos_the;
		final double s_phi_c_the = sin_phi * cos_the;
		final double c_phi_s_the = cos_phi * sin_the;
		final double s_phi_s_the = sin_phi * sin_the;

		// compute the quaternion
		final double q_w = c_phi_c_the * cos_psi + s_phi_s_the * sin_psi;
		final double q_x = s_phi_c_the * cos_psi - c_phi_s_the * sin_psi;
		final double q_y = c_phi_s_the * cos_psi + s_phi_c_the * sin_psi;
		final double q_z = c_phi_c_the * sin_psi - s_phi_s_the * cos_psi;
		
		return new double[] {q_w, q_x, q_y, q_z};
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
	 * Parses String with common time format 'timepoint_%d'
	 * and returns a double precision number that represents
	 * the time passed since 1970.
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
	private Point msgPoint(Vector3d v){
// 		// TODO make node a class attribute?
// 		Point p = MarkerPublisher.get().getNode().getTopicMessageFactory().newFromType(Point._TYPE);
// 		p.setX(v.x);
// 		p.setY(v.y);
// 		p.setZ(v.z);
// 		return p;
  return null;
	}

	
	////////////////////////////////////////////////////////////////
	///// MONGO FUNCTIONS	
	/**
	 * Set the database to be queried
	 */
	public void SetDatabase(String dbName){

		this.db = mongoClient.getDB(dbName);

		System.out.println("Java - Db: " + this.db.getName());
		
		Set<String> all_collections = this.db.getCollectionNames();
		
		for (String coll_name : all_collections)
		{
			System.out.println("\t Coll: " + coll_name);
		}
	}

	/**
	 * Set the collection to be queried
	 */
	public void SetCollection(String collName){
		// get the collection
		this.coll = this.db.getCollection(collName);

		//System.out.println("Java - Coll: " + this.coll.getName());
	}

	/**
	 * Index the database on timestamp
	 */
	public void IndexOnTimestamp(String dbName){

		this.db = mongoClient.getDB(dbName);

		System.out.println("Java - SetTimestampIndex Db: " + this.db.getName());
		
		Set<String> all_collections = this.db.getCollectionNames();
		
		// index all the collections
		for (String coll_name : all_collections)
		{
			if(!coll_name.equals("system.indexes"))
			{
				System.out.println("\tIndexing on timestamp in coll: " + coll_name);
				db.getCollection(coll_name).createIndex(new BasicDBObject("timestamp", 1));
			}
		}
	}
	
	////////////////////////////////////////////////////////////////
	///// MARKER FUNCTIONS	
	/**
	 * Create the rviz markers
	 */
	public void CreateMarkers(ArrayList<Vector3d> pointsArr, String markerID){
		// create marker with default color and scale
		this.CreateMarkers(pointsArr, markerID, "point", "red", 0.01f);
	}
	
	/**
	 * Create the rviz markers
	 */
	public void CreateMarkers(ArrayList<Vector3d> pointsArr, String markerID, String markerType, String color, float scale){
// 		// List of marker points
// 		List<Point> marker_points = new ArrayList<Point>();
// 		
// 		// iterate the 3d vector to create the marker points
// 		for (Vector3d p_iter : pointsArr){
// 			marker_points.add(msgPoint(p_iter));
// 		}
// 
// 		// check if marker already exists
// 		MarkerObject m = MarkerPublisher.get().getMarker(markerID);
// 		if(m==null) {			
// 			// create marker
// 			m = MarkerPublisher.get().createMarker(markerID);
// 			// set the type of the marker
// 			m.setType(markerFromString(markerType));
// 			// set the positions of the list markers
// 			m.getMessage().setPoints(marker_points);
// 			// transform string color to float[] array
// 			m.setColor(this.colorFromString(color));
// 			// set the scale of the marker
// 			m.setScale(new float[] {scale, scale, scale});
// 		}
// 		// add ID to the marker container
// 		this.markerIDs.add(markerID);
	}
	
	/**
	 * Create the rviz mesh marker
	 */
	public void CreateMeshMarker(double translation[], double orientation[], String meshPath, String markerID){		
		// check if marker already exists
// 		MarkerObject m = MarkerPublisher.get().getMarker(markerID);
// 		if(m==null) {
// 			// create marker
// 			m = MarkerPublisher.get().createMarker(markerID);
// 			// set the type of the marker
// 			m.setType(Marker.MESH_RESOURCE);
// 			// set the path to the mesh
// 			m.setMeshResource(meshPath);
// 			// set pos and rotation
// 			m.setTranslation(translation);
// 			m.setOrientation(orientation);
// 			// set scale
// 			m.setScale(new float[] {1.0f,1.0f,1.0f});
// 		}
// 		// add ID to the marker container
// 		this.markerIDs.add(markerID);
	}

	/**
	 * Create the nested rviz mesh marker
	 */
	public void CreateNestedMeshMarkers(
			List<double[]> translations,
			List<double[]> orientations,
			List<String> names,
			String meshFolderPath,
			String markerID){
		// create marker for every link mesh
// 		for (int i = 0; i < names.size(); ++i)
// 		{
// 			final String curr_name = names.get(i);
// 			final String curr_id = markerID + curr_name;
// 			// check if marker already exists (ID + link names)
// 			MarkerObject m = MarkerPublisher.get().getMarker(curr_id);
// 			if(m==null) {
// 				// create marker
// 				m = MarkerPublisher.get().createMarker(curr_id);
// 				// set the type of the marker
// 				m.setType(Marker.MESH_RESOURCE);
// 				// set the path to the mesh
// 				m.setMeshResource(meshFolderPath + curr_name + ".dae");
// 				// set pos and rotation
// 				m.setTranslation(translations.get(i));
// 				m.setOrientation(orientations.get(i));
// 				// set scale
// 				m.setScale(new float[] {1.0f,1.0f,1.0f});
// 			}
// 			// add ID to the marker container
// 			this.markerIDs.add(curr_id);		
// 		}
	}
	
	/**
	 * Remove the rviz marker with the given ID
	 */
	public void RemoveMarker(String markerID){
// 		MarkerPublisher.get().eraseMarker(markerID);		
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
	///// POSE QUERY FUNCTIONS	
	//////////////////// MODEL
	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * save result to MongoDB
	 */
	public void WriteModelPoseAt(double timestamp, 
			String model_name, 
			String pose_traj_db_name, 
			String pose_traj_coll_name){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// writing results to a mongo collection
		try {

			// connect to client
			MongoClient mongoClient = new MongoClient( this.dbHost , 27017 );

			// get the db
			DB pose_traj_db = mongoClient.getDB(pose_traj_db_name);

			// check if the collection already exists
			if (pose_traj_db.collectionExists(pose_traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + pose_traj_coll_name + "\' already exists!" );				
			}
			// create the collection
			else
			{
				// create collection
				DBCollection pose_traj_coll = pose_traj_db.getCollection(pose_traj_coll_name);

				System.out.println("Writing most recent pose to \'" + pose_traj_coll_name + "\'" );

				// if query has a response, append metadata to it
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", pose_traj_coll_name)
					.append("type", "trajectory")
					.append("timestamp", timestamp)
					.append("description", "Model pose query..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					pose_traj_coll.insert(first_doc);
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}	
	}

	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * view results as rviz markers 
	 */
	public void ViewModelPoseAt(double timestamp, 
			String model_name, 
			String markerID){
		// view pose with default color and scale
		this.ViewModelPoseAt(timestamp, model_name, markerID, "point", "red", 0.01f);
	}
	
	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * view results as rviz markers 
	 */
	public void ViewModelPoseAt(double timestamp, 
			String model_name, 
			String markerID,
			String markerType,
			String color,
			float scale){
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		System.out.println("Java - ViewModelPoseAt");
		if(cursor.hasNext())
		{
			// Traj as dynamic array
			ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
			
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			pos.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			
			this.CreateMarkers(pos, markerID, markerType, color, scale);	
		}	
	}
	
	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * view results as rviz mesh marker 
	 */
	public void ViewModelMeshAt(String ts, 
			String model_name,
			String meshPath,
			String markerID){
		// transform the knowrob time to double with 3 decimal precision
		double timestamp = (double) Math.round((parseTime_d(ts) - TIME_OFFSET) * 1000) / 1000;	
		
		this.ViewModelMeshAt(timestamp, model_name, meshPath, markerID);
	}

	/**
	 * Query the Pose of the given model at the given timepoint (or the most recent one)
	 * view results as rviz mesh marker 
	 */
	public void ViewModelMeshAt(double timestamp, 
			String model_name,
			String meshPath,
			String markerID){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		if(cursor.hasNext())
		{
			// Traj as dynamic array
			ArrayList<Vector3d> pos = new ArrayList<Vector3d>();
			
			// get the first document as the next cursor
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			double[] translation = new double[] {
					((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
					((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
					((BasicDBObject) curr_doc.get("pos")).getDouble("z")};
			double[] orientation = this.quatFromEulerRad(
					((BasicDBObject) curr_doc.get("rot")).getDouble("x"),
					((BasicDBObject) curr_doc.get("rot")).getDouble("y"),
					((BasicDBObject) curr_doc.get("rot")).getDouble("z"));
			this.CreateMeshMarker(translation, orientation, meshPath, markerID);	
		}	
	}
	
	/**
	 * Get the poses of the model links meshes at the given timestamp
	 * view results as rviz markers
	 */
	public void ViewNestedMeshesAt(
			String ts_str,
			String model_name,
			String meshFolderPath,
			String markerID){		
		// transform the knowrob time to double with 3 decimal precision
		double timestamp = (double) Math.round((parseTime_d(ts_str) - TIME_OFFSET) * 1000) / 1000;	

		this.ViewNestedMeshesAt(timestamp, model_name, meshFolderPath, markerID);
	}
	
	/**
	 * Get the poses of the model links meshes at the given timestamp
	 * view results as rviz markers
	 */
	public void ViewNestedMeshesAt(double timestamp,
			String model_name,
			String meshFolderPath,
			String markerID){		
					
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);		
		proj_fields.put("links_pos", "$models.links.pos");
		proj_fields.put("links_rot", "$models.links.rot");
		proj_fields.put("links_name", "$models.links.name");
		
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
			
		// get the result of the query
		if(cursor.hasNext())
		{			
			// get the first document as the next cursor
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();			
			System.out.println(curr_doc);
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) curr_doc.get("links_pos");
			BasicDBList rot_list = (BasicDBList) curr_doc.get("links_rot");
			BasicDBList names_list = (BasicDBList) curr_doc.get("links_name");
			
			// Dynamic arrays of the links positions, rotations and names
			List<double[]> translations = new ArrayList<double[]>();
			List<double[]> orientations = new ArrayList<double[]>();
			List<String> names = new ArrayList<String>();
			
			// pos_list and rot_list and names_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{	
				// add the pos's
				translations.add(new double[] {
						((BasicDBObject) pos_list.get(i)).getDouble("x"),
						((BasicDBObject) pos_list.get(i)).getDouble("y"),
						((BasicDBObject) pos_list.get(i)).getDouble("z")});
				
				// add the rots
				orientations.add(this.quatFromEulerRad(
						((BasicDBObject) rot_list.get(i)).getDouble("x"),
						((BasicDBObject) rot_list.get(i)).getDouble("y"),
						((BasicDBObject) rot_list.get(i)).getDouble("z")));

				// add the names
				names.add((String)names_list.get(i));
			}			
			// create the nested mesh markers
			this.CreateNestedMeshMarkers(translations, orientations, names, meshFolderPath, markerID);			
		}	
	}
	
	//////////////////// LINK
	/**
	 * Query the Pose of the given link at the given timepoint (or the most recent one)
	 * save result in MongoDB
	 */
	public void WriteLinkPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String pose_traj_db_name, 
			String pose_traj_coll_name){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject("$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject("$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.pos");
		proj_fields.put("rot", "$models.links.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model,
				project_links, unwind_links, match_link, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// writing results to a mongo collection
		try {

			// connect to client
			MongoClient mongoClient = new MongoClient( this.dbHost , 27017 );

			// get the db
			DB pose_traj_db = mongoClient.getDB(pose_traj_db_name);

			// check if the collection already exists
			if (pose_traj_db.collectionExists(pose_traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + pose_traj_coll_name + "\' already exists!" );				
			}
			// create the collection
			else
			{
				// create collection
				DBCollection pose_traj_coll = pose_traj_db.getCollection(pose_traj_coll_name);

				System.out.println("Writing most recent pose to \'" + pose_traj_coll_name + "\'" );

				// if query has a response, append metadata to it
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", pose_traj_coll_name)
					.append("type", "trajectory")
					.append("timestamp", timestamp)
					.append("description", "Link pose query..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					pose_traj_coll.insert(first_doc);
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}	
	}
	
	/**
	 * Query the Pose of the given link at the given timepoint (or the most recent one)
	 * save result in MongoDB
	 */
	public void ViewLinkPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String markerID){
		// view pose with default color and scale
		this.ViewLinkPoseAt(timestamp, model_name, link_name, markerID, "point", "red", 0.01f);	
	}
	
	/**
	 * Query the Pose of the given link at the given timepoint (or the most recent one)
	 * save result in MongoDB
	 */
	public void ViewLinkPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String markerID,
			String markerType,
			String color,
			float scale){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject("$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject("$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.pos");
		proj_fields.put("rot", "$models.links.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model,
				project_links, unwind_links, match_link, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		if(cursor.hasNext())
		{
			// Traj as dynamic array
			ArrayList<Vector3d> pose = new ArrayList<Vector3d>();
			
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			pose.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			
			this.CreateMarkers(pose, markerID, markerType, color, scale);	
		}
	}

	//////////////////// COLLISION
	/**
	 * Query the Pose of the given collision at the given timepoint (or the most recent one)
	 * save result in MongoDB
	 */
	public void WriteCollisionPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String collision_name,
			String pose_traj_db_name, 
			String pose_traj_coll_name){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject("$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject("$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_collision_fields = new BasicDBObject("timestamp", 1);
		proj_collision_fields.put("models.links.collisions", 1);
		DBObject project_collisions = new BasicDBObject("$project", proj_collision_fields);

		// $unwind the collisions
		DBObject unwind_collisions = new BasicDBObject("$unwind", "$models.links.collisions");

		// $match for the given collision name from the unwinded collisions
		DBObject match_collision = new BasicDBObject(
				"$match", new BasicDBObject("models.links.collisions.name", collision_name));	

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.collisions.pos");
		proj_fields.put("rot", "$models.links.collisions.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model,
				project_links, unwind_links, match_link, 
				project_collisions, unwind_collisions, match_collision,
				project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// writing results to a mongo collection
		try {

			// connect to client
			MongoClient mongoClient = new MongoClient( this.dbHost , 27017 );

			// get the db
			DB pose_traj_db = mongoClient.getDB(pose_traj_db_name);

			// check if the collection already exists
			if (pose_traj_db.collectionExists(pose_traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + pose_traj_coll_name + "\' already exists!" );				
			}
			// create the collection
			else
			{
				// create collection
				DBCollection pose_traj_coll = pose_traj_db.getCollection(pose_traj_coll_name);

				System.out.println("Writing most recent pose to \'" + pose_traj_coll_name + "\'" );

				// if query has a response, append metadata to it
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", pose_traj_coll_name)
					.append("type", "trajectory")
					.append("timestamp", timestamp)
					.append("description", "Collision pose query..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					pose_traj_coll.insert(first_doc);
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}	
	}

	/**
	 * Query the Pose of the given collision at the given timepoint (or the most recent one)
	 * view results as rviz markers
	 */
	public void ViewCollisionPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String collision_name,
			String markerID){
		// view pose with default color and scale
		this.ViewCollisionPoseAt(timestamp, model_name, link_name, collision_name, markerID, "point", "red", 0.01f);	
	}
	
	/**
	 * Query the Pose of the given collision at the given timepoint (or the most recent one)
	 * view results as rviz markers
	 */
	public void ViewCollisionPoseAt(double timestamp, 
			String model_name,
			String link_name,
			String collision_name,
			String markerID,
			String markerType,
			String color,
			float scale){

		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject("$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject("$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_collision_fields = new BasicDBObject("timestamp", 1);
		proj_collision_fields.put("models.links.collisions", 1);
		DBObject project_collisions = new BasicDBObject("$project", proj_collision_fields);

		// $unwind the collisions
		DBObject unwind_collisions = new BasicDBObject("$unwind", "$models.links.collisions");

		// $match for the given collision name from the unwinded collisions
		DBObject match_collision = new BasicDBObject(
				"$match", new BasicDBObject("models.links.collisions.name", collision_name));	

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.collisions.pos");
		proj_fields.put("rot", "$models.links.collisions.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model,
				project_links, unwind_links, match_link, 
				project_collisions, unwind_collisions, match_collision,
				project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		if(cursor.hasNext())
		{
			// Traj as dynamic array
			ArrayList<Vector3d> pose = new ArrayList<Vector3d>();
			
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			pose.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			
			this.CreateMarkers(pose, markerID, markerType, color, scale);	
		}
	}

	////////////////////////////////////////////////////////////////
	///// TRAJ QUERY FUNCTIONS
	////////////////////MODEL
	/**
	 * Query the trajectory of the given model with string timestamps, Knowrob specific
	 * save result in MongoDB
	 */
	public void WriteModelTrajectory(String start,
			String end,
			String model_name,
			String traj_db_name){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// set the collection name
		String traj_coll_name = this.coll.getName();// + "_" + model_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteModelTrajectory(start_ts, end_ts, model_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given model with double timestamps using default coll name
	 */
	public void WriteModelTrajectory(double start_ts,
			double end_ts,
			String model_name,
			String traj_db_name){

		// set the collection name
		String traj_coll_name = this.coll.getName();// + "_" + model_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteModelTrajectory(start_ts, end_ts, model_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given model with double timestamps
	 * save result in MongoDB
	 */
	public void WriteModelTrajectory(double start_ts,
			double end_ts,
			String model_name,
			String traj_db_name,
			String traj_coll_name){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// writing results to a mongo collection
		try {

			// connect to client
			MongoClient mongoClient = new MongoClient( this.dbHost , 27017 );

			// get the db
			DB traj_db = mongoClient.getDB(traj_db_name);


			// check if the collection already exists
			if (traj_db.collectionExists(traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + traj_coll_name + "\' already exists!" );				
			}
			// create the collection
			else
			{
				// create collection
				DBCollection traj_coll = traj_db.getCollection(traj_coll_name);

				System.out.println("Java - Writing to \'" + traj_db_name+ "." + traj_coll_name + "\'" );

				// if cursor not empty, append matadata to the first doc
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", traj_coll_name)
					.append("type", "trajectory")
					.append("start", start_ts)
					.append("end", end_ts)
					.append("description", "Model trajectory..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					traj_coll.insert(first_doc);
				}
				// if query returned no values for these timestamps, get the pose at the nearest timestamp
				else
				{
					// write the pose to the given db and coll
					this.WriteModelPoseAt(start_ts, model_name, traj_db_name, traj_coll_name);
				}

				// insert rest of trajectory
				while (cursor.hasNext()) {
					traj_coll.insert(cursor.next());
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Query the trajectory of the given model with string timestamps, Knowrob specific
	 * view traj using rviz markers
	 */
	public void ViewModelTrajectory(String start,
			String end,
			String model_name,
			String markerID){
		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;
		
		// call the traj with double timestamps
		this.ViewModelTrajectory(start_ts, end_ts, model_name, markerID);
	}

	/**
	 * Query the trajectory of the given model with double timestamps
	 * view as rviz markers
	 */
	public ArrayList<Vector3d> ViewModelTrajectory(double start_ts,
			double end_ts,
			String model_name,
			String markerID){
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();	
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));

			if(markerID != null)
			{
				traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("rot")).getDouble("x"),
					((BasicDBObject) first_doc.get("rot")).getDouble("y"),
					((BasicDBObject) first_doc.get("rot")).getDouble("z")));
			}

		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewModelPoseAt(start_ts, model_name, markerID);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			// add position to trajectory
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));

			if(markerID != null)
			{
				traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("rot")).getDouble("x"),
					((BasicDBObject) first_doc.get("rot")).getDouble("y"),
					((BasicDBObject) first_doc.get("rot")).getDouble("z")));
			}
		}
		
		// create the markers
		if(markerID != null )this.CreateMarkers(traj, markerID);
		return traj;
	}
	
	/**
	 * Query the trajectory of the given model with string timestamps, Knowrob specific
	 * view traj using rviz markers
	 */
	public void ViewModelTrajectory(String start,
			String end,
			String model_name,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewModelTrajectory(start_ts, end_ts, model_name, markerID, markerType, color, scale, deltaT);
	}
	
	/**
	 * Query the trajectory of the given model with double timestamps
	 * view as rviz markers
	 */
	public void ViewModelTrajectory(double start_ts,
			double end_ts,
			String model_name,
			String markerID,
			String markerType,			
			String color,
			float scale,
			double deltaT){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			// set the timestamp
			prev_ts = first_doc.getDouble("timestamp");
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewModelPoseAt(start_ts, model_name, markerID, markerType, color, scale);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				traj.add(new Vector3d(
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z")));
				prev_ts = curr_ts;
			}
		}
		
		// create the markers
		this.CreateMarkers(traj, markerID, markerType, color, scale);
	}
	
	////////////////////MESH
	/**
	 * Query the trajectory of the given model with string timestamps, Knowrob specific
	 * view traj using rviz markers
	 */
	public void ViewModelMeshTrajectory(String start,
			String end,
			String model_name,
			String meshPath,
			String markerID,
			double deltaT){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewModelMeshTrajectory(start_ts, end_ts, model_name, meshPath, markerID, deltaT);
	}
	
	/**
	 * Query the trajectory of the given model with double timestamps
	 * view as rviz markers
	 */
	public void ViewModelMeshTrajectory(double start_ts,
			double end_ts,
			String model_name,
			String meshPath,
			String markerID,
			double deltaT){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("pos", "$models.pos");
		proj_fields.put("rot", "$models.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// marker ID suffix
		int marker_suff = 0;
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			double[] translation = new double[] {
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")};
			double[] orientation = this.quatFromEulerRad(
					((BasicDBObject) first_doc.get("rot")).getDouble("x"),
					((BasicDBObject) first_doc.get("rot")).getDouble("y"),
					((BasicDBObject) first_doc.get("rot")).getDouble("z"));
			// create the markers
			this.CreateMeshMarker(translation, orientation, meshPath, markerID);
			
			// set the timestamp
			prev_ts = first_doc.getDouble("timestamp");
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewModelMeshAt(start_ts, model_name, markerID, meshPath);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				double[] translation = new double[] {
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z")};
				double[] orientation = this.quatFromEulerRad(
						((BasicDBObject) curr_doc.get("rot")).getDouble("x"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("y"),
						((BasicDBObject) curr_doc.get("rot")).getDouble("z"));
				// create the markers
				this.CreateMeshMarker(translation, orientation, meshPath, markerID + marker_suff);	
				
				// update current ts
				prev_ts = curr_ts;
				
				// append to marker suffix
				marker_suff++;
			}
		}
	}
	
	////////////////////LINK
	/**
	 * Query the trajectory of the given link of the given model with string timestamps, Knowrob specific
	 * save result in MongoDB
	 */
	public void WriteLinkTrajectory(String start, 
			String end, 
			String model_name, 
			String link_name,
			String traj_db_name){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;


		String traj_coll_name = this.coll.getName();// + "_" + link_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteLinkTrajectory(start_ts, end_ts, model_name, link_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given link of the given model with string timestamps, with default collection name
	 * save result in MongoDB
	 */
	public void WriteLinkTrajectory(double start_ts, 
			double end_ts, 
			String model_name, 
			String link_name,
			String traj_db_name){

		String traj_coll_name = this.coll.getName();// + "_" + link_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteLinkTrajectory(start_ts, end_ts, model_name, link_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given link of the given model with double timestamps
	 * save result in MongoDB
	 */
	public void WriteLinkTrajectory(double start_ts, 
			double end_ts, 
			String model_name, 
			String link_name,
			String traj_db_name,
			String traj_coll_name){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.pos");
		proj_fields.put("rot", "$models.links.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project_links, unwind_links, match_link, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		try {

			MongoClient mongoClient = new MongoClient( this.dbHost , 27017 );

			DB traj_db = mongoClient.getDB(traj_db_name);

			// check if the collection already exists
			if (traj_db.collectionExists(traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + traj_db_name + "." + traj_coll_name + "\' already exists!" );
			}
			// create the collection
			else
			{
				// create collection
				DBCollection traj_coll = traj_db.getCollection(traj_coll_name);

				System.out.println("Java - Writing to \'" + traj_db_name + "." + traj_coll_name + "\'" );

				// if cursor not empty, append matadata to the first doc
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", traj_coll_name)
					.append("type", "trajectory")
					.append("start", start_ts)
					.append("end", end_ts)
					.append("description", "Link trajectory..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					traj_coll.insert(first_doc);
				}
				// if query returned no values for these timestamps, get the pose at the nearest timestamp
				else
				{
					// write the pose to the given db and coll
					this.WriteLinkPoseAt(start_ts, model_name, link_name, traj_db_name, traj_coll_name);
				}

				// insert rest of trajectory
				while (cursor.hasNext()) {
					traj_coll.insert(cursor.next());
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}  
	}

	/**
	 * Query the trajectory of the given link of the given model with string timestamps, Knowrob specific
	 * view result as rviz markers
	 */
	public void ViewLinkTrajectory(String start, 
			String end, 
			String model_name,
			String link_name,
			String markerID){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewLinkTrajectory(start_ts, end_ts, model_name, link_name, markerID);
	}

	/**
	 * Query the trajectory of the given link of the given model with double timestamps
	 * save result in MongoDB
	 */
	public void ViewLinkTrajectory(double start_ts, 
			double end_ts, 
			String model_name, 
			String link_name,
			String markerID){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.pos");
		proj_fields.put("rot", "$models.links.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project_links, unwind_links, match_link, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewLinkPoseAt(start_ts, model_name, link_name, markerID);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			// add position to trajectory
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
		}
		
		// create the markers
		this.CreateMarkers(traj, markerID);
	}

	/**
	 * Query the trajectory of the given link of the given model with string timestamps, Knowrob specific
	 * view result as rviz markers
	 */
	public void ViewLinkTrajectory(String start, 
			String end, 
			String model_name,
			String link_name,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewLinkTrajectory(start_ts, end_ts, model_name, link_name, markerID, markerType, color, scale, deltaT);
	}
	
	/**
	 * Query the trajectory of the given link of the given model with double timestamps
	 * save result in MongoDB
	 */
	public void ViewLinkTrajectory(double start_ts, 
			double end_ts, 
			String model_name, 
			String link_name,
			String markerID,
			String markerType,			
			String color,
			float scale,
			double deltaT){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.pos");
		proj_fields.put("rot", "$models.links.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project_links, unwind_links, match_link, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			// set the timestamp
			prev_ts = first_doc.getDouble("timestamp");
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewLinkPoseAt(start_ts, model_name, link_name, markerID, markerType, color, scale);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				traj.add(new Vector3d(
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z")));
				prev_ts = curr_ts;
			}
		}
		
		// create the markers
		this.CreateMarkers(traj, markerID, markerType, color, scale);
	}

	////////////////////COLLISION
	/**
	 * Query the trajectory of the given collision of the given link of the given model with string timestamps,
	 * Knowrob specific
	 * save result in MongoDB
	 */
	public void WriteCollisionTrajectory(
			String start,
			String end,
			String model_name,
			String link_name,
			String collision_name,
			String traj_db_name){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// set default coll name
		String traj_coll_name = this.coll.getName();// + "_" + collision_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteCollisionTrajectory(start_ts, end_ts, model_name, link_name, collision_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given collision of the given link of the given model with string timestamps,
	 * with default coll
	 * save result in MongoDB
	 */
	public void WriteCollisionTrajectory(
			double start_ts,
			double end_ts,
			String model_name,
			String link_name,
			String collision_name,
			String traj_db_name){

		// set default coll name
		String traj_coll_name = this.coll.getName();// + "_" + collision_name + "_" + start_ts + "-" + end_ts;

		// call the traj with double timestamps
		this.WriteCollisionTrajectory(start_ts, end_ts, model_name, link_name, collision_name, traj_db_name, traj_coll_name);
	}

	/**
	 * Query the trajectory of the given collision of the given link of the given model from double timestamps
	 * save result in MongoDB
	 */
	public void WriteCollisionTrajectory(
			double start_ts,
			double end_ts,
			String model_name,
			String link_name,
			String collision_name,
			String traj_db_name,
			String traj_coll_name){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_collision_fields = new BasicDBObject("timestamp", 1);
		proj_collision_fields.put("models.links.collisions", 1);
		DBObject project_collisions = new BasicDBObject("$project", proj_collision_fields);

		// $unwind the collisions
		DBObject unwind_collisions = new BasicDBObject("$unwind", "$models.links.collisions");

		// $match for the given collision name from the unwinded collisions
		DBObject match_collision = new BasicDBObject(
				"$match", new BasicDBObject("models.links.collisions.name", collision_name));	

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.collisions.pos");
		proj_fields.put("rot", "$models.links.collisions.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, 
				project_links, unwind_links, match_link, 
				project_collisions, unwind_collisions, match_collision,
				project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		try {

			MongoClient mongoClient = new MongoClient(this.dbHost , 27017);

			DB traj_db = mongoClient.getDB(traj_db_name);

			// check if the collection already exists
			if (traj_db.collectionExists(traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + traj_db_name + "." + traj_coll_name + "\' already exists!" );
			}
			// create the collection
			else
			{
				// create collection
				DBCollection traj_coll = traj_db.getCollection(traj_coll_name);

				System.out.println("Java  - Writing to \'" + traj_db_name + "." + traj_coll_name + "\'" );

				// if cursor not empty, append matadata to the first doc
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", traj_coll_name)
					.append("type", "trajectory")
					.append("start", start_ts)
					.append("end", end_ts)
					.append("description", "Collision trajectory..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					traj_coll.insert(first_doc);
				}
				// if query returned no values for these timestamps, get the pose at the nearest timestamp
				else
				{
					// write the pose to the given db and coll
					this.WriteCollisionPoseAt(
							start_ts, model_name, link_name, collision_name, traj_db_name, traj_coll_name);
				}

				// insert rest of trajectory
				while (cursor.hasNext()) {
					traj_coll.insert(cursor.next());
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}  

	}
	
	/**
	 * Query the trajectory of the given collision of the given link of the given model with string timestamps,
	 * Knowrob specific
	 * view results via rviz markers
	 */
	public void ViewCollisionTrajectory(
			String start,
			String end,
			String model_name,
			String link_name,
			String collision_name,
			String markerID){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewCollisionTrajectory(start_ts, end_ts, model_name, link_name, collision_name, markerID);
	}

	/**
	 * Query the trajectory of the given collision of the given link of the given model from double timestamps
	 * view results as rviz markers
	 */
	public void ViewCollisionTrajectory(
			double start_ts,
			double end_ts,
			String model_name,
			String link_name,
			String collision_name,
			String markerID){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_collision_fields = new BasicDBObject("timestamp", 1);
		proj_collision_fields.put("models.links.collisions", 1);
		DBObject project_collisions = new BasicDBObject("$project", proj_collision_fields);

		// $unwind the collisions
		DBObject unwind_collisions = new BasicDBObject("$unwind", "$models.links.collisions");

		// $match for the given collision name from the unwinded collisions
		DBObject match_collision = new BasicDBObject(
				"$match", new BasicDBObject("models.links.collisions.name", collision_name));	

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.collisions.pos");
		proj_fields.put("rot", "$models.links.collisions.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, 
				project_links, unwind_links, match_link, 
				project_collisions, unwind_collisions, match_collision,
				project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();		
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewCollisionPoseAt(start_ts, model_name, link_name, collision_name, markerID);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			// add position to trajectory
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
		}
		
		// create the markers
		this.CreateMarkers(traj, markerID);
	}
	
	/**
	 * Query the trajectory of the given collision of the given link of the given model with string timestamps,
	 * Knowrob specific
	 * view results via rviz markers
	 */
	public void ViewCollisionTrajectory(
			String start,
			String end,
			String model_name,
			String link_name,
			String collision_name,
			String markerID,
			String markerType,
			String color,
			float scale,
			double deltaT){

		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start) - TIME_OFFSET) * 1000) / 1000;		
		double end_ts = (double) Math.round((parseTime_d(end) - TIME_OFFSET) * 1000) / 1000;

		// call the traj with double timestamps
		this.ViewCollisionTrajectory(
				start_ts, end_ts, model_name, link_name, collision_name, markerID, markerType, color, scale, deltaT);
	}
	
	/**
	 * Query the trajectory of the given collision of the given link of the given model from double timestamps
	 * view results as rviz markers
	 */
	public void ViewCollisionTrajectory(
			double start_ts,
			double end_ts,
			String model_name,
			String link_name,
			String collision_name,
			String markerID,
			String markerType,			
			String color,
			float scale,
			double deltaT){

		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
				new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind the models
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));		

		// build the $projection operation
		DBObject proj_links_fields = new BasicDBObject("_id", 0);
		proj_links_fields.put("timestamp", 1);
		proj_links_fields.put("models.links", 1);
		DBObject project_links = new BasicDBObject("$project", proj_links_fields);

		// $unwind the links
		DBObject unwind_links = new BasicDBObject("$unwind", "$models.links");

		// $match for the given link name from the unwinded links
		DBObject match_link = new BasicDBObject(
				"$match", new BasicDBObject("models.links.name", link_name));

		// build the final $projection operation
		DBObject proj_collision_fields = new BasicDBObject("timestamp", 1);
		proj_collision_fields.put("models.links.collisions", 1);
		DBObject project_collisions = new BasicDBObject("$project", proj_collision_fields);

		// $unwind the collisions
		DBObject unwind_collisions = new BasicDBObject("$unwind", "$models.links.collisions");

		// $match for the given collision name from the unwinded collisions
		DBObject match_collision = new BasicDBObject(
				"$match", new BasicDBObject("models.links.collisions.name", collision_name));	

		// build the final $projection operation
		DBObject proj_fields = new BasicDBObject("timestamp", 1);
		proj_fields.put("pos", "$models.links.collisions.pos");
		proj_fields.put("rot", "$models.links.collisions.rot");
		DBObject project = new BasicDBObject("$project", proj_fields);


		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, 
				project_links, unwind_links, match_link, 
				project_collisions, unwind_collisions, match_collision,
				project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// Traj as dynamic array
		ArrayList<Vector3d> traj = new ArrayList<Vector3d>();
		
		// timestamp used for deltaT
		double prev_ts = 0;
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			traj.add(new Vector3d(
					((BasicDBObject) first_doc.get("pos")).getDouble("x"),
					((BasicDBObject) first_doc.get("pos")).getDouble("y"),
					((BasicDBObject) first_doc.get("pos")).getDouble("z")));
			// set the timestamp
			prev_ts = first_doc.getDouble("timestamp");
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			// write the pose to the given db and coll
			this.ViewCollisionPoseAt(start_ts, model_name, link_name, collision_name, markerID, markerType, color, scale);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{
				traj.add(new Vector3d(
						((BasicDBObject) curr_doc.get("pos")).getDouble("x"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("y"),
						((BasicDBObject) curr_doc.get("pos")).getDouble("z")));
				prev_ts = curr_ts;
			}
		}
		
		// create the markers
		this.CreateMarkers(traj, markerID, markerType, color, scale);
	}
	
	////////////////////LINKS
	/**
	 * Get the positions of the model links at the given timestamp
	 * save result in MongoDB
	 */
	public void WriteLinksPositionsAt(
			String ts_str,
			String model_name,
			String traj_db_name){		
					
		// transform the knowrob time to double with 3 decimal precision
		double timestamp = (double) Math.round((parseTime_d(ts_str) - TIME_OFFSET) * 1000) / 1000;
		
		// set default coll name
		String traj_coll_name = this.coll.getName();// + "_" + model_name + "_links_at_"+ timestamp;	
							
		// remove the knowrob namespace (http://knowrob.org/kb/knowrob.owl#) form the model 
		// String model_name = kr_model_name.split("#")[1];
		
		//System.out.println("Java - timestamp: " + timestamp + " model name: " + model_name);
		
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		try {

			MongoClient mongoClient = new MongoClient(this.dbHost , 27017);

			DB traj_db = mongoClient.getDB(traj_db_name);

			// check if the collection already exists
			if (traj_db.collectionExists(traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + traj_db_name + "." + traj_coll_name + "\' already exists!" );
			}
			// create the collection
			else
			{
				// create collection
				DBCollection traj_coll = traj_db.getCollection(traj_coll_name);

				System.out.println("Java  - Writing to \'" + traj_db_name + "." + traj_coll_name + "\'" );

				// if cursor not empty, append matadata to the first doc
				if(cursor.hasNext())
				{
					// get pancake roundess again in order to append it to the metadata
					double roundess = this.GetPancakeRoundness(ts_str, model_name);
					
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", traj_coll_name)
					.append("type", "links_pos")
					.append("timestamp", timestamp)
					.append("roundness", roundess)
					.append("description", "Pancake links positions..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					traj_coll.insert(first_doc);
				}
				// if query returned no values for these timestamps, get the pose at the nearest timestamp
				else
				{
					System.out.println("Java  - WriteLinksPositionsAt Query returned no results!'" );
				}

				// insert rest of trajectory
				while (cursor.hasNext()) {
					traj_coll.insert(cursor.next());
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}		
	}

	/**
	 * Get the positions of the model links at the given timestamp
	 * view results as rviz markers
	 */
	public void ViewLinksPositionsAt(
			String ts_str,
			String model_name,
			String markerID){
		// view poses with default color and scale
		this.ViewLinksPositionsAt(ts_str, model_name, markerID, "point", "red", 0.01f);
	}
	
	/**
	 * Get the positions of the model links at the given timestamp
	 * view results as rviz markers
	 */
	public void ViewLinksPositionsAt(
			String ts_str,
			String model_name,
			String markerID,
			String markerType,
			String color,
			float scale){		
					
		// transform the knowrob time to double with 3 decimal precision
		double timestamp = (double) Math.round((parseTime_d(ts_str) - TIME_OFFSET) * 1000) / 1000;
		
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<Vector3d> positions = new ArrayList<Vector3d>();	
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) first_doc.get("links_pos");
			
			// pos_list and rot_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{				
				positions.add(new Vector3d(
						((BasicDBObject) pos_list.get(i)).getDouble("x"),
						((BasicDBObject) pos_list.get(i)).getDouble("y"),
						((BasicDBObject) pos_list.get(i)).getDouble("z")));
			}
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			System.out.println("Java  - ViewLinksPositionsAt Query returned no results!'" );
		}
		
		// create the markers
		this.CreateMarkers(positions, markerID, markerType, color, scale);
	}

	/**
	 * Get the positions of the model links at the given timestamp
	 * save result in MongoDB
	 */
	public void WriteLinksTrajs(
			String start_str,
			String end_str,
			String model_name,
			String traj_db_name){		
		
		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start_str) - TIME_OFFSET) * 1000) / 1000;
		double end_ts = (double) Math.round((parseTime_d(end_str) - TIME_OFFSET) * 1000) / 1000;
		
		// set default coll name
		String traj_coll_name = this.coll.getName();/* + "_" 
				+ model_name + "_links_trajs_" + start_ts + "_" + end_ts;*/	
								
		// remove the knowrob namespace (http://knowrob.org/kb/knowrob.owl#) form the model 
		// String model_name = kr_model_name.split("#")[1];
		
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
		new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		try {

			MongoClient mongoClient = new MongoClient(this.dbHost , 27017);

			DB traj_db = mongoClient.getDB(traj_db_name);

			// check if the collection already exists
			if (traj_db.collectionExists(traj_coll_name))
			{
				System.out.println("!!! Collection: \'" + traj_db_name + "." + traj_coll_name + "\' already exists!" );
			}
			// create the collection
			else
			{
				// create collection
				DBCollection traj_coll = traj_db.getCollection(traj_coll_name);

				System.out.println("Java  - Writing to \'" + traj_db_name + "." + traj_coll_name + "\'" );

				// if cursor not empty, append matadata to the first doc
				if(cursor.hasNext())
				{
					// create metadata doc
					BasicDBObject meta_data = new BasicDBObject("name", traj_coll_name)
					.append("type", "links_trajs")
					.append("start", start_ts)
					.append("end", end_ts)
					.append("description", "Pancake links trajectories..");

					// get the first document as the next cursor and append the metadata to it
					BasicDBObject first_doc = (BasicDBObject) cursor.next();

					first_doc.append("metadata", meta_data);

					// insert document with metadata
					traj_coll.insert(first_doc);
				}
				// if query returned no values for these timestamps, get the pose at the nearest timestamp
				else
				{
					System.out.println("Java  - WriteLinksPositionsAt Query returned no results!'" );
				}

				// insert rest of trajectory
				while (cursor.hasNext()) {
					traj_coll.insert(cursor.next());
				}
			}
		}catch (UnknownHostException e) {
			e.printStackTrace();
		}		
	}
	
	/**
	 * Get the positions of the model links at the given timestamp
	 * view as rviz markers
	 */
	public void ViewLinksTrajs(
			String start_str,
			String end_str,
			String model_name,
			String markerID){		
		
		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start_str) - TIME_OFFSET) * 1000) / 1000;
		double end_ts = (double) Math.round((parseTime_d(end_str) - TIME_OFFSET) * 1000) / 1000;
		
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
		new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<Vector3d> trajs = new ArrayList<Vector3d>();	
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) first_doc.get("links_pos");
			
			// pos_list and rot_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{				
				trajs.add(new Vector3d(
						((BasicDBObject) pos_list.get(i)).getDouble("x"),
						((BasicDBObject) pos_list.get(i)).getDouble("y"),
						((BasicDBObject) pos_list.get(i)).getDouble("z")));
			}
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			this.ViewLinksPositionsAt(start_str, model_name, markerID);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) curr_doc.get("links_pos");
			
			// pos_list and rot_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{				
				trajs.add(new Vector3d(
						((BasicDBObject) pos_list.get(i)).getDouble("x"),
						((BasicDBObject) pos_list.get(i)).getDouble("y"),
						((BasicDBObject) pos_list.get(i)).getDouble("z")));
			}
		}
		
		// create the markers
		this.CreateMarkers(trajs, markerID);
	}
	
	/**
	 * Get the positions of the model links at the given timestamp
	 * view as rviz markers
	 */
	public void ViewLinksTrajs(
			String start_str,
			String end_str,
			String model_name,
			String markerID,
			String markerType,			
			String color,
			float scale,
			double deltaT){		
		
		// transform the knowrob time to double with 3 decimal precision
		double start_ts = (double) Math.round((parseTime_d(start_str) - TIME_OFFSET) * 1000) / 1000;
		double end_ts = (double) Math.round((parseTime_d(end_str) - TIME_OFFSET) * 1000) / 1000;
		
		// create the pipeline operations, first with the $match check the times
		DBObject match_time = new BasicDBObject("$match", new BasicDBObject("timestamp", 
		new BasicDBObject("$gte", start_ts).append("$lte", end_ts)));

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);
		
		// Traj as dynamic array
		ArrayList<Vector3d> trajs = new ArrayList<Vector3d>();
		
		double prev_ts = 0;
		
		// if cursor not empty, append matadata to the first doc
		if(cursor.hasNext())
		{
			// get the first document as the next cursor
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) first_doc.get("links_pos");
			
			// get the timestamp
			prev_ts = first_doc.getDouble("timestamp");
			
			// pos_list and rot_list length should be always the same
			for (int i = 0; i < pos_list.size(); ++i)
			{				
				trajs.add(new Vector3d(
						((BasicDBObject) pos_list.get(i)).getDouble("x"),
						((BasicDBObject) pos_list.get(i)).getDouble("y"),
						((BasicDBObject) pos_list.get(i)).getDouble("z")));
			}
		}
		// if query returned no values for these timestamps, get the pose at the nearest timestamp
		else
		{
			this.ViewLinksPositionsAt(start_str, model_name, markerID, markerType, color, scale);
		}
		// insert rest of trajectory
		while (cursor.hasNext()) {			
			// get the current document
			BasicDBObject curr_doc = (BasicDBObject) cursor.next();
			
			// get the list of links pos
			BasicDBList pos_list = (BasicDBList) curr_doc.get("links_pos");
			
			double curr_ts = curr_doc.getDouble("timestamp");
			
			// if time diff > then deltaT add position to trajectory
			if(curr_ts - prev_ts > deltaT)
			{		
				// pos_list and rot_list length should be always the same
				for (int i = 0; i < pos_list.size(); ++i)
				{				
					trajs.add(new Vector3d(
							((BasicDBObject) pos_list.get(i)).getDouble("x"),
							((BasicDBObject) pos_list.get(i)).getDouble("y"),
							((BasicDBObject) pos_list.get(i)).getDouble("z")));
				}
				prev_ts = curr_ts;
			}
		}	
		
		// create the markers
		this.CreateMarkers(trajs, markerID, markerType, color, scale);
	}
	
	
	////////////////////////////////////////////////////////////////
	///// PANCAKE COMPUTABLE FUNCTIONS	
	/**
	 * Get the positions of the model links at the given timestamp
	 */
	public List<Point3d> GetLinksPositions(
			String ts_str,
			String model_name){
		
		// list of all the links positions
		List<Point3d> links_positions = new ArrayList<Point3d>();		
				
		// transform the knowrob time to double with 3 decimal precision
		double timestamp = (double) Math.round((parseTime_d(ts_str) - TIME_OFFSET) * 1000) / 1000;
					
		// remove the knowrob namespace (http://knowrob.org/kb/knowrob.owl#) form the model 
		// String model_name = kr_model_name.split("#")[1];
		
		//System.out.println("Java - timestamp: " + timestamp + " model name: " + model_name);
		
		// $and list for querying the $match in the aggregation
		BasicDBList time_and_name = new BasicDBList();

		// add the timestamp and the model name
		time_and_name.add(new BasicDBObject("timestamp", new BasicDBObject("$lte", timestamp)));
		time_and_name.add(new BasicDBObject("models.name", model_name));

		// create the pipeline operations, first the $match
		DBObject match_time_and_name = new BasicDBObject(
				"$match", new BasicDBObject( "$and", time_and_name)); 

		// sort the results in descending order on the timestamp (keep most recent result first)
		DBObject sort_desc = new BasicDBObject(
				"$sort", new BasicDBObject("timestamp", -1));

		// $limit the result to 1, we only need one pose
		DBObject limit_result = new BasicDBObject("$limit", 1);

		// $unwind models in order to output only the queried model
		DBObject unwind_models = new BasicDBObject("$unwind", "$models");

		// $match for the given model name from the unwinded models
		DBObject match_model = new BasicDBObject(
				"$match", new BasicDBObject("models.name", model_name));

		// build the $projection operation
		DBObject proj_fields = new BasicDBObject("_id", 0);
		proj_fields.put("timestamp", 1);
		proj_fields.put("links_pos", "$models.links.pos");
		DBObject project = new BasicDBObject("$project", proj_fields);

		// run aggregation
		List<DBObject> pipeline = Arrays.asList(
				match_time_and_name, sort_desc, limit_result, unwind_models, match_model, project);

		AggregationOptions aggregationOptions = AggregationOptions.builder()
				.batchSize(100)
				.outputMode(AggregationOptions.OutputMode.CURSOR)
				.allowDiskUse(true)
				.build();

		Cursor cursor = this.coll.aggregate(pipeline, aggregationOptions);

		// if query has a response, append metadata to it
		if(cursor.hasNext())
		{
			// get the first doc
			BasicDBObject first_doc = (BasicDBObject) cursor.next();
			
			// get the positions array
			BasicDBList links_pos_arr = (BasicDBList) first_doc.get("links_pos");
			
			// iterate the results
			for (int i = 0; i < links_pos_arr.size(); i++) {
				
				// current position
				Point3d pos = new Point3d();
				
				// set the positions
				pos.x = ((BasicDBObject) links_pos_arr.get(i)).getDouble("x");
				pos.y = ((BasicDBObject) links_pos_arr.get(i)).getDouble("y");
				pos.z = ((BasicDBObject) links_pos_arr.get(i)).getDouble("z");
				
				// add position to the list
				links_positions.add(pos);
			}
		}		
		return links_positions;
	}

	/**
	 * Return the PCA of the 3d points
	 */
	public PrincipalComponents GetPCA(List<Point3d> points, boolean center_data){

		// pca
		PrincipalComponents pca = new PrincipalComponents();
		
		// Create the x y z attributes
		FastVector atts = new FastVector();

		Attribute x = new Attribute("x");
		Attribute y = new Attribute("y");
		Attribute z = new Attribute("z");
		
		atts.addElement(x);
		atts.addElement(y);
		atts.addElement(z);
		
		// Create instances
		Instances points_dataset = new Instances("PointsPCA", atts, points.size());
		
		// iterate through all the points
		for (int i = 0; i < points.size(); i++) {
			
			// new intance of 3 values
			Instance inst = new SparseInstance(3); 
			
			// get pos point
			Point3d pos = points.get(i);
			
			// Set instance's values for the attributes x, y, z
			inst.setValue(x, pos.x); 
			inst.setValue(y, pos.y); 
			inst.setValue(z, pos.z);  
			
			// add instance to dataset
			points_dataset.add(inst);
		}
		
		// center data
		pca.setCenterData(center_data);
		
		try{					
			// build evaluator
			pca.buildEvaluator(points_dataset);
			
		}catch (java.lang.Exception e)
		{
			e.printStackTrace();
		}		
//		System.out.println(points_dataset.toSummaryString());
//		System.out.println(pca.toString());		
		return pca;
	}
		
	/**
	 * Return the roundness from the PCA, using the eigenvalues
	 */
	public double GetPCARoundness(PrincipalComponents pca){		

		// get the eigenvalues of the pca
		double[] eigenvalues = pca.getEigenValues();
		
		// get the nr of eigenvalues
		int length = eigenvalues.length;
		
		// return the roundness by dividing the before last value with the last
		// (the last [length-1] being the greatest)
		return eigenvalues[length-2] / eigenvalues[length-1];
	}
	
	/**
	 * Return the roundness of the pancake at the given timestamp
	 */
	public double GetPancakeRoundness(
			String ts_str,
			String model_name){
		
		// get the links positions
		List<Point3d> points = this.GetLinksPositions(ts_str, model_name);
		
		// flag for centering the pca data
		boolean center_data = true;
		
		// get the pca of the points
		PrincipalComponents pca = this.GetPCA(points, center_data);
		
		// return the roundness of the pancake
		return this.GetPCARoundness(pca);		
	}
	
	/**
	 * Check if the model has been flipped between the timestamps
	 */
	public boolean CheckModelFlip(
			String start_str,
			String end_str,
			String model_name){

		// get the links positions
		List<Point3d> start_points = this.GetLinksPositions(start_str, model_name);

		// the indexes of the points used for the plane
		int index1 = -1;
		int index2 = -1;
		int index3 = -1;
		
		// get three relative distant points from the 
		// links to compute the normal of its plane
		outerloop:
			for (int i = 0; i < start_points.size(); i++){
				// get the first point
				Point3d first_point = start_points.get(i);			

				for (int j = 0; j < start_points.size(); j++){
					// get the second point and check if the distance is big enough
					Point3d second_point = start_points.get(j);
					if (first_point.distance(second_point) > 0.04){

						for (int k = 0; k < start_points.size(); k++){
							// get the third point and check if distances are big enough
							Point3d third_point = start_points.get(k);							
							if (first_point.distance(third_point) > 0.04 &&
									second_point.distance(third_point) > 0.04){						
															
								// set the indexes for the plane
								index1 = i;
								index2 = j;
								index3 = k;
							
								break outerloop;
							}
						}
					}
				}
			}

		// continue if three points have been found
		if ((index1 != -1) && (index2 != -1) && (index3 != -1))
		{
			/* START TIMESTAMP */	
			// get the three points from the start, cast from Point3d to Tuple3d
			Tuple3d start_p1 = (Tuple3d) start_points.get(index1);
			Tuple3d start_p2 = (Tuple3d) start_points.get(index2);
			Tuple3d start_p3 = (Tuple3d) start_points.get(index3);
			
			// subtract from points 2 and 3 point 1 to get the plane vectors (p2<--p1 , p3<--p1)
			start_p2.sub(start_p1);
			start_p3.sub(start_p1);
			
			// set the two plane vectors v1 = p2<--p1;  v2 = p3<--p1
			Vector3d start_v1 = new Vector3d(start_p2);
			Vector3d start_v2 = new Vector3d(start_p3);
			
			start_v1.normalize();
			start_v2.normalize();
			
			// compute the normal vector
			Vector3d start_normal = new Vector3d();			
			start_normal.cross(start_v1, start_v2);

			
			/* END TIMESTAMP */			
			// get the links positions at the end
			List<Point3d> end_points = this.GetLinksPositions(end_str, model_name);

			// get the three points from the end, cast from Point3d to Tuple3d
			Tuple3d end_p1 = (Tuple3d) end_points.get(index1);
			Tuple3d end_p2 = (Tuple3d) end_points.get(index2);
			Tuple3d end_p3 = (Tuple3d) end_points.get(index3);

			// subtract from points 2 and 3 point 1 to get the plane vectors (p2<--p1 , p3<--p1)
			end_p2.sub(end_p1);
			end_p3.sub(end_p1);
			
			// set the two plane vectors v1 = p2<--p1;  v2 = p3<--p1
			Vector3d end_v1 = new Vector3d(end_p2);
			Vector3d end_v2 = new Vector3d(end_p3);
			
			end_v1.normalize();
			end_v2.normalize();
			
			// compute the normal vector
			Vector3d end_normal = new Vector3d();			
			end_normal.cross(end_v1, end_v2);
					
			// compute the dot product between the start and end normal
			double dot_res = start_normal.dot(end_normal);

//			System.out.println("Dot prod:");
//			System.out.println(String.format("%.12f", dot_res));
			
			// if the dot product is negative the model has flipped
			if (dot_res < 0)
			{
				return true;
			}
		}
		return false;
	}

}

