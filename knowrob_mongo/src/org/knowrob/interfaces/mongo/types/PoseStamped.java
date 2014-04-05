package org.knowrob.interfaces.mongo.types;

import java.util.Date;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;


import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class PoseStamped extends ros.pkg.geometry_msgs.msg.PoseStamped {

	public PoseStamped() {
		super();
	}
	
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of PoseStamped with values from row
	 */
	public PoseStamped readFromDBObject(DBObject row) {
		
		BasicDBObject header = (BasicDBObject) row.get("header");
		BasicDBObject pose   = (BasicDBObject) row.get("pose");

		// return null for empty poses
		if(pose == null)
			return null;
		
		if(header != null)
		{
			this.header.frame_id = header.getString("frame_id");
			this.header.seq= header.getLong("seq");
			this.header.stamp = new ISODate((Date) header.get("stamp")).toROSTime();
		}
		
		this.pose.position.x = ((BasicDBObject) pose.get("position")).getDouble("x");
		this.pose.position.y = ((BasicDBObject) pose.get("position")).getDouble("y");
		this.pose.position.z = ((BasicDBObject) pose.get("position")).getDouble("z");
		
		this.pose.orientation.x = ((BasicDBObject) pose.get("orientation")).getDouble("x");
		this.pose.orientation.y = ((BasicDBObject) pose.get("orientation")).getDouble("y");
		this.pose.orientation.z = ((BasicDBObject) pose.get("orientation")).getDouble("z");
		this.pose.orientation.w = ((BasicDBObject) pose.get("orientation")).getDouble("w");
		
		return this;
	}
	
	/**
	 * Transform pose to matrix representation 
	 * 
	 * @return Matrix4d representing this pose
	 */
	public Matrix4d getMatrix4d() {
		
		Quat4d q = new Quat4d(this.pose.orientation.x, 
							  this.pose.orientation.y,
							  this.pose.orientation.z,
							  this.pose.orientation.w);
		
		Vector3d t = new Vector3d(this.pose.position.x,
								  this.pose.position.y,
								  this.pose.position.z);
		
		return new Matrix4d(q, t, 1);
	}
	
}
