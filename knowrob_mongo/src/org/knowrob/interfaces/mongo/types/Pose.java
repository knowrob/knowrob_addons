package org.knowrob.interfaces.mongo.types;

import java.util.Date;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;


import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;


public class Pose extends ros.pkg.geometry_msgs.msg.Pose {

	public Pose() {
		super();
	}
	
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Pose with values from row
	 */
	public Pose readFromDBObject(DBObject row) {	
			
		this.position.x = ((BasicDBObject) row.get("position")).getDouble("x");
		this.position.y = ((BasicDBObject) row.get("position")).getDouble("y");
		this.position.z = ((BasicDBObject) row.get("position")).getDouble("z");
		
		this.orientation.x = ((BasicDBObject) row.get("orientation")).getDouble("x");
		this.orientation.y = ((BasicDBObject) row.get("orientation")).getDouble("y");
		this.orientation.z = ((BasicDBObject) row.get("orientation")).getDouble("z");
		this.orientation.w = ((BasicDBObject) row.get("orientation")).getDouble("w");
		return this;
	}
	
	/**
	 * Transform pose to matrix representation 
	 * 
	 * @return Matrix4d representing this pose
	 */
	public Matrix4d getMatrix4d() {
		
		Quat4d q = new Quat4d(this.orientation.x, 
							  this.orientation.y,
							  this.orientation.z,
							  this.orientation.w);
		
		Vector3d t = new Vector3d(this.position.x,
								  this.position.y,
								  this.position.z);
		
		return new Matrix4d(q, t, 1);
	}
	
}
