package org.knowrob.interfaces.mongo.types;

import java.util.Date;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import com.mongodb.BasicDBObject;

import tfjava.Frame;

public class TransformStorage extends tfjava.TransformStorage {

	/**
	 * Normal constructor 
	 * 
	 * @param translation
	 * @param rotation
	 * @param timeStamp
	 * @param parentFrame
	 * @param childFrame
	 */
    public TransformStorage(Vector3d translation, Quat4d rotation, long timeStamp, Frame parentFrame, Frame childFrame) {
        super(translation, rotation, timeStamp, parentFrame, childFrame);
    }   

    /**
     * Simple constructor. To be used with "readFromDBObject()
     */
    public TransformStorage() {
    	super(new Vector3d(), new Quat4d(), 0l, new Frame("", 0), new Frame("", 0));
    }
    

	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of TransformStorage with values from row
	 */
	public TransformStorage readFromDBObject(BasicDBObject transform, Frame childFrame, Frame parentFrame) {
	
		this.childFrame  = childFrame;
		this.parentFrame = parentFrame;
		
		BasicDBObject pos = (BasicDBObject) ((BasicDBObject) transform.get("transform")).get("translation");
		this.translation.x = pos.getDouble("x");
		this.translation.y = pos.getDouble("y");
		this.translation.z = pos.getDouble("z");

		BasicDBObject rot = (BasicDBObject) ((BasicDBObject) transform.get("transform")).get("rotation");
		this.rotation.x = rot.getDouble("x");
		this.rotation.y = rot.getDouble("y");
		this.rotation.z = rot.getDouble("z");
		this.rotation.w = rot.getDouble("w");

		this.timeStamp = ((Date) ((BasicDBObject) transform.get("header")).get("stamp")).getTime();

		return this;
	}
}
