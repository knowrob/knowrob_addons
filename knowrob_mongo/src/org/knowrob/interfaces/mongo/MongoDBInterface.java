package org.knowrob.interfaces.mongo;

import com.mongodb.MongoClient;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.DBCursor;

import java.net.UnknownHostException;
import java.sql.Timestamp;

import ros.communication.Time;
import tfjava.StampedTransform;

public class MongoDBInterface {

	// duration through which transforms are to be kept in the buffer
	protected final static int BUFFER_SIZE = 5;
	
	MongoClient mongoClient;
	DB db;

	TFMemory mem;

	public MongoDBInterface() {

		try {
			mongoClient = new MongoClient( "localhost" , 27017 );
			db = mongoClient.getDB("roslog-pr2");
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		mem = TFMemory.getInstance();
	}
	
	
	public double[] getPose(String name) {
		
		double[] res = new double[2];
		
		String coll_name = name + "_pose";
		
		DBCollection coll = db.getCollection(coll_name);
		
		DBObject query = new BasicDBObject();
		DBObject cols  = new BasicDBObject();
		cols.put("x", 1 );
		cols.put("y",  1 );
		
		
		
		DBCursor cursor = coll.find(query, cols );
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {
				
				DBObject row = cursor.next();
				res[0] = Double.valueOf(row.get("x").toString());
				res[1] = Double.valueOf(row.get("y").toString());
				break;
			}
		} finally {
			cursor.close();
		}

		return res;
	}


//	public StampedTransform lookupTransform(String targetFrameId, String sourceFrameId, Time time) {
//		return lookupTransform(targetFrameId, sourceFrameId, new ISO8601Date(time).getDate());
//	}
//
//	public StampedTransform lookupTransform(String targetFrameId, String sourceFrameId, Date date) {
//
//		StampedTransform res = loadTransformFromDB(targetFrameId, sourceFrameId, date);
//		res = mem.lookupTransform(targetFrameId, sourceFrameId, new ISO8601Date(date).getROSTime());
//
//		return res;
//	}



	public static void main(String[] args) {

		MongoDBInterface m = new MongoDBInterface();
		System.out.println("pose: [" + m.getPose("turtle1")[0] + ", " + m.getPose("turtle1")[1] + "]");
		
		Timestamp timestamp = Timestamp.valueOf("2013-07-24 11:44:01.0");
		Time t = new Time(timestamp.getTime()/1000);
		
		TFMemory tf = new TFMemory();
		StampedTransform trans = tf.lookupTransform("/sensor_mount_link", "/head_mount_kinect_ir_link", t);
		System.out.println(trans);
		
	}
}

