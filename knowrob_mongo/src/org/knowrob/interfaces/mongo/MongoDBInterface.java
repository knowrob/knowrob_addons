package org.knowrob.interfaces.mongo;

import com.mongodb.MongoClient;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.DBCursor;
import com.mongodb.QueryBuilder;
import com.mongodb.util.Util;

import java.net.UnknownHostException;
import java.sql.Timestamp;
import java.util.Date;

import org.knowrob.interfaces.mongo.util.ISO8601Date;

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


	private StampedTransform loadTransformFromDB(String childFrameID, Date date) {
		
		DBCollection coll = db.getCollection("tf");
		DBObject query = new BasicDBObject();
		
		// select time slice from BUFFER_SIZE seconds before to one second after given time
		Date start = new Date(date.getTime()-BUFFER_SIZE * 1000);
		Date end   = new Date(date.getTime() + 1000);
		
		query = QueryBuilder.start("transforms")
				.elemMatch(new BasicDBObject("child_frame_id", childFrameID))
				.and("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end )
				.get();

		DBObject cols  = new BasicDBObject();
		cols.put("_id", 1 );
		cols.put("__recorded",  1 );
		cols.put("transforms",  1 );


		DBCursor cursor = coll.find(query, cols );
		cursor.sort(new BasicDBObject("__recorded", -1));
		
		
		StampedTransform res = null;
		try {
			int i = 0;
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				System.out.println(row.get("__recorded").toString() + " " + i++);

				mem.setTransforms(row.get("transforms").toString());

				break;
			}
		} finally {
			cursor.close();
		}
		return res;
	}

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

