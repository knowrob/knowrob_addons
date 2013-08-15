package org.knowrob.interfaces.mongo;

import com.mongodb.MongoClient;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.DBCursor;
import com.mongodb.QueryBuilder;

import java.net.UnknownHostException;
import java.sql.Timestamp;
import java.util.Date;

import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.interfaces.mongo.types.ISODate;

import ros.communication.Time;
import tfjava.Stamped;
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
			db = mongoClient.getDB("roslog");
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

	/**
	 * Wrapper around the lookupTransform method of the TFMemory class
	 * 
	 * @param sourceFrameId ID of the source frame of the transformation
	 * @param targetFrameId ID of the target frame of the transformation
	 * @param posix_ts POSIX timestamp (seconds since 1.1.1970)
	 * @return
	 */
	public StampedTransform lookupTransform(String targetFrameId, String sourceFrameId, int posix_ts) {
		
		Time t = new Time();
		t.secs = posix_ts;
		return(mem.lookupTransform(targetFrameId, sourceFrameId, t));
	}


	public boolean transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
		return mem.transformPose(targetFrameID, stampedIn, stampedOut);
	}


	public Designator latestUIMAPerceptionBefore(int posix_ts) {
		
		Designator res = null;		
		DBCollection coll = db.getCollection("uima_uima_results");

		// read all events up to one minute before the time
		Date start = new ISODate((long) 1000 * (posix_ts - 60) ).getDate();
		Date end   = new ISODate((long) 1000 * posix_ts ).getDate();

		DBObject query = QueryBuilder
				.start("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end ).get();

		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );
		
		DBCursor cursor = coll.find(query, cols );
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {
				
				DBObject row = cursor.next();
				res = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				break;
			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return res;
	}

	// gets perception of given object at given time within a 1 minute interval
	public Designator getUIMAPerception(String object, int posix_ts) {
		
		Designator res = null;		
		DBCollection coll = db.getCollection("uima_uima_results");

		// read all events up to one minute before the time
		Date start = new ISODate((long) 1000 * (posix_ts - 30) ).getDate();
		Date end   = new ISODate((long) 1000 * (posix_ts + 30) ).getDate();

		DBObject query = QueryBuilder
				.start("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end )
				.and("designator.ObjectID").is(object).get();

		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );		

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {
				
				DBObject row = cursor.next();
				res = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				break;
			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return res;
	}
	

	public static void main(String[] args) {

		MongoDBInterface m = new MongoDBInterface();
		
		
		// test transformation lookup based on DB information
		
		Timestamp timestamp = Timestamp.valueOf("2013-07-26 14:27:22.0");
		Time t = new Time(timestamp.getTime()/1000);
		
		long t0 = System.nanoTime();
		TFMemory tf = new TFMemory();
		StampedTransform trans  = tf.lookupTransform("/base_bellow_link", "/head_mount_kinect_ir_link", t);
		System.out.println(trans);
		long t1 = System.nanoTime();
		StampedTransform trans2 = tf.lookupTransform("/base_link", "/head_mount_kinect_ir_link", t);
		System.out.println(trans2);
		long t2 = System.nanoTime();
		
		double first = (t1-t0)/ 1E6;
		double second = (t2-t1)/ 1E6;
		
		System.out.println("Time to look up first transform: " + first + "ms");
		System.out.println("Time to look up second transform in same time slice: " + second + "ms");
		
		// test lookupTransform wrapper
		trans = m.lookupTransform("/base_link", "/head_mount_kinect_ir_link",  1376484439 );
		System.out.println(trans);
		
		// test UIMA result interface
		Designator d = m.latestUIMAPerceptionBefore(1374841669);
		System.out.println(d);
	}
}

