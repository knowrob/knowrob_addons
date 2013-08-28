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
import java.util.*;

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

	/** 
	 * Constructor
	 * 
	 * Initialize DB client and connect to database.
	 * 
	 */
	public MongoDBInterface() {

		try {
			mongoClient = new MongoClient( "localhost" , 27017 );
			db = mongoClient.getDB("roslog");
			
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		mem = TFMemory.getInstance();
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

	/**
	 * Wrapper around the transformPose method of the TFMemory class
	 * 
	 * @param targetFrameID  ID of the target frame of the transformation
	 * @param stampedIn      Stamped<Matrix4d> with the pose in the original coordinates
	 * @param stampedOut     Stamped<Matrix4d> that will hold the resulting pose
	 * @return               true if transform succeeded
	 */
	public boolean transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
		return mem.transformPose(targetFrameID, stampedIn, stampedOut);
	} 

	
	/**
	 * Read designator value from either the uima_uima_results collection 
	 * or the logged_designators collection.
	 * 
	 * @param designator Designator ID to be read
	 * @return Instance of a Designator
	 */
	public Designator getDesignatorByID(String designator) {
		
		for(String db_name : new String[]{"uima_uima_results", "logged_designators"}) {

			DBCollection coll = db.getCollection(db_name);
			DBObject query = QueryBuilder.start("designator.__id").is(designator).get();

			DBObject cols  = new BasicDBObject();
			cols.put("designator", 1 );				

			DBCursor cursor = coll.find(query, cols);
			
			while(cursor.hasNext()) {
				DBObject row = cursor.next();
				return new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
			}
			cursor.close();
		}
		return null;
	}

	
	/**
	 * Read the latest perception before the time point identified by posix_ts
	 * 
	 * @param posix_ts Time stamp in POSIX format (seconds since 1.1.1970)
	 * @return Designator object returned by the last perception before that time
	 */
	public Designator latestUIMAPerceptionBefore(int posix_ts) {

		Designator desig = null;		
		DBCollection coll = db.getCollection("uima_uima_results");

		// read all events up to one minute before the time
		Date start = new ISODate((long) 1000 * (posix_ts - 60) ).getDate();
		Date end   = new ISODate((long) 1000 * (posix_ts + 60) ).getDate();

		DBObject query = QueryBuilder
				.start("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end ).get();

		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {
				DBObject row = cursor.next();
				desig = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				break;
			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return desig;
	}

	
	/**
	 * Get all times when an object has been detected
	 * 
	 * @param object
	 * @return
	 */
	public List<Date> getUIMAPerceptionTimes(String object) {
		List<Date> times = new ArrayList<Date>();	
		DBCollection coll = db.getCollection("uima_uima_results");

		// TODO: This will always return a single result since the ID is unique
		DBObject query = QueryBuilder
				.start("designator.__id").is(object).get();

		DBObject cols  = new BasicDBObject();
		cols.put("__recorded", 1 );				

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				Date currentTime = (new ISODate(0).readFromDBObject((BasicDBObject) row.get("__recorded"))).getDate();
				times.add(currentTime);

			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return times;
	}

	
	/**
	 * 
	 * @param posix_ts
	 * @return
	 */
	public List<String> getUIMAPerceptionObjects(int posix_ts) {

		Designator res = null;

		Date start = new ISODate((long) 1000 * (posix_ts - 30) ).getDate();
		Date end   = new ISODate((long) 1000 * (posix_ts + 30) ).getDate();

		List<String> objects = new ArrayList<String>();	
		DBCollection coll = db.getCollection("uima_uima_results");

		DBObject query = QueryBuilder
				.start("__recorded").greaterThanEquals( start )
				.and("__recorded").lessThan( end ).get();


		DBObject cols  = new BasicDBObject();
		cols.put("designator", 1 );				

		DBCursor cursor = coll.find(query, cols);
		cursor.sort(new BasicDBObject("__recorded", -1));
		try {
			while(cursor.hasNext()) {

				DBObject row = cursor.next();
				res = new Designator().readFromDBObject((BasicDBObject) row.get("designator"));
				objects.add((String)res.get("__id"));

			}
		} catch(Exception e){
			e.printStackTrace();
		} finally {
			cursor.close();
		}
		return objects;
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
		trans = m.lookupTransform("/base_bellow_link", "/head_mount_kinect_ir_link", 1374841534);
		System.out.println(trans);

		// test UIMA result interface
		Designator d = m.latestUIMAPerceptionBefore(1374841669);
		System.out.println(d);
	}
}

