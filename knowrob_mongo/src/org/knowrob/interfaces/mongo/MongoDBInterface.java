package org.knowrob.interfaces.mongo;

import com.mongodb.MongoClient;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.DBCursor;

import java.net.UnknownHostException;

public class MongoDBInterface {

	MongoClient mongoClient;
	DB db;

	public MongoDBInterface() {

		try {
			mongoClient = new MongoClient( "localhost" , 27017 );
			db = mongoClient.getDB("roslog");
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
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


	public static void main(String[] args) {

		MongoDBInterface m = new MongoDBInterface();
		System.out.println("pose: [" + m.getPose("turtle1")[0] + ", " + m.getPose("turtle1")[1] + "]");

	}
}

