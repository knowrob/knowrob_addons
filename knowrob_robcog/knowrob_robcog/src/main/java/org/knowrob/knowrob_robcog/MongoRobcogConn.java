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

import java.util.Map;
import java.util.Set;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.MongoClient;
import java.net.UnknownHostException;

public class MongoRobcogConn{
	
	// mongo connection to the db 
	private MongoClient mongoClient;
	// mongo db
	private DB db;	
	// host name
	private String dbHost;	
	// selected collection
	public DBCollection coll;	

	/**
	 * MongoRobcogConn constructor
	 */
	public MongoRobcogConn() {		
		// check if MONGO_PORT_27017_TCP_ADDR and MONGO_PORT_27017_TCP_PORT 
		// environment variables are set		
        Map<String, String> env = System.getenv();
        
        // default port
        int port = 27017;
        
        // Get host address
        if(env.containsKey("MONGO_PORT_27017_TCP_ADDR")) {
        	this.dbHost = env.get("MONGO_PORT_27017_TCP_ADDR");
        }
        else{
        	this.dbHost = "localhost";        	
        }
        
        // Get host port
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

	/**
	 * Set the database to be queried
	 */
	public void SetDatabase(String db_name){
		// set db
		this.db = mongoClient.getDB(db_name);

		// Print out all its available collections
		System.out.println("Java - Db: " + this.db.getName() + ", available collections: ");		
		Set<String> all_collections = this.db.getCollectionNames();		
		for (String coll_name : all_collections){
			System.out.println("\t * " + coll_name);
		}
	}

	/**
	 * Set the collection to be queried
	 */
	public void SetCollection(String coll_name){
		// set the collection
		this.coll = this.db.getCollection(coll_name);		
		System.out.println("Java - Curr selected collection: " + this.coll.getName());
	}
}

