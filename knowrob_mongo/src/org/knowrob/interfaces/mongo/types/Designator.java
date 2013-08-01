package org.knowrob.interfaces.mongo.types;

import java.util.LinkedHashMap;
import java.util.Map;


import com.mongodb.BasicDBObject;

public class Designator {

	protected Map<String, Object> values;
	
	public Designator() {
		values = new LinkedHashMap<String, Object>();
	}

	public boolean containsKey(Object arg0) {
		return values.containsKey(arg0);
	}

	public Object get(Object arg0) {
		return values.get(arg0);
	}

	public boolean isEmpty() {
		return values.isEmpty();
	}

	public Object put(String arg0, Object arg1) {
		return values.put(arg0, arg1);
	}

	public int size() {
		return values.size();
	}

	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Designator with values from row
	 */
	public Designator readFromDBObject(BasicDBObject row) {

		for(String key : row.keySet()) {
			
			Object val = null;
			if(key.equals("pose")) {
				val = new PoseStamped().readFromDBObject((BasicDBObject) row.get(key));
				
			} else if(key.equals("type")) {
				val = row.getString(key);
			}
			
			if(val!=null)
				values.put(key, val);
		}
		return this;
	}
}
