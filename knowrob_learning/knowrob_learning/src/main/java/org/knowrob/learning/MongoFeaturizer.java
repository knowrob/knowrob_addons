package org.knowrob.learning;

import java.util.ArrayList;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;
import com.mongodb.DB;
import com.mongodb.DBCollection;
import com.mongodb.DBCursor;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

import org.knowrob.interfaces.mongo.MongoDBInterface;

public class MongoFeaturizer
{
	static MongoDBInterface mongo;

	public static ArrayList<String[]> getInstancesFromMongo(String collection, String[] keys, String[] relations, Object[] values, String[] fields)
	{
		ArrayList<String[]> result = new ArrayList<String[]>();

		if (mongo == null)
			mongo = new MongoDBInterface();
 
		try 
		{
			
			DBCursor cursor = mongo.query(collection, keys, relations, values);
			
			while(cursor.hasNext())
			{
				DBObject row = cursor.next();
				String[] instanceValues = new String[fields.length];
				int i = 0;
				for(String key : fields)
				{
					Object val = row.get(key);
					
					if(val != null)
					{
						instanceValues[i] = val.toString();
					}
					else
					{
						instanceValues[i] = "0";
					}
					i++;
				}
				result.add(instanceValues);

			}
			cursor.close();		

		}
		catch(Exception e) {
			 System.err.println(e.getMessage());
			 e.printStackTrace();
		}
		return result;

	}

}
