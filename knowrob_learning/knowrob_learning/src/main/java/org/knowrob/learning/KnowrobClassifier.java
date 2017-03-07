package org.knowrob.learning;

import java.lang.Double;

import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

       
import org.knowrob.interfaces.mongo.MongoDBInterface;

public abstract class KnowrobClassifier
{
	public static KnowrobClassifier setAClassifier(Object classifier, String[] attrNames, float[][] trainInstances) throws Exception
  	{	
    		if (classifier.getClass().getName().startsWith("weka")) 
		{
			WekaKnowrobClassifier wkc = new WekaKnowrobClassifier(classifier, attrNames, trainInstances); 
						
      			return wkc;
    		}	
    		else 
		{
      			throw new Exception("other classifiers not implementet yet");
    		}
  	}

	public static KnowrobClassifier setAClassifier(Object classifier, String[] attrNames, String[][] trainInstances) throws Exception
  	{	
    		if (classifier.getClass().getName().startsWith("weka")) 
		{
			WekaKnowrobClassifier wkc = new WekaKnowrobClassifier(classifier, attrNames, trainInstances); 
						
      			return wkc;
    		}	
    		else 
		{
      			throw new Exception("other classifiers not implementet yet");
    		}
  	}

	public void saveToFile(String filename) throws IOException
  	{
    		ObjectOutputStream streamer = new ObjectOutputStream(new FileOutputStream(filename));
    		streamer.writeObject(this);
    		streamer.close();
  	}

	public static KnowrobClassifier loadFromFile(String filename) throws IOException, ClassNotFoundException
  	{
    		ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename));
    		KnowrobClassifier wc = (KnowrobClassifier)ois.readObject();
    		ois.close();
    		return wc;
  	}

	public abstract Object classify(float[][] testData);
}
