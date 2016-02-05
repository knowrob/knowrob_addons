package org.knowrob.learning;

import java.lang.Double;

import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import weka.core.*;
import weka.filters.*;
import weka.attributeSelection.*;

import org.knowrob.interfaces.mongo.MongoDBInterface;

public abstract class Classifier
{
	public static Classifier getAClassifier(String type, double[] optionsForClassifier, Instances trainInstances) throws Exception
  	{
    		if (type.startsWith("weka.J48")) 
		{
						
      			return null;
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

	public static Classifier loadFromFile(String filename) throws IOException, ClassNotFoundException
  	{
    		ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename));
    		WekaClassifier wc = (WekaClassifier)ois.readObject();
    		ois.close();
    		return wc;
  	}

	public abstract Object classify(String[][] testData);
}
