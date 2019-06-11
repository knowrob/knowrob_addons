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

public abstract class AttributeSelector
{
	public static AttributeSelector getASelector(String[] type, double[] options, String[][] data) throws Exception
  	{
    		if (type[0].startsWith("weka.relieff")) 
		{
			if(type[1].startsWith("weka.ranker"))
			{
				Ranker rnk = new Ranker();
				ReliefFAttributeEval relief = new ReliefFAttributeEval();
				if(options != null && options.length > 2)
				{
					if(options[0] > 0)
						rnk.setGenerateRanking(true);
					else
						rnk.setGenerateRanking(false);
					rnk.setNumToSelect(((int)options[1]));
					rnk.setThreshold(options[2]);
				}
				else
				{
					rnk.setGenerateRanking(true);
					rnk.setNumToSelect(-1);
					rnk.setThreshold(-1.8);
				}
				return new WekaReliefFSelector(relief, rnk, data, options);
			}
			else
			{
				throw new Exception("others search methods not implemented yet");
			}			

      			
    		}	
    		else 
		{
      			throw new Exception("other attribute selectors not implementet yet");
    		}
  	}

	public void saveToFile(String filename) throws IOException
  	{
    		ObjectOutputStream streamer = new ObjectOutputStream(new FileOutputStream(filename));
    		streamer.writeObject(this);
    		streamer.close();
  	}

	public static AttributeSelector loadFromFile(String filename) throws IOException, ClassNotFoundException
  	{
    		ObjectInputStream ois = new ObjectInputStream(new FileInputStream(filename));
    		WekaAttributeSelector wr = (WekaAttributeSelector)ois.readObject();
    		ois.close();
    		return wr;
  	}


	public abstract Instances applyFeatureSelection();
}
