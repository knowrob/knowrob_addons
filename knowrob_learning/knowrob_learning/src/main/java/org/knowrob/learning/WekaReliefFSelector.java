package org.knowrob.learning;

import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.io.PrintStream;
import java.io.Serializable;
import weka.classifiers.Classifier;
import weka.core.Attribute;
import weka.core.FastVector;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.Utils;
import weka.attributeSelection.*;

public class WekaReliefFSelector extends WekaAttributeSelector implements Serializable
{
	private static final long serialVersionUID = -8991580797375760583L;
  	//private ASSearch s;
	//private ReliefFAttributeEval relief;
  	private Instances insts;
	private Attribute[] attributes;

	public WekaReliefFSelector(ReliefFAttributeEval relief, Ranker searcher, String[][] data, double[] options) throws Exception
	{
		super((ASEvaluation)relief, (ASSearch) searcher, data);

		if(options != null)
		{
			relief.setNumNeighbours((int)options[4]);
			relief.setSampleSize((int)options[5]);
			relief.setSeed((int)options[6]);
			relief.setSigma((int)options[7]);

			if(options[3] > 0)
				relief.setWeightByDistance(true);
			else
				relief.setWeightByDistance(false);

		}
		else
		{
			relief.setNumNeighbours(10);
			relief.setSampleSize(-1);
			relief.setSeed(1);
			relief.setSigma(2);
			relief.setWeightByDistance(false);
		}
		
		
	}	

	/*public Instances applyFeatureSelection()
	{
		return null;
	}*/

	public static void main(String[] args)
	{
		String[] keys = new String[1];
		keys[0] = "_id";

		Object[] vals = new Object[1];
		vals[0] = "action_KIyM5lgFaU8cum";

		String[] rels = new String[1];
		rels[0] = "==";

		String[] fields = new String[1];
		fields[0] = "_annotation";

		String[] labels = new String[1];
		labels[0] = "A";

		ArrayList<String[]> testers = MongoFeaturizer.getInstancesFromMongo("chemlab", keys, rels, vals, fields);

		System.out.println(testers.size());

		/*MongoDBInterface mng = new MongoDBInterface();

		MongoDBInterface mng = new MongoDBInterface();

		WekaReliefFSelector wrs = new WekaReliefFSelector(mng); 

		String[] keys = new String[1];
		keys[0] = "_id";

		Object[] vals = new Object[1];
		vals[0] = "action_KIyM5lgFaU8cum";

		String[] rels = new String[1];
		rels[0] = "==";

		String[] fields = new String[1];
		fields[0] = "_annotation";

		String[] labels = new String[1];
		labels[0] = "A";

		wrs.getInstances("chemlab", keys, rels, vals, fields, labels);*/
	}
}
