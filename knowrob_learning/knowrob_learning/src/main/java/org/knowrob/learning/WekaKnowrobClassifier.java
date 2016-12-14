package org.knowrob.learning;

import java.lang.Double;
import java.util.ArrayList;
import java.io.*;
import weka.core.*;
import weka.filters.*;
import weka.attributeSelection.*;
import weka.classifiers.Evaluation;
import weka.classifiers.trees.J48;
import weka.classifiers.functions.*;
import weka.classifiers.Classifier; 
import weka.classifiers.Evaluation;
import weka.core.converters.ArffSaver;
import java.util.Random;
import java.util.List;
import java.util.ArrayList;

import org.knowrob.interfaces.mongo.MongoDBInterface;
import org.knowrob.interfaces.mongo.*;


public class WekaKnowrobClassifier extends KnowrobClassifier implements Serializable
{
	private static final long serialVersionUID = -8991580797375760587L;
	private Instances trainData;
	private String[][] options;
	private Classifier classifier;
	private FastVector attributes;

	public WekaKnowrobClassifier(Object classifier, String[] attrNames, float[][] trainData)
	{
		Classifier clsfr = (Classifier) classifier;

		attributes = new FastVector();
		for(int i = 0; i < attrNames.length; i++)
		{
			if(i < attrNames.length - 1)
			{
				Attribute attr1 = new Attribute(attrNames[i]);
				attributes.addElement(attr1);
			}
			else
			{
				List possibleValues = new ArrayList();
				possibleValues.add("value_" + 1);
				possibleValues.add("value_" + 2);
				
				Attribute attr1 = new Attribute(attrNames[i], possibleValues);
				attributes.addElement(attr1);
			}
				
		}

		try
		{
			Instances insts = arrayToInstances(trainData, "Training data", false);
			this.trainData = insts;
			this.classifier = clsfr;
		}
		catch (Exception e)
		{
			System.out.println("The number of attributes should be same as the number of features in the dataset");
		}

	}

	public String classify(float[][] testData)
	{
		try
		{	
			Instances insts = arrayToInstances(testData, "Testing data", true);
			trainData.setClassIndex(trainData.numAttributes() - 1);
			classifier.buildClassifier(trainData);
			insts.setClassIndex(trainData.numAttributes() - 1);
			String output = "";

			for (int i = 0; i < insts.numInstances(); i++) {
				double pred = classifier.classifyInstance(insts.instance(i));
				output += pred + " ";	
			}


 
			//Evaluation eval = new Evaluation(trainData);
 			//eval.evaluateModel(classifier, insts);
			//String output = eval.toSummaryString();
			return output;
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}

		return "error!";
		
	}	

	public String crossValidate()
	{
		try
		{
			trainData.setClassIndex(trainData.numAttributes() - 1); 
			Evaluation eval = new Evaluation(trainData);
 			eval.crossValidateModel(classifier, trainData, 10, new Random(1));
			String output = eval.toCumulativeMarginDistributionString();
			PrintWriter writer = new PrintWriter( "/home/ros/user_data/svm.plt", "UTF-8");
			writer.println(output);

			ArffSaver saver = new ArffSaver();
			saver.setInstances(trainData);
			saver.setFile(new File("/home/ros/user_data/test.arff"));
			saver.setDestination(new File("/home/ros/user_data/test.arff"));   // **not** necessary in 3.5.4 and later
 			saver.writeBatch();

			return output;
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}

		return "";

	}



	public Instances arrayToInstances (float[][] arrayEntries, String instancesName, boolean isTesting) throws Exception
	{
		if(attributes.size() != arrayEntries[0].length && !isTesting)
			throw new Exception("The number of attributes should be same as the number of features in the dataset");
		else if(attributes.size() < arrayEntries[0].length && isTesting)
			throw new Exception("The number of attributes should be same as the number of features in the dataset");

		Instances instances = new Instances(instancesName, attributes, 0);

		for(int i = 0; i < arrayEntries.length; i++)
		{
			double[] doubleEntries = new double[attributes.size()];
			//Instance current = new Instance(attributes.size());
			for(int j = 0; j < attributes.size(); j++)
			{
				//doubleEntries[j] = (double) arrayEntries[i][j];
				if(j < attributes.size() - 1)
					//current.setValue(j, (double) arrayEntries[i][j]);
					doubleEntries[j] = (double) arrayEntries[i][j];
				else if(!isTesting)
				{
					if(arrayEntries[i][j] == 1.0)
						//current.setValue(j, "Success");
						doubleEntries[j] = instances.attribute(j).indexOfValue("value_1");
					else
						//current.setValue(j, "Failed");
						doubleEntries[j] = instances.attribute(j).indexOfValue("value_2");
				}	
				else
				{
						doubleEntries[j] = Utils.missingValue();

				}
			}

			instances.add(new DenseInstance(1.0, doubleEntries));
		}
		return instances;

	}

	
	
}

