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

	public WekaKnowrobClassifier(Object classifier, String[] attrNames, String[][] trainData)
	{
		Classifier clsfr = (Classifier) classifier;

		attributes = new FastVector();

		//int different_labels = 0;
		/*List mapping = new ArrayList();
		for(int i = 0; i < trainData.length; i++)
		{

			boolean isSameFound = false;
			for(int j = 0; j < i; j++)
			{
				if(trainData[j][trainData[j].length - 1].equals(trainData[i][trainData[i].length - 1]))
				{

					isSameFound = true;
					break;
				}
			}
			if(!isSameFound)
			{ 
				mapping.add(trainData[i][trainData[i].length - 1]);
				//different_labels++;
			}
		}*/


		List[] mapping = new List[trainData[0].length];

		for(int k = 0; k < trainData[0].length; k++)
		{
			mapping[k] = new ArrayList();

			for(int i = 0; i < trainData.length; i++)
			{
				boolean isSameFound = false;
				for(int j = 0; j < i; j++)
				{
					if(trainData[j][k].equals(trainData[i][k]))
					{

						isSameFound = true;
						break;
					}
				}
				if(!isSameFound)
				{ 
					mapping[k].add(trainData[i][k]);
					//different_labels++;
				}
			}
		}

		for(int i = 0; i < attrNames.length; i++)
		{
			if(i < attrNames.length - 1)
			{
				Attribute attr1 = new Attribute(attrNames[i], mapping[i]);
				attributes.addElement(attr1);
			}
			else
			{
				List possibleValues = new ArrayList();

				//for(int j = 0; j < i; j++)
				//{
				//	possibleValues.add("value_" + j);
				//}

				//possibleValues.add("value_" + 1);
				//possibleValues.add("value_" + 2);
				
				Attribute attr1 = new Attribute(attrNames[i], mapping[i]);
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
			e.printStackTrace();
		}

	}
	

	public WekaKnowrobClassifier(Object classifier, String[] attrNames, float[][] trainData)
	{
		Classifier clsfr = (Classifier) classifier;

		attributes = new FastVector();

		//int different_labels = 0;
		List mapping = new ArrayList();
		for(int i = 0; i < trainData.length; i++)
		{

			boolean isSameFound = false;
			for(int j = 0; j < i; j++)
			{
				if(trainData[j][trainData[j].length - 1] == trainData[i][trainData[i].length - 1])
				{

					isSameFound = true;
					break;
				}
			}
			if(!isSameFound)
			{ 
				mapping.add(trainData[i][trainData[i].length]);
				//different_labels++;
			}
		}



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

				//for(int j = 0; j < i; j++)
				//{
				//	possibleValues.add("value_" + j);
				//}

				//possibleValues.add("value_" + 1);
				//possibleValues.add("value_" + 2);
				
				Attribute attr1 = new Attribute(attrNames[i], mapping);
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

	public String classify(String[][] testData)
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

	public void visualizeGraph()
	{
		try
		{
			J48 j48 = (J48)classifier;
	
			trainData.setClassIndex(trainData.numAttributes() - 1);
			j48.buildClassifier(trainData);
			
			PrintWriter writer = new PrintWriter( "/home/ros/user_data/dt.dot", "UTF-8");
			System.out.println(j48.graph());
			writer.println(j48.graph());

			Process p = Runtime.getRuntime().exec("dot -Tpng /home/ros/user_data/dt.dot > /home/ros/user_data/dt.png", null, new File("/home/ros/user_data/"));
			
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		
	}	

	public String crossValidate()
	{
		try
		{
			trainData.setClassIndex(trainData.numAttributes() - 1); 
			Evaluation eval = new Evaluation(trainData);
 			eval.crossValidateModel(classifier, trainData, 10, new Random(1));
			String output = eval.toSummaryString();
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

	public Instances arrayToInstances (String[][] arrayEntries, String instancesName, boolean isTesting) throws Exception
	{
		if(attributes.size() != arrayEntries[0].length && !isTesting)
			throw new Exception("The number of attributes should be same as the number of features in the dataset");
		else if(attributes.size() < arrayEntries[0].length && isTesting)
			throw new Exception("The number of attributes should be same as the number of features in the dataset");

		Instances instances = new Instances(instancesName, attributes, 0);
		instances.setClassIndex(instances.numAttributes() - 1);

		for(int i = 0; i < arrayEntries.length; i++)
		{
			/*double[] doubleEntries = new double[attributes.size()];
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
			}*/

			DenseInstance current = new DenseInstance(attributes.size());
			current.setDataset(instances);

			boolean isAnswer = false;

			for(int j = 0; j < attributes.size(); j++)
			{
				if(j < attributes.size() - 1)
					current.setValue(j, arrayEntries[i][j]);
				else if(!isTesting)
				{
					if(arrayEntries[i][j].contains("UERY"))
					{
						isAnswer=true;
						break;
					}
					
					current.setValue(j, arrayEntries[i][j]);
				}	
				else
				{
					current.setValue(j, Utils.missingValue());

				}
			}
			 
			if(!isAnswer) instances.add(current);
		}
		return instances;

	}
	
}

