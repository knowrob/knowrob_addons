package org.knowrob.learning;

import java.lang.Double;
import java.util.ArrayList;
import java.io.*;
import weka.core.*;
import weka.filters.*;
import weka.attributeSelection.*;
import weka.classifiers.Evaluation;
import weka.classifiers.trees.J48;

import org.knowrob.interfaces.mongo.MongoDBInterface;
import org.knowrob.interfaces.mongo.*;


public class WekaClassifier extends Classifier implements Serializable
{
	private static final long serialVersionUID = -8991580797375760587L;
	private Instances train;

	public WekaClassifier(double[] options, Instances trainData) throws Exception
	{
		this.train = trainData;
	}

	public Object classify(String[][] testData)
	{
		return null;
	}	

	
	
}

