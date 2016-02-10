package org.knowrob.learning;

import java.lang.Double;
import java.io.*;
import weka.core.*;
import weka.filters.*;
import weka.attributeSelection.*;

import org.knowrob.interfaces.mongo.MongoDBInterface;
import org.knowrob.interfaces.mongo.*;

import org.apache.commons.lang.ArrayUtils;

public class WekaAttributeSelector extends AttributeSelector implements Serializable
{
	private static final long serialVersionUID = -8991580797375760585L;
	private ASEvaluation evaluater;
	private ASSearch searcher;
	private Instances instances;
  	private Attribute[] attributes;

	public WekaAttributeSelector(ASEvaluation evaluater, ASSearch searcher, String[][] data) throws Exception
	{
		this.evaluater = evaluater;
		this.searcher = searcher;
		loadData(data);
	}

	public WekaAttributeSelector(String evaluaterPath, String searcherPath, String[][] data) throws Exception
	{
		ObjectInputStream ois_e = new ObjectInputStream(new FileInputStream(evaluaterPath));
		ObjectInputStream ois_s = new ObjectInputStream(new FileInputStream(searcherPath));
    		this.evaluater = ((ASEvaluation)ois_e.readObject());
		this.searcher = ((ASSearch)ois_s.readObject());
		loadData(data);
	}

	private void loadData(String[][] data)
	{
		FastVector attributesVector = new FastVector();
		for (int i = 0; i < data[0].length; i++)
		{
			Attribute a = new Attribute("Attribute " + i);
			try
			{
				for (int j = 0; j < data.length; j++) {
					Double.parseDouble(data[j][i]);
				}
			}
			catch (NumberFormatException e)
			{
				FastVector f = new FastVector();
				for (int j = 0; j < data.length; j++) {
					if (!f.contains(data[j][i])) {
						f.addElement(data[j][i]);
					}
				}
				a = new Attribute("Attribute " + i, f);
			}
			attributesVector.addElement(a);
		}

		this.instances = new Instances("Instances", attributesVector, data.length);
		this.instances.setClassIndex(0);
    
		this.attributes = new Attribute[attributesVector.size()];

		for (int i = 0; i < attributesVector.size(); i++) {
			this.attributes[i] = ((Attribute)attributesVector.elementAt(i));
		}

    		for (int i = 0; i < data.length; i++) {
      			this.instances.add(toInstance(data[i]));
    		}
	}	

	private Instance toInstance(String[] data)
	{
		double[] vals = new double[data.length];
		for (int i = 0; i < data.length; i++) {
			vals[i] = getInternalValue(this.attributes[i], data[i]);
		}
		Instance in = new Instance(1.0D, vals);
		in.setDataset(this.instances);
		return in;
	}	


	public Instances applyFeatureSelection()
	{
		Instances newData = null;
		try
		{
			weka.filters.supervised.attribute.AttributeSelection filter = new  weka.filters.supervised.attribute.AttributeSelection();
			filter.setEvaluator(evaluater);
			filter.setSearch(searcher);
			filter.setInputFormat(instances);
	
			newData = Filter.useFilter(instances, filter);
		}
		catch(Exception e)
		{
			System.out.println("Exception thrown during the feature selection");
		}

		return newData;
	}

	private double getInternalValue(Attribute att, String val)
	{
		if (("$NaN".equals(val)) || ("null".equals(val))) {
			return -1.0;
		}
		if (att.isNumeric()) {
			return Double.parseDouble(val);
		}
		return att.indexOfValue(val);
	}
	
}

