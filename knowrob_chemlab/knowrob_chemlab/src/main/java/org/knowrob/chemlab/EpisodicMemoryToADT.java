/*
 * Copyright (C) 2016 Asil Kaan Bozcuoglu.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.knowrob.chemlab;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.AbstractMap;
import java.util.Set;
import java.util.StringTokenizer;
import java.lang.Float;


import org.w3c.dom.*;
import javax.xml.parsers.*;
import javax.vecmath.Vector3d;
import java.io.*;

import org.knowrob.knowrob_sim_games.MongoSimGames;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLLiteral;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyExpression;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyExpression;
import org.semanticweb.owlapi.model.PrefixManager;
import org.semanticweb.owlapi.model.AddImport;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.OWLDeclarationAxiom;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLImportsDeclaration;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.OWLReasonerConfiguration;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.HermiT.Reasoner;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasonerFactory;
import org.semanticweb.owlapi.reasoner.NodeSet;
import org.semanticweb.owlapi.util.SimpleIRIMapper;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.converters.Converter;
import com.thoughtworks.xstream.converters.MarshallingContext;
import com.thoughtworks.xstream.converters.UnmarshallingContext;
import com.thoughtworks.xstream.io.HierarchicalStreamReader;
import com.thoughtworks.xstream.io.HierarchicalStreamWriter;

/**
 *
 * @author asil@cs.uni-bremen.de
 *
 */
public class EpisodicMemoryToADT
{
	final String task_root_type = "http://knowrob.org/kb/knowrob.owl#RobotExperiment";
	final String sub_action_type = "http://knowrob.org/kb/knowrob.owl#subAction";
        final String start_time_type = "http://knowrob.org/kb/knowrob.owl#startTime";
	final String end_time_type = "http://knowrob.org/kb/knowrob.owl#endTime";
        final String translocation_type = "http://knowrob.org/kb/knowrob.owl#Translocation";
	final String trajectory_type = "http://knowrob.org/kb/knowrob.owl#Trajectory";

	OWLOntologyManager manager;
	OWLDataFactory factory;
	OWLReasoner reason;

	PrefixManager adtPM;
	PrefixManager adtExamplePM;

	OWLOntology ontology; 
	OWLOntology adtOntology; 
	Set<OWLClass> classes;
	XStream xStream;

	OWLNamedIndividual adt_of_interest;

	Map<String,String> dictionary;
	Map<String,String> isMongo;

	String owl_path;
	String dictionary_path;
	String output_path;
	String experiment_name;

	public EpisodicMemoryToADT(String owl_path, String dictionary_path, String output_path, String adt_prefix, String adt_example_prefix, String experiment_name)
	{
		xStream = new XStream();
        	xStream.registerConverter(new MapEntryConverter());
        	xStream.alias("root", Map.class);


		this.owl_path = owl_path;
		this.dictionary_path = dictionary_path;
		this.output_path = output_path;
		this.experiment_name = experiment_name;

		manager = OWLManager.createOWLOntologyManager();
		factory = manager.getOWLDataFactory();

		parseOWL();
		OWLReasonerFactory rFactory = new StructuralReasonerFactory();
		//reason = rFactory.createReasoner(ontology);

		OWLReasonerConfiguration config = new SimpleConfiguration(50000);
        	// Create a reasoner that will reason over our ontology and its imports
        	// closure. Pass in the configuration.
        	reason = rFactory.createReasoner(ontology, config);

		adtPM = new DefaultPrefixManager(adt_prefix);
		adtExamplePM = new DefaultPrefixManager(adt_example_prefix);

		initializeDictionaries();
	}


	public void initializeDictionaries()
	{	
		dictionary = initializeDictionary(false);
		isMongo = initializeDictionary(true);
	}

	public Map<String,String> initializeDictionary(boolean mongoCheck)
	{	
		String xmlContent;
		if(mongoCheck)
		 	xmlContent =  writeXMLToString(dictionary_path + "/mongo.xml");
		else
			xmlContent =  writeXMLToString(dictionary_path + "/dictionary.xml");

		Object o = xStream.fromXML(xmlContent);
		
		return (Map<String,String>) o;
	}	

	public boolean initializeADTOntology(String actionClass, OWLNamedIndividual ind)
	{
		try
		{
			actionClass =  getIndividualTagName(actionClass, "#");
			OWLClass clsADT = factory.getOWLClass(":#" + actionClass.replaceAll("Action", "") + "ADT", adtPM);
			adtOntology = manager.createOntology(IRI.create(adtExamplePM.getDefaultPrefix()));
		
			OWLDeclarationAxiom declarationAxiom = factory.getOWLDeclarationAxiom(clsADT);
			manager.addAxiom(ontology, declarationAxiom);	

			String indName = ind.getIRI().toString();
			indName =  getIndividualTagName(indName, "#");

			adt_of_interest = factory.getOWLNamedIndividual(":#ActionDataTable_" + indName, adtExamplePM);
			OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom(clsADT, adt_of_interest);	

			manager.addAxiom(adtOntology, classAssertion);


			OWLImportsDeclaration imprt = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "https://raw.githubusercontent.com/knowrob/knowrob_addons/master/knowrob_cram/owl/knowrob_cram.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/cram_log.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/master/knowrob_cram/owl/knowrob_cram.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt ) );

			OWLImportsDeclaration imprt2 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "https://raw.githubusercontent.com/knowrob/knowrob/master/knowrob_common/owl/knowrob.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/knowrob.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob/master/knowrob_common/owl/knowrob.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt2 ) );

			OWLImportsDeclaration imprt3 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/acat.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/acat.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/acat.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt3 ) );

		}
		catch(Exception e)
		{
			e.printStackTrace();
		}

		return true;
	}

	public String getIndividualTagName(String ind, String delim)
	{
		StringTokenizer st = new StringTokenizer(ind, delim);
		st.nextToken();
		return st.nextToken();
	}

	public boolean parseOWL()
	{
		try
		{	
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_common/owl/knowrob.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob/master/knowrob_common/owl/knowrob.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_cram/owl/knowrob_cram.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/master/knowrob_cram/owl/knowrob_cram.owl")));
			File file = new File(owl_path + "/log.owl");
	      		ontology = manager.loadOntologyFromOntologyDocument(file);
			System.out.println("Loaded ontology: " + file);

			classes = ontology.getClassesInSignature(true);	
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}	
			return true;
	}

	public boolean generateADT(String actionClass)
	{
		IRI actionClassIRI = IRI.create(actionClass);
		NodeSet<OWLNamedIndividual> setOfInstances = null;

		for(OWLClass currentClass : classes)
		{
			if(currentClass.getIRI().toString().equals(actionClass))
			{

				setOfInstances = reason.getInstances(currentClass, true);
				break;
			}

		}

		int count = 0;

		if(setOfInstances != null)
		{
			for(OWLNamedIndividual instance : setOfInstances.getFlattened())
			{
				generateADTFromIndividual(actionClass, instance, count);
				count++;
			}
		}

		return true;
		/*OWLClass classToBeImported = null; 
	
		OWLClass[] classesArray = new OWLClass[classes.size()];
		classes.toArray(classesArray);
	
		for(int x = 0; x < classesArray.length; x++)
		{
			OWLClass currentClass = classesArray[i];

			String uri = currentClass.toStringID();
			String subString = uri.substring(uri.length , uri.length() - 1)

			if(subString.equals(actionClass))
			{
				classToBeImported = currentClass;
				break;
			}
		}*/

		

	}

	public boolean generateADTFromIndividual(String adtType, OWLIndividual ind, int count)
	{
		initializeADTOntology(adtType, ind.asOWLNamedIndividual());
		Set<String> keySet = dictionary.keySet();
		String[] keySetArray = new String[keySet.size()];
		keySet.toArray(keySetArray);
	
		for(int x = 0; x < keySetArray.length; x++)
		{
			String owlField = dictionary.get(keySetArray[x]);
			
			
			Set<OWLObjectPropertyExpression> objectProperties = getRelatedSubObjectProperties(ind);
			Set<OWLDataPropertyExpression> dataProperties = getRelatedSubDataProperties(ind);


			for (OWLObjectPropertyExpression property : objectProperties) {
				if(owlField.equals(property.asOWLObjectProperty().getIRI().toString()))
				{	
						
					Set<OWLIndividual> values =  ind.getObjectPropertyValues(property, ontology);
					
					if(values != null && values.size() > 0)
					{
						
						OWLIndividual[] valuesArray = new OWLIndividual[values.size()];
						values.toArray(valuesArray);

						OWLNamedIndividual valueFirst = valuesArray[0].asOWLNamedIndividual();


						Set<OWLClassExpression> classes = valueFirst.getTypes(ontology);
						OWLClassExpression[] classesArray = new OWLClassExpression[classes.size()];
						classes.toArray(classesArray);

						OWLDeclarationAxiom declarationAxiom = factory.getOWLDeclarationAxiom(classesArray[0].asOWLClass());
						manager.addAxiom(adtOntology, declarationAxiom);	
						OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom(classesArray[0].asOWLClass(), valueFirst);	
						AddAxiom addAxiomChange1 = new AddAxiom(adtOntology, classAssertion);
						manager.applyChange(addAxiomChange1);

						String adtClassIRI = isMongo.get(keySetArray[x]);
						OWLClass adtClass = factory.getOWLClass(IRI.create(adtClassIRI));
						declarationAxiom = factory.getOWLDeclarationAxiom(adtClass);
						manager.addAxiom(adtOntology, declarationAxiom);	
						classAssertion = factory.getOWLClassAssertionAxiom(adtClass, valueFirst);	
						addAxiomChange1 = new AddAxiom(adtOntology, classAssertion);
						manager.applyChange(addAxiomChange1);


						OWLObjectProperty interpretedProperty = factory.getOWLObjectProperty(IRI.create(keySetArray[x].replaceAll("http:", "http://").
											replaceAll("_", "/").replaceAll("-", "#")));

						makeObjectPropertyAssertion(adt_of_interest, valueFirst, interpretedProperty);
					}	
					
													
					break;
				}
			}

			for (OWLDataPropertyExpression property : dataProperties) {
				if(owlField.equals(property.asOWLDataProperty().getIRI().toString()))
				{		
					Set<OWLLiteral> values =  ind.getDataPropertyValues(property, ontology);

					OWLLiteral[] valuesArray = new OWLLiteral[values.size()];
					values.toArray(valuesArray);

					OWLLiteral valueFirst = valuesArray[0];
					OWLDataPropertyExpression interpretedProperty = factory.getOWLDataProperty(IRI.create(keySetArray[x].replaceAll("http:", "http://").
											replaceAll("_", "/").replaceAll("-", "#")));

						
					OWLDataPropertyAssertionAxiom assertion = factory.getOWLDataPropertyAssertionAxiom
												(interpretedProperty, adt_of_interest, valueFirst);
					AddAxiom addAxiomChange = new AddAxiom(adtOntology, assertion);
					manager.applyChange(addAxiomChange);
					break;
				}
			}
		}

		generateActionChuck(ind);		
		File ontologyFile = new File(output_path + "/adt" + count + ".owl");

		RDFXMLOntologyFormat rdfxmlformat = new RDFXMLOntologyFormat();
		try
		{
			manager.saveOntology(adtOntology, rdfxmlformat, IRI.create(ontologyFile));

		}
		catch(Exception e)
		{
			e.printStackTrace();
		}

		return true;
		
	}

	private Set<OWLObjectPropertyExpression> getRelatedSubObjectProperties(OWLIndividual individual) {
		Map<OWLObjectPropertyExpression,java.util.Set<OWLIndividual>> mapping = individual.getObjectPropertyValues(ontology);		
		return mapping.keySet();
	}

	private Set<OWLDataPropertyExpression> getRelatedSubDataProperties(OWLIndividual individual) {
		Map<OWLDataPropertyExpression,java.util.Set<OWLLiteral>> mapping = individual.getDataPropertyValues(ontology);		
		return mapping.keySet();
	}


	public boolean generateActionChuck(OWLIndividual ind)
	{

		OWLClass rootClass = factory.getOWLClass(IRI.create(task_root_type));
		NodeSet<OWLNamedIndividual> rootInstances = reason.getInstances(rootClass, true);

		for(OWLNamedIndividual root : rootInstances.getFlattened())
		{
			
			Set<OWLObjectPropertyExpression> objectProperties = getRelatedSubObjectProperties(root);
			
			for(OWLObjectPropertyExpression propExpr : objectProperties)
			{
				OWLObjectProperty prop = propExpr.asOWLObjectProperty();
				if(prop.getIRI().toString().equals(sub_action_type))
				{
					Set<OWLIndividual> values =  root.getObjectPropertyValues(prop, ontology);
					
					if(values != null && values.size() > 0)
					{
						for(OWLIndividual value : values)
						{
							if(intervalIncludes(ind, value))
							{
								OWLClass chunkClass = factory.getOWLClass(":#ActionChunk", adtPM);
								OWLNamedIndividual valueNew = factory.getOWLNamedIndividual
											(":#ActionChunk_" + value.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
								OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom(chunkClass, valueNew);
								manager.addAxiom(adtOntology, classAssertion);

								OWLObjectProperty chunkProperty = factory.getOWLObjectProperty
										(":#adtAction", adtPM);

								makeObjectPropertyAssertion(adt_of_interest, valueNew, chunkProperty);

								OWLNamedIndividual start = timeInstanceInd(value, true);
								OWLNamedIndividual end = timeInstanceInd(value, false);

								OWLObjectProperty startProperty = factory.getOWLObjectProperty
										(IRI.create(start_time_type));

								OWLObjectProperty endProperty = factory.getOWLObjectProperty
										(IRI.create(end_time_type));

								makeObjectPropertyAssertion(valueNew, start, startProperty);

								makeObjectPropertyAssertion(valueNew, end, endProperty);
					
								createTrajectoryForActionChunk(valueNew, 
											timeInstance(value, true), timeInstance(value, false), "PR2LGripper");

							}

						}

					}

					break;
				}

			}			
		
		}
		
		return true;
	}

	public void makeObjectPropertyAssertion(OWLIndividual ind1, OWLIndividual ind2, OWLObjectProperty property)
	{
		OWLObjectPropertyAssertionAxiom assertion = factory.getOWLObjectPropertyAssertionAxiom(property, ind1, ind2);
		AddAxiom addAxiomChange = new AddAxiom(adtOntology, assertion);
		manager.applyChange(addAxiomChange);

	}

	public boolean intervalIncludes(OWLIndividual ind1, OWLIndividual ind2)
	{
		float start1 = timeInstance(ind1, true);
		float end1 = timeInstance(ind1, false);
		float start2 = timeInstance(ind2, true);
		float end2 = timeInstance(ind2, false);

		return ((start1 <= start2) && (end1 >= end2));
	}

	public float timeInstance(OWLIndividual ind, boolean isStart)
	{
		OWLNamedIndividual timeInst = timeInstanceInd(ind, isStart);
		
		if (timeInst == null)
			return (float)0.0;

		String iri = timeInst.getIRI().toString();
		StringTokenizer st = new StringTokenizer(iri, "_");
		st.nextToken();
		st.nextToken();
		return Float.parseFloat(st.nextToken());
	}


	public OWLNamedIndividual timeInstanceInd(OWLIndividual ind, boolean isStart)
	{
		String property;
		if(isStart) property = start_time_type;
		else property = end_time_type;
		

		Set<OWLObjectPropertyExpression> objectProperties = getRelatedSubObjectProperties(ind);
			
		for(OWLObjectPropertyExpression propExpr : objectProperties)
		{
			OWLObjectProperty prop = propExpr.asOWLObjectProperty();

			if(prop.getIRI().toString().equals(property))
			{
				Set<OWLIndividual> values =  ind.getObjectPropertyValues(prop, ontology);
				OWLIndividual[] valuesArray = new OWLIndividual[values.size()];
				values.toArray(valuesArray);

				OWLNamedIndividual valueFirst = valuesArray[0].asOWLNamedIndividual();
				return valueFirst;
			}

		}
		return null;
	}

	public String writeXMLToString(String path)
	{
		String everything = "";
		
		try
		{
			BufferedReader br = new BufferedReader(new FileReader(path));
			try {
	    			StringBuilder sb = new StringBuilder();
	    			String line = br.readLine();

	    			while (line != null) {
					sb.append(line);
					sb.append(System.lineSeparator());
					line = br.readLine();
	    			}
	    			everything = sb.toString();
			} finally {
	    			br.close();
			}
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		

		return everything;

	}


	public boolean createTrajectoryForActionChunk(OWLNamedIndividual chunk, float start, float end, String model)
	{
		if(start < end)
		{
			MongoSimGames mongosimgames = new MongoSimGames();
			mongosimgames.SetDatabase(experiment_name);
			mongosimgames.SetCollection(experiment_name + "_raw");

			ArrayList<Vector3d> trajPoints =  mongosimgames.ViewModelTrajectory((double) start, (double) end, model, null);

			OWLClass translocationClass = factory.getOWLClass(IRI.create(translocation_type));
			OWLClass trajectoryClass = factory.getOWLClass(IRI.create(trajectory_type));
			OWLNamedIndividual trajectoryInstance = factory.getOWLNamedIndividual(":#Trajectory_" + chunk.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
			OWLClassAssertionAxiom translocationClassAssertion = factory.getOWLClassAssertionAxiom(translocationClass, trajectoryInstance);
			manager.addAxiom(adtOntology, translocationClassAssertion);
			OWLClassAssertionAxiom trajectoryClassAssertion = factory.getOWLClassAssertionAxiom(trajectoryClass, trajectoryInstance);
			manager.addAxiom(adtOntology, trajectoryClassAssertion);

			String trajectorySamples = "";

			for(Vector3d point: trajPoints)
			{
				trajectorySamples += point.x + " " + point.y + " " + point.z + "";			 
			}

			OWLLiteral trajectoryLiteral = factory.	getOWLLiteral(trajectorySamples);
			OWLDataPropertyExpression trajectoryProperty = factory.getOWLDataProperty(":#poses", adtPM);
			OWLDataPropertyAssertionAxiom assertion = factory.getOWLDataPropertyAssertionAxiom(trajectoryProperty, trajectoryInstance, trajectoryLiteral);
			AddAxiom addAxiomChange = new AddAxiom(adtOntology, assertion);
			manager.applyChange(addAxiomChange);

			OWLObjectProperty chunkProperty = factory.getOWLObjectProperty(":#trajectoryDuring", adtPM);
			OWLObjectPropertyAssertionAxiom chunkAssertion = factory.getOWLObjectPropertyAssertionAxiom(chunkProperty, trajectoryInstance, chunk);
			addAxiomChange = new AddAxiom(adtOntology, chunkAssertion);
			manager.applyChange(addAxiomChange);
		}
		else
			return false;

		return true;

	}


	public static class MapEntryConverter implements Converter {

		public boolean canConvert(Class clazz) {
		    return AbstractMap.class.isAssignableFrom(clazz);
		}

		public void marshal(Object value, HierarchicalStreamWriter writer, MarshallingContext context) {

		    AbstractMap map = (AbstractMap) value;
		    for (Object obj : map.entrySet()) {
		        Map.Entry entry = (Map.Entry) obj;
		        writer.startNode(entry.getKey().toString());
		        Object val = entry.getValue();
		        if ( null != val ) {
		            writer.setValue(val.toString());
		        }
		        writer.endNode();
		    }

		}

		public Object unmarshal(HierarchicalStreamReader reader, UnmarshallingContext context) {

		    Map<String, String> map = new HashMap<String, String>();

		    while(reader.hasMoreChildren()) {
		        reader.moveDown();

		        String key = reader.getNodeName(); // nodeName aka element's name
		        String value = reader.getValue();
		        map.put(key, value);

		        reader.moveUp();
		    }

		    return map;
		}

    }
	

}
 
