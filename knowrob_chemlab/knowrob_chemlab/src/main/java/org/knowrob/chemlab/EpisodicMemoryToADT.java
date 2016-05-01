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
import org.semanticweb.owlapi.model.OWLDatatype;
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
	final String trajectory_type = "http://knowrob.org/kb/knowrob.owl#Trajectory";
	final String translation_type = "http://knowrob.org/kb/knowrob.owl#translation";
	final String quaternion_type = "http://knowrob.org/kb/knowrob.owl#quaternion";
	final String prac_type = "http://knowrob.org/kb/acat.owl#PracAdt";
	final String supporting_type = "http://knowrob.org/kb/acat.owl#supportingPlane";
	final String supported_type = "http://knowrob.org/kb/acat.owl#supportedObject";
	final String supporting_adt_type = "http://knowrob.org/kb/acat-adt.owl#adtChunkSupport";
	final String supported_adt_type = "http://knowrob.org/kb/knowrob.owl#supportedObject";
	final String actioncore_type = "http://knowrob.org/kb/acat-adt.owl#actioncore";

	OWLOntologyManager manager;
	OWLDataFactory factory;
	OWLReasoner reason;
	OWLReasoner reason_map;

	PrefixManager adtPM;
	PrefixManager adtExamplePM;

	OWLOntology ontology; 
	OWLOntology adtOntology;
	OWLOntology mapOntology; 
	Set<OWLClass> classes;
	XStream xStream;

	OWLNamedIndividual adt_of_interest;

	Map<String,String> dictionary;
	Map<String,String> isMongo;
	Map<String,String> prac;

	String owl_path;
	String log_path;
	String dictionary_path;
	String output_path;
	String experiment_name;

	public EpisodicMemoryToADT(String owl_path, String log_path, String dictionary_path, String output_path, String adt_prefix, String adt_example_prefix, String experiment_name)
	{
		xStream = new XStream();
        	xStream.registerConverter(new MapEntryConverter());
        	xStream.alias("root", Map.class);


		this.owl_path = owl_path;
		this.log_path = log_path;
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
		reason_map = rFactory.createReasoner(mapOntology, config);

		adtPM = new DefaultPrefixManager(adt_prefix);
		adtExamplePM = new DefaultPrefixManager(adt_example_prefix);

		initializeDictionaries();
	}


	public void initializeDictionaries()
	{	
		isMongo = initializeDictionary(0);
		dictionary = initializeDictionary(1);
		prac = initializeDictionary(2);
	}

	public Map<String,String> initializeDictionary(int mode)
	{	
		String xmlContent;
		if(mode == 0)
		 	xmlContent =  writeXMLToString(dictionary_path + "/mongo.xml");
		else if(mode == 1)
			xmlContent =  writeXMLToString(dictionary_path + "/dictionary.xml");
		else 
			xmlContent =  writeXMLToString(dictionary_path + "/prac.xml");

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

			OWLLiteral lit = factory.getOWLLiteral(actionClass.replaceAll("Action", ""));

			OWLDataProperty actionCoreProperty = factory.getOWLDataProperty(IRI.create(actioncore_type));

			makeDataPropertyAssertion(adt_of_interest, lit, actionCoreProperty);

			OWLImportsDeclaration imprt = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "package://knowrob_cram/owl/knowrob-cram.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/cram_log.owl"), 
						IRI.create("package://knowrob_cram/owl/knowrob-cram.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt ) );

			OWLImportsDeclaration imprt2 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "package://knowrob_common/owl/knowrob.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/knowrob.owl"), 
						IRI.create("package://knowrob_common/owl/knowrob.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt2 ) );

			OWLImportsDeclaration imprt3 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "package://knowrob_chemlab/owl/acat-adt.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/acat-adt.owl"), 
						IRI.create("package://knowrob_chemlab/owl/acat-adt.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt3 ) );


			OWLImportsDeclaration imprt4 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "package://knowrob_chemlab/owl/chemlab.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/chemlab.owl"), 
						IRI.create("package://knowrob_chemlab/owl/chemlab.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/chemlab-actions.owl"), 
						IRI.create("package://knowrob_chemlab/owl/chemlab-actions.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/chemlab-substances.owl"), 
						IRI.create("package://knowrob_chemlab/owl/chemlab-substances.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/chemlab-objects.owl"), 
						IRI.create("package://knowrob_chemlab/owl/chemlab-objects.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt4 ) );


			OWLImportsDeclaration imprt5 = manager.getOWLDataFactory().getOWLImportsDeclaration
							( IRI.create( "package://knowrob_chemlab/owl/chemlab-map_review-2016.owl" ) );
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("http://knowrob.org/kb/chemlab-map_review-2016.owl"), 
						IRI.create("package://knowrob_chemlab/owl/chemlab-map_review-2016.owl")));

    			manager.applyChange( new AddImport( adtOntology, imprt5 ) );


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
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/acat-adt.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/acat-adt.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/chemlab.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/chemlab.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/chemlab-actions.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/chemlab-actions.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/chemlab-objects.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/chemlab-objects.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/chemlab-substances.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/chemlab-substances.owl")));
			manager.addIRIMapper(new SimpleIRIMapper(IRI.create("package://knowrob_chemlab/owl/chemlab-map_review-2016.owl"), 
						IRI.create("https://raw.githubusercontent.com/knowrob/knowrob_addons/acat/knowrob_chemlab/owl/chemlab-map_review-2016.owl")));

			File file = new File(log_path);
	      		ontology = manager.loadOntologyFromOntologyDocument(file);
			System.out.println("Loaded ontology: " + file);

			File mapFile = new File(owl_path + "/chemlab-map_review-2016.owl");
	      		mapOntology = manager.loadOntologyFromOntologyDocument(mapFile);
			System.out.println("Loaded ontology: " + mapFile);

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
				break;
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
	

		OWLObjectProperty taskProperty = factory.getOWLObjectProperty(":#adtAction", adtPM);
		makeObjectPropertyAssertion(adt_of_interest, ind, taskProperty);

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

						if(!owlField.contains("Time"))
						{
							
							OWLNamedIndividual newValue = factory.getOWLNamedIndividual(":#ActionObjectDescription_" 
									+ valueFirst.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
							addSemanticMapFeatures(newValue, valueFirst);
							valueFirst = newValue;
						}

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


						OWLObjectProperty interpretedProperty = factory.getOWLObjectProperty(IRI.create(keySetArray[x].
											replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://")));

						

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
					OWLDataProperty interpretedProperty = factory.getOWLDataProperty(IRI.create(keySetArray[x].
											replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://")));

					makeDataPropertyAssertion(adt_of_interest, valueFirst, interpretedProperty);
					break;
				}
			}
			includePRACData(adt_of_interest);
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
		return getRelatedSubObjectProperties(individual, ontology);
	}

	private Set<OWLDataPropertyExpression> getRelatedSubDataProperties(OWLIndividual individual) {
		return getRelatedSubDataProperties(individual, ontology);
	}

	private Set<OWLObjectPropertyExpression> getRelatedSubObjectProperties(OWLIndividual individual, OWLOntology o) {
		Map<OWLObjectPropertyExpression,java.util.Set<OWLIndividual>> mapping = individual.getObjectPropertyValues(o);		
		return mapping.keySet();
	}

	private Set<OWLDataPropertyExpression> getRelatedSubDataProperties(OWLIndividual individual, OWLOntology o) {
		Map<OWLDataPropertyExpression,java.util.Set<OWLLiteral>> mapping = individual.getDataPropertyValues(o);		
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

								OWLObjectProperty chunkProperty = factory.getOWLObjectProperty(IRI.create(sub_action_type));

								makeObjectPropertyAssertion(adt_of_interest, valueNew, chunkProperty);

								OWLNamedIndividual start = timeInstanceInd(value, true);
								OWLNamedIndividual end = timeInstanceInd(value, false);

								OWLClass startClass = factory.getOWLClass(IRI.create("http://knowrob.org/kb/knowrob.owl#TimePoint"));
								OWLClass endClass = factory.getOWLClass(IRI.create("http://knowrob.org/kb/knowrob.owl#TimePoint")); 
								OWLClassAssertionAxiom startAssertion = factory.getOWLClassAssertionAxiom(startClass, start);
								manager.addAxiom(adtOntology, startAssertion);
								OWLClassAssertionAxiom endAssertion = factory.getOWLClassAssertionAxiom(endClass, end);
								manager.addAxiom(adtOntology, endAssertion);


								OWLObjectProperty startProperty = factory.getOWLObjectProperty
										(IRI.create(start_time_type));

								OWLObjectProperty endProperty = factory.getOWLObjectProperty
										(IRI.create(end_time_type));

								makeObjectPropertyAssertion(valueNew, start, startProperty);

								makeObjectPropertyAssertion(valueNew, end, endProperty);
					
								createTrajectoryForActionChunk(valueNew, 
											timeInstance(value, true), timeInstance(value, false), "PR2LGripper");

								OWLNamedIndividual supported = instanceOfProperty(value, supported_type);
								OWLNamedIndividual supporting = instanceOfProperty(value, supporting_type);
								OWLNamedIndividual supporting_new = null;

								if(supporting != null)
								{
									supporting_new = factory.getOWLNamedIndividual(":#SupportingPlane_" 
									+ supporting.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);

									OWLObjectProperty supportingProperty = factory.getOWLObjectProperty(IRI.create(supporting_adt_type));
									makeObjectPropertyAssertion(valueNew, supporting_new, supportingProperty);

									String data = normalValuesOfPlane(supporting);
									OWLLiteral lit = factory.getOWLLiteral(data);
									OWLDataProperty normalProperty = factory.getOWLDataProperty(IRI.create(translation_type));
									makeDataPropertyAssertion(supporting_new, lit, normalProperty);

									OWLLiteral lit_q = factory.getOWLLiteral("1 0 0 0");
									OWLDataProperty qProperty = factory.getOWLDataProperty(IRI.create(quaternion_type));
									makeDataPropertyAssertion(supporting_new, lit_q, qProperty);	
								}
								if(supported != null)
								{
									OWLNamedIndividual supported_new = factory.getOWLNamedIndividual(":#ActionObjectDescription_" 
									+ supported.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);

									OWLObjectProperty supportedProperty = factory.getOWLObjectProperty(IRI.create(supported_adt_type));
									makeObjectPropertyAssertion(supporting_new, supported_new, supportedProperty);
								}
								

							}

						}

					}

					break;
				}

			}			
		
		}
		
		return true;
	}

	public boolean includePRACData(OWLIndividual ind)
	{
		OWLClass pracClass = factory.getOWLClass(IRI.create(prac_type));
		NodeSet<OWLNamedIndividual> pracInstances = reason.getInstances(pracClass, true);

		String amount = "";
		String amount_field = "";
		OWLDatatype unit = null;

		for(OWLNamedIndividual pracInd : pracInstances.getFlattened())
		{
			Set<OWLObjectPropertyExpression> objectProperties = getRelatedSubObjectProperties(pracInd);
			

			Set<String> keySet = prac.keySet();
			String[] keySetArray = new String[keySet.size()];
			keySet.toArray(keySetArray);
			for(OWLObjectPropertyExpression propExpr : objectProperties)
			{
				OWLObjectProperty prop = propExpr.asOWLObjectProperty();
					
				for(int x = 0; x < keySetArray.length; x++)
				{
					String owlField = prac.get(keySetArray[x]);
					if(prop.getIRI().toString().equals(owlField))
					{
						Set<OWLIndividual> values =  pracInd.getObjectPropertyValues(prop, ontology);
						
					
						if(values != null && values.size() > 0)
						{
						
							OWLIndividual[] valuesArray = new OWLIndividual[values.size()];
							values.toArray(valuesArray);

							OWLNamedIndividual valueFirst = valuesArray[0].asOWLNamedIndividual();


							String adtClassIRI = isMongo.get(keySetArray[x]);

							if(adtClassIRI.equals("literal"))
							{
								OWLLiteral lit = objectToLiteral(valueFirst);

								OWLDataProperty interpretedProperty = factory.getOWLDataProperty(IRI.create(keySetArray[x].
											replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://")));

								makeDataPropertyAssertion(ind.asOWLNamedIndividual(), lit, interpretedProperty);
							}
							else if(adtClassIRI.equals("literal1"))
							{
								OWLLiteral lit = objectToLiteral(valueFirst);
								unit = factory.getOWLDatatype(IRI.create("http://knowrob.org/kb/chemlab-substances.owl#" + lit.getLiteral()));
							}
							else if(adtClassIRI.equals("literal2"))
							{
								OWLLiteral lit = objectToLiteral(valueFirst);
								amount = lit.getLiteral();
								amount_field = keySetArray[x].replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://");
							}
							else
							{
								Set<OWLClassExpression> classes = valueFirst.getTypes(ontology);
								OWLClassExpression[] classesArray = new OWLClassExpression[classes.size()];
								classes.toArray(classesArray);

								OWLDeclarationAxiom declarationAxiom = factory.getOWLDeclarationAxiom(classesArray[0].asOWLClass());
								manager.addAxiom(adtOntology, declarationAxiom);	
								OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom
													(classesArray[0].asOWLClass(), valueFirst);	
								AddAxiom addAxiomChange1 = new AddAxiom(adtOntology, classAssertion);
								manager.applyChange(addAxiomChange1);

								OWLClass adtClass = factory.getOWLClass(IRI.create(adtClassIRI));
								declarationAxiom = factory.getOWLDeclarationAxiom(adtClass);
								manager.addAxiom(adtOntology, declarationAxiom);

								valueFirst = factory.getOWLNamedIndividual(":#ActionObjectDescription_" 
									+ valueFirst.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
	
								classAssertion = factory.getOWLClassAssertionAxiom(adtClass, valueFirst);	
								addAxiomChange1 = new AddAxiom(adtOntology, classAssertion);
								manager.applyChange(addAxiomChange1);


								OWLObjectProperty interpretedProperty = factory.getOWLObjectProperty(IRI.create(keySetArray[x].
													replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://")));

								makeObjectPropertyAssertion(ind, valueFirst, interpretedProperty);
							}
						}
					}
				}
			}
			
			Set<OWLDataPropertyExpression> dataProperties = getRelatedSubDataProperties(pracInd);
			for(OWLDataPropertyExpression propExpr : dataProperties)
			{
				OWLDataProperty prop = propExpr.asOWLDataProperty();
				for(int x = 0; x < keySetArray.length; x++)
				{
					String owlField = prac.get(keySetArray[x]);
					System.out.println(owlField + "--" + prop.getIRI().toString());
					if(prop.getIRI().toString().equals(owlField))
					{
						Set<OWLLiteral> values =  pracInd.getDataPropertyValues(prop, ontology);
										
						if(values != null && values.size() > 0)
						{
							OWLLiteral[] valuesArray = new OWLLiteral[values.size()];
							values.toArray(valuesArray);

							OWLLiteral valueFirst = valuesArray[0];
							OWLDataProperty interpretedProperty = factory.getOWLDataProperty(IRI.create(keySetArray[x].
													replaceAll("_", "/").replaceAll(":", "#").replaceAll("http", "http://")));


							makeDataPropertyAssertion(ind.asOWLNamedIndividual(), valueFirst, interpretedProperty);

						}

					}
				}

			}


		}
		OWLLiteral amount1 = factory.getOWLLiteral(amount, unit);
		OWLDataProperty amountProperty = factory.getOWLDataProperty(IRI.create(amount_field));
		makeDataPropertyAssertion(ind.asOWLNamedIndividual(), amount1, amountProperty);
		return true;
	}

	public void makeObjectPropertyAssertion(OWLIndividual ind1, OWLIndividual ind2, OWLObjectProperty property)
	{
		OWLObjectPropertyAssertionAxiom assertion = factory.getOWLObjectPropertyAssertionAxiom(property, ind1, ind2);
		AddAxiom addAxiomChange = new AddAxiom(adtOntology, assertion);
		manager.applyChange(addAxiomChange);

	}


	public void makeDataPropertyAssertion(OWLNamedIndividual ind, OWLLiteral lit, OWLDataProperty property)
	{
		OWLDataPropertyAssertionAxiom assertion = factory.getOWLDataPropertyAssertionAxiom(property, ind, lit);
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
		
		return instanceOfProperty(ind, property);
	}

	public OWLNamedIndividual instanceOfProperty(OWLIndividual ind, String property)
	{
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

			OWLClass trajectoryClass = factory.getOWLClass(IRI.create(trajectory_type));
			OWLNamedIndividual trajectoryInstance = factory.getOWLNamedIndividual(":#Trajectory_" + chunk.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
			OWLClassAssertionAxiom trajectoryClassAssertion = factory.getOWLClassAssertionAxiom(trajectoryClass, trajectoryInstance);
			manager.addAxiom(adtOntology, trajectoryClassAssertion);

			String trajectorySamples = "";

			for(Vector3d point: trajPoints)
			{
				trajectorySamples += point.x + " " + point.y + " " + point.z + " ";			 
			}

			OWLLiteral trajectoryLiteral = factory.	getOWLLiteral(trajectorySamples);
			OWLDataPropertyExpression trajectoryProperty = factory.getOWLDataProperty(":#poses", adtPM);
			OWLDataPropertyAssertionAxiom assertion = factory.getOWLDataPropertyAssertionAxiom(trajectoryProperty, trajectoryInstance, trajectoryLiteral);
			AddAxiom addAxiomChange = new AddAxiom(adtOntology, assertion);
			manager.applyChange(addAxiomChange);

			OWLObjectProperty chunkProperty = factory.getOWLObjectProperty(":#adtChunkTrajectory", adtPM);
			OWLObjectPropertyAssertionAxiom chunkAssertion = factory.getOWLObjectPropertyAssertionAxiom(chunkProperty, chunk, trajectoryInstance);
			addAxiomChange = new AddAxiom(adtOntology, chunkAssertion);
			manager.applyChange(addAxiomChange);
		}
		else
			return false;

		return true;

	}


	public OWLLiteral objectToLiteral(OWLNamedIndividual ind)
	{
		return factory.getOWLLiteral(objectToString(ind));
	}

	public String objectToString(OWLNamedIndividual ind)
	{
		String iri = getIndividualTagName(ind.getIRI().toString(), "#");

		StringTokenizer st = new StringTokenizer(iri, "_");
		String literal = st.nextToken().replaceAll("%20", " ");

		return literal;
	}

	public boolean addSemanticMapFeatures(OWLNamedIndividual adtInd, OWLNamedIndividual logInd)
	{
		Set<OWLClassExpression> classesSem = logInd.getTypes(ontology);
		OWLClassExpression[] classesSemArray = new OWLClassExpression[classesSem.size()];
		classesSem.toArray(classesSemArray);
		String iri_postfix = getIndividualTagName(classesSemArray[0].asOWLClass().getIRI().toString(), "#");

		OWLClass iri_object_class = factory.getOWLClass(IRI.create("http://knowrob.org/kb/chemlab-objects.owl#" + iri_postfix));
		OWLClass iri_substance_class = factory.getOWLClass(IRI.create("http://knowrob.org/kb/chemlab-substances.owl#" + iri_postfix));


		NodeSet<OWLNamedIndividual> objectInstances = reason_map.getInstances(iri_object_class, true);
		NodeSet<OWLNamedIndividual> substanceInstances = reason_map.getInstances(iri_substance_class, true);

		addIndividualFeaturesMap(objectInstances, adtInd);
		addIndividualFeaturesMap(substanceInstances, adtInd);
		return true;
	}

	public boolean addIndividualFeaturesMap(NodeSet<OWLNamedIndividual> objectInstances, OWLNamedIndividual adtInd)
	{
		for(OWLNamedIndividual ind : objectInstances.getFlattened())
		{
			Set<OWLObjectPropertyExpression> objectProperties = getRelatedSubObjectProperties(ind, mapOntology);
			
			for(OWLObjectPropertyExpression propExpr : objectProperties)
			{

				OWLObjectProperty prop = propExpr.asOWLObjectProperty();

				Set<OWLIndividual> values =  ind.getObjectPropertyValues(prop, mapOntology);
				OWLIndividual[] valuesArray = new OWLIndividual[values.size()];
				values.toArray(valuesArray);

				OWLNamedIndividual valueFirst = valuesArray[0].asOWLNamedIndividual();
				makeObjectPropertyAssertion(adtInd, valueFirst, prop);
			}

			Set<OWLDataPropertyExpression> dataProperties = getRelatedSubDataProperties(ind, mapOntology);
			
			for(OWLDataPropertyExpression propExpr : dataProperties)
			{
				OWLDataProperty prop = propExpr.asOWLDataProperty();

				Set<OWLLiteral> values =  ind.getDataPropertyValues(prop, mapOntology);
				OWLLiteral[] valuesArray = new OWLLiteral[values.size()];
				values.toArray(valuesArray);

				OWLLiteral valueFirst = valuesArray[0];
				makeDataPropertyAssertion(adtInd, valueFirst, prop);
			}
				

			break;
		}
		return true;
	}


	public String normalValuesOfPlane(OWLNamedIndividual ind)
	{
		String iri = getIndividualTagName(ind.getIRI().toString(), "#");

		StringTokenizer st = new StringTokenizer(iri, "_");
		String literal = st.nextToken();
		literal = literal.concat(" ");
		literal = literal.concat(st.nextToken());
		literal = literal.concat(" ");
		literal = literal.concat(st.nextToken());
		return literal;
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
 

