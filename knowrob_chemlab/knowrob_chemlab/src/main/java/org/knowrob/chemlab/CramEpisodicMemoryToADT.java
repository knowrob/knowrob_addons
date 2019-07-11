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
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;
import java.io.*;
import org.knowrob.tfmemory.StampedTransform;


import org.knowrob.knowrob_sim_games.MongoSimGames;
import org.knowrob.interfaces.mongo.MongoDBInterface;

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
public class CramEpisodicMemoryToADT extends EpisodicMemoryToADT
{
	MongoDBInterface mongo;


	public CramEpisodicMemoryToADT
		(String owl_path, String log_path, String dictionary_path, String output_path, String adt_prefix, String adt_example_prefix, String experiment_name)
	{
		super(owl_path, log_path, dictionary_path, output_path, adt_prefix, adt_example_prefix, experiment_name);
		this.initializeDictionaries();

		mongo = new MongoDBInterface();
		mongo.setDatabase(experiment_name);
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
		 	xmlContent =  writeXMLToString(dictionary_path + "/mongo_cram.xml");
		else 
			xmlContent =  writeXMLToString(dictionary_path + "/dictionary_cram.xml");
		Object o = xStream.fromXML(xmlContent);
		
		return (Map<String,String>) o;
	}

	public boolean initializeADTOntology(String actionClass, OWLNamedIndividual ind)
	{
		super.initializeADTOntology(actionClass, ind);
		return true;
	}

	public boolean parseOWL()
	{
		return super.parseOWL();
		
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
	}

	public boolean generateADTFromIndividual(String adtType, OWLIndividual ind, int count)
	{
		super.generateADTFromIndividual(adtType, ind, count, false);
		this.generateActionChuck(ind);
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

	public void makeObjectPropertyAssertion(OWLIndividual ind1, OWLIndividual ind2, OWLObjectProperty property)
	{
		super.makeObjectPropertyAssertion(ind1, ind2, property);
	}


	public void makeDataPropertyAssertion(OWLNamedIndividual ind, OWLLiteral lit, OWLDataProperty property)
	{
		super.makeDataPropertyAssertion(ind, lit, property);
	}

	public boolean generateActionChuck(OWLIndividual ind)
	{
		OWLObjectProperty chunkProperty = factory.getOWLObjectProperty(IRI.create(sub_action_type));
		Set<OWLIndividual> values =  ind.getObjectPropertyValues(chunkProperty, ontology);

		if(values != null && values.size() > 0)
		{
			for(OWLIndividual value : values)
			{
				OWLClass chunkClass = factory.getOWLClass(":#ActionChunk", adtPM);
				OWLNamedIndividual valueNew = factory.getOWLNamedIndividual
							(":#ActionChunk_" + value.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
				OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom(chunkClass, valueNew);
				manager.addAxiom(adtOntology, classAssertion);

				makeObjectPropertyAssertion(adt_of_interest, valueNew, chunkProperty);

				OWLNamedIndividual start = timeInstanceInd(value, true);
				OWLNamedIndividual end = timeInstanceInd(value, false);

				OWLObjectProperty startProperty = factory.getOWLObjectProperty(IRI.create(start_time_type));

				OWLObjectProperty endProperty = factory.getOWLObjectProperty(IRI.create(end_time_type));

				makeObjectPropertyAssertion(valueNew, start, startProperty);

				makeObjectPropertyAssertion(valueNew, end, endProperty);

				//TODO: trajectory and supporting planes addition
				createTrajectoryForActionChunk(valueNew, timeInstance(value, true), timeInstance(value, false), "/l_wrist_roll_link");

			}

		}
		return true;
	}

	public boolean createTrajectoryForActionChunk(OWLNamedIndividual chunk, float start, float end, String model)
	{
		System.out.println(start + " " + end);
		if(start < end)
		{
			

			String trajectorySamples = "";
			for(double x = (double)start; x < end; x = x + 0.1)
			{
				StampedTransform trans = mongo.lookupTransform("/map", model, x);
				Vector3d t = trans.getTranslation();
				Quat4d q = trans.getRotation();
				
				trajectorySamples = trajectorySamples.concat(t.x + " " + t.y + " " + t.z + " " + q.w + " " + q.x + " " + q.y + " " + q.z + " ");

			}

			OWLClass trajectoryClass = factory.getOWLClass(IRI.create(trajectory_type));
			OWLNamedIndividual trajectoryInstance = factory.getOWLNamedIndividual(":#Trajectory_" + chunk.asOWLNamedIndividual().getIRI().getFragment(), adtExamplePM);
			OWLClassAssertionAxiom trajectoryClassAssertion = factory.getOWLClassAssertionAxiom(trajectoryClass, trajectoryInstance);
			manager.addAxiom(adtOntology, trajectoryClassAssertion);

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
		return true;
	}

}
