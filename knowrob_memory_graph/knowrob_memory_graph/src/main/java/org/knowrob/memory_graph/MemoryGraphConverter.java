/*
 * Copyright (c) 2016 Asil Kaan Bozcuoglu
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

package org.knowrob.memory_graph;

import java.io.*;
import java.util.StringTokenizer;
import java.lang.Double;
import java.util.ArrayList;
import java.text.DecimalFormat;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLOntologyStorageException;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLClassExpression;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.knowrob.tfmemory.TFMemory;

/**
 *  
 *
 * @author asil@cs.uni-bremen.de
 *
 */
public class MemoryGraphConverter 
{
	String path;
	private final String owlClassPrefix = "http://knowrob.org/kb/knowrob.owl#";
	ArrayList<String> classTypes;
	OWLOntologyManager manager;
	OWLOntology ontology;
	OWLDataFactory df;
	File owlFile;
	MemoryGraph memory;

	public MemoryGraphConverter(String path)
	{
		this.manager = OWLManager.createOWLOntologyManager();
		df = OWLManager.getOWLDataFactory();
		setPathOwlFile(path);		
		classTypes = new ArrayList<String>();
	}

	public void setPathOwlFile(String path)
	{
		try 
		{
			this.path = path;
			this.owlFile = new File(path);
			ontology = manager.loadOntologyFromOntologyDocument(owlFile);
		} 
		catch (OWLOntologyCreationException e) 
		{
			e.printStackTrace();
		}
	}

	public boolean addNewClassType(String newClass)
	{
		memory = null;
		classTypes.add(owlClassPrefix + newClass);
		return true;
	}

	public MemoryGraph convert()
	{
		ArrayList<Edge> edges = new ArrayList<Edge>(); 
		ArrayList<Node> nodes = new ArrayList<Node>();

		int class_size = classTypes.size();

		IRI[] class_iris = new IRI[class_size];
		OWLClass[] owl_classes = new OWLClass[class_size];

		for(int i = 0; i < class_size; i++)
		{
			class_iris[i] = IRI.create(classTypes.get(i));
			owl_classes[i] = df.getOWLClass(class_iris[i]);	
		}

		ArrayList<OWLNamedIndividual> important_individuals = new ArrayList<OWLNamedIndividual>();  
		for (OWLNamedIndividual current : ontology.getIndividualsInSignature())
		{
			Set<OWLClassExpression> current_object_classes = current.getTypes(ontology);	
			for(int i = 0; i < current_object_classes.size(); i++)
			{
				OWLClassExpression[] current_object_classes_arrays = current_object_classes.toArray(new OWLClassExpression[0]);
				OWLClass class_to_check = current_object_classes_arrays[i].asOWLClass();
				for(int j = 0; j < class_size; j++)
				{
					if(owl_classes[j].toStringID().equals(class_to_check.toStringID()))
					{
						important_individuals.add(current);
					}	
				}
			}
		}


		int important_individuals_size = important_individuals.size();

		for(int i = 0; i < important_individuals_size; i++)
		{
			OWLNamedIndividual source = important_individuals.get(i);

			for(int j = 0; j < important_individuals_size; j++)
			{
				OWLNamedIndividual target = important_individuals.get(j);
			}

		}


		MemoryGraph newOne = new MemoryGraph(edges, nodes);

		return newOne;

	}


}
