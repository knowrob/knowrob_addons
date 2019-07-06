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
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLOntologyStorageException;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.knowrob.tfmemory.TFMemory;

/**
 *  
 *
 * @author asil@cs.uni-bremen.de
 *
 */

public class MemoryGraph 
{
	private ArrayList<Edge> edges;
	private ArrayList<Node> nodes;

	public MemoryGraph(ArrayList<Edge> edges, ArrayList<Node> nodes)
	{
		this.edges = edges;
		this.nodes = nodes;
	}

	public ArrayList<Node> getNodes()
	{
		return nodes;
	}

	public ArrayList<Edge> getEdges()
	{
		return edges;
	}

	public boolean addANewEdge(Node from, Node to)
	{
		Edge e = from.addEdge(to);
		edges.add(e);

		return true;
	}

	public boolean addANewNode(Node aNewNode)
	{
		nodes.add(aNewNode);

		for(int i = 0; i < aNewNode.getInEdges().size(); i++)
		{
			Edge current = aNewNode.getInEdges().get(i);
			edges.add(current);
		}

		for(int i = 0; i < aNewNode.getOutEdges().size(); i++)
		{
			Edge current = aNewNode.getOutEdges().get(i);
			edges.add(current);
		}		

		return true;
	}

}


