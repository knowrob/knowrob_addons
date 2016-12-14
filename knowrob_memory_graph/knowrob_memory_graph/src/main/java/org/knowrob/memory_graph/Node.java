package org.knowrob.memory_graph;

import java.util.ArrayList;
import java.util.Iterator;

public class Node
{
	private String name;
	private String owlClass;
	
	private final ArrayList<Edge> inEdges;
	private final ArrayList<Edge> outEdges;

	public Node(String name, String owlClass) 
	{
		this.name = name;
		this.owlClass = owlClass;
		inEdges = new ArrayList<Edge>();
		outEdges = new ArrayList<Edge>();
	}

	public ArrayList<Edge> getInEdges()
	{
		return inEdges;
	}

	public ArrayList<Edge> getOutEdges()
	{
		return outEdges;
	}

	public Edge addEdge(Node node)
        {
		Edge e = new Edge(this, node);
		outEdges.add(e);
		node.getInEdges().add(e);

		return e;
	}

}
