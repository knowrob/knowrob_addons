package org.knowrob.memory_graph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;

public class Edge
{
	private Node endNode;
	private Node startNode;

	public Edge(Node from, Node to) 
	{
		startNode = from;
      		endNode = to;
    	}

	public void setEndNode(Node to)
	{
		endNode = to;
	}

	public Node getEndNode()
	{
		return endNode;
	}

	public void setStartNode(Node from)
	{
		startNode = from;
	}

	public Node getStartNode()
	{
		return endNode;
	}	


}
