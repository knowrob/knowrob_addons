/*
 * Copyright (c) 2010 Nacer Khalil
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.util.LinkedList;


public class MyNode 
{
	
	private boolean textNode;
	private String value;
	private LinkedList<MyNode> child;
	private int index;
	
	public MyNode(boolean textNode, String value)
	{
		index = 0;
		this.textNode = textNode;
		this.value = value;
		if(!textNode)
			child = new LinkedList<MyNode>();
	}
	
	public void addChild(MyNode node) throws Exception
	{
		if(textNode)
			throw new Exception("Cannot add child to TextNode");
		else
			child.add(node);
	}
	
	public MyNode getNext()
	{
		index++;
		return child.get(index - 1);		
	}
	
	public void resetCursor()
	{
		index = 0;
		if(child != null)
		{
			for(int i = 0;i < child.size(); i++)
			{
				resetChild(child.get(i));
			}
		}
	}
	private void resetChild(MyNode n)
	{
		n.index = 0;
		if(n.child != null)
		{
			for(int i = 0; i < n.child.size(); i++)
				resetChild(n.child.get(i));
		}
	}
	
	public boolean hasNext()
	{
		if(textNode || index == child.size())
			return false;
		return true;
	}
	
	public boolean isTextNode()
	{
		return textNode;
	}
	
	public String getValue()
	{
		return value;
	}
}
