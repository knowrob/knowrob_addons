/*
 * Copyright (c) 2009-10 Daniel Nyga
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
package instruction.semanticObjects;


import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Instruction extends SemanticObject {
	
	public static final int OPTIONAL_FALSE = 0;
	public static final int OPTIONAL_LOCAL = 1;
	public static final int OPTIONAL_GLOBAL = 2;
	
	Action action = new Action();
	
	String nlSentence = "";
	
	List<ObjectX> objects = new ArrayList<ObjectX>();
	
	List<Precondition> preconditions = new ArrayList<Precondition>();
	
	List<Postcondition> postconditions = new ArrayList<Postcondition>();
	
	List<Preposition> prepositions = new ArrayList<Preposition>();
	
	Quantifier timeConstraint = null;
	
	int optional = OPTIONAL_FALSE;
		
	
	public Instruction() {
	}
	
	public void setTimeConstraint(Quantifier q) {
		timeConstraint = q;
	}
	
	public Quantifier getTimeConstraint() {
		return timeConstraint;
	}
	
	public void setAction(Action action) {
		this.action = action;
	}
	
	public void addObject(ObjectX object) {
		objects.add(object);
	}
	
	public void setObjects(List<ObjectX> os) {
		objects = os;
	}
	
	public void setPrepositions(List<Preposition> pp) {
		prepositions = pp;
	}
	
	public void setPreconditions(List<Precondition> pc) {
		preconditions = pc;
	}
	
	public void setPostconditions(List<Postcondition> pc) {
		postconditions = pc;
	}

	public void addPreposition(Preposition prep) {
		prepositions.add(prep);
	}
	
	public Action getAction() {
		return action;
	}
	
	public List<ObjectX> getObjects() {
		return objects;
	}
	
	public List<Preposition> getPrepositions() {
		return prepositions;
	}
	
	public List<Postcondition> getPostconditions() {
		return postconditions;
	}
	
	public List<Precondition> getPreconditions() {
		return preconditions;
	}
	
	public String toString() {
		String str = "(";
//		str += "preconditions: ";
//		for (Iterator i = preconditions.iterator(); i.hasNext(); ) {
//			str += ((Precondition) i.next()).toString();
//			if (i.hasNext())
//				str += ", ";
//		}
		str += "\n\t" + action.getAction();
		
		str += ",\n\t{";
		for (Iterator i = objects.iterator(); i.hasNext(); ) {
			str += ((ObjectX) i.next()).toString();
			if (i.hasNext())
				str += ", ";
		}
	
		str += "},\n\t{";
		for (Iterator i = prepositions.iterator(); i.hasNext(); ) {
			str += ((Preposition) i.next()).toString();
			if (i.hasNext())
				str += ", ";
		}
		if (timeConstraint != null) {
			str += "},\n\t";
			str += timeConstraint.toString();
		}
		str += ")";
		
//		str += "\n\tpostconditions: ";
//		for (Iterator i = postconditions.iterator(); i.hasNext(); ) {
//			str += ((Postcondition) i.next()).toString();
//			if (i.hasNext())
//				str += ", ";
//		}
		
		
		return str;
	}

	
	public boolean equals(Object i) {
		if (! (i instanceof Instruction))
			return false;
		boolean equals = true;
		
		equals = equals && action.equals( ((Instruction) i).getAction() );
		equals = equals && objects.equals( ((Instruction) i).getObjects() );
		equals = equals && preconditions.equals( ((Instruction) i).getPreconditions() );
		equals = equals && postconditions.equals( ((Instruction) i).getPostconditions() );
		equals = equals && prepositions.equals( ((Instruction) i).getPrepositions() );
		
		return equals;

	}
	
	public void setNLSentence(String sentence) {
		nlSentence = sentence;
	}
	
	public String getNLSentence() {
		return nlSentence;
	}
	
	public boolean isOptional() {

		return optional == OPTIONAL_GLOBAL || optional == OPTIONAL_LOCAL;
	}
	
	public boolean isGlobalOptional() {
		return optional == OPTIONAL_GLOBAL;
	}
	
	public void setOptional( int optional ) {

		this.optional = optional;
	}
}
