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
import java.util.List;

public class ObjectX extends SemanticObject {

	/**
	 * A list of Words that determine the object's name
	 */
	private List<Word> name = new ArrayList<Word>();

	/**
	 * Specifies an quantitative statement about the object
	 */
	private Quantifier quantifier = new Quantifier();

	/**
	 * Makes a prepositional statement about the object, e.g. its location
	 */
	private List<Preposition> prepositions = new ArrayList<Preposition>();

	/**
	 * Contains a set of adjectives that characterize the object more exactly
	 */
	private List<Word> adjectives = new ArrayList<Word>();

	/**
	 * Conatins a set of simple sentences that specify the object, e.g. relative
	 * clauses
	 */
	private List<Description> descriptions = new ArrayList<Description>();
	
	private List<Word> participles = new ArrayList<Word>();

	/**
	 * Constructs a new ObjectX object and adds the <code>Word</code> 
	 * <code>name</code>
	 * as a component to the object's name
	 * 
	 * @param obj
	 */
	public ObjectX ( Word name ) {

		this.name.add( name );
	}

	/**
	 * Constructs an empty <code>ObjectX</code>
	 */
	public ObjectX () {

	}

	public void addNameComponent( Word n ) {

		name.add( n );
	}

	public List<Word> getName() {

		return name;
	}

	public void setName( List<Word> n ) {

		name = n;
	}

	public Quantifier getQuantifier() {

		return quantifier;
	}

	public List<Description> getDescriptions() {

		return descriptions;
	}

	public void addDescription( Description desc ) {

		descriptions.add( desc );
	}

	public void setDescriptions( ArrayList<Description> descs ) {

		descriptions = descs;
	}

	public void setQuantifier( Quantifier q ) {

		quantifier = q;
	}

	public List<Word> getAdjectives() {

		return adjectives;
	}

	public void addPreposition( Preposition loc ) {

		prepositions.add( loc );
	}

	public List<Preposition> getPrepositions() {

		return prepositions;
	}

	public String toString() {

		String str = "(";

		for ( int i = 0; i < name.size(); i++ ) {
			str += name.get( i );
			if ( i < name.size() - 1 )
				str += ",";
		}
		str += ",";

		str += "{";
		for ( int i = 0; i < adjectives.size(); i++ ) {
			str += adjectives.get( i );
			if ( i + 1 < adjectives.size() )
				str += ",";
		}
		str += "},";

		str += "{";
		for ( int i = 0; i < prepositions.size(); i++ ) {
			str += prepositions.get( i );
			if ( i < prepositions.size() - 1 )
				str += ",";
		}
		str += "})";
		return str;
	}

	public boolean equals( Object o ) {

		if ( ! ( o instanceof ObjectX ) )
			return false;

		boolean equals = true;

		equals = equals && name.equals( ( (ObjectX) o ).getName() );
		equals = equals && adjectives.equals( ( (ObjectX) o ).getAdjectives() );
		equals = equals && prepositions.equals( ( (ObjectX) o ).getPrepositions() );
		equals = equals && quantifier.equals( ( (ObjectX) o ).getQuantifier() );

		return equals;
	}
	
	public void setParticiples( List<Word> participles ) {

		this.participles = participles;
	}
	
	public List<Word> getParticiples() {

		return participles;
	}
	
	public void addParticiple(Word part) {
		
		participles.add( part );
		
	}
}
