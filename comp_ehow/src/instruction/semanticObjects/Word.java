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

/**
 * 
 * @author Daniel Nyga
 * 
 */
public class Word extends SemanticObject {

	public static final int TYPE_UNKNOWN = - 1;
	public static final int TYPE_NOUN = 0;
	public static final int TYPE_VERB_INFINITIVE = 1;
	public static final int TYPE_ADJ = 2;
	public static final int TYPE_ADV = 3;
	public static final int TYPE_PARTICLE = 4;
	public static final int TYPE_NUMBER = 5;
	public static final int TYPE_CONJ = 7;
	public static final int TYPE_PRONOUN = 8;
	public static final int TYPE_PREPOSITION = 9;
	public static final int TYPE_ARTICLE = 10;
	public static final int TYPE_PUNCTUATION = 11;
	public static final int TYPE_PAST_PARTICIPLE = 12;
	public static final int TYPE_GERUND = 13;

	/**
	 * Specifies the type of the Word. One of the <code>TYPE_</code>-Fields
	 */
	private int type;

	/**
	 * The label of the Word
	 */
	private String label;

	/**
	 * A List of WordNet-SynSet-IDs.<br>
	 * <b>Is not automatically filled!</b><br>
	 * Use <code>WordNet.getSynIDs() method to retrieve them!</code>
	 */
	private ArrayList<String> synSetIDs = new ArrayList<String>();

	/**
	 * A List of Cyc-Concepts associated with the SynSets of
	 * <code>synSetIDs</code><br>
	 * <b>Is not automatically filled!</b>
	 */
	private ArrayList<String> cycConcepts = new ArrayList<String>();

	/**
	 * Constructs a new <code>Word</code> with empty label and unknown type
	 */
	public Word () {

		type = TYPE_UNKNOWN;
		label = "";
	}

	public Word ( int type, String label ) {

		this.type = type;
		this.label = label;
	}

	public ArrayList<String> getSynSetIDs() {

		return synSetIDs;
	}

	public ArrayList<String> getCycConcepts() {

		return cycConcepts;
	}
	
	public void setCycConcepts(ArrayList<String> concepts) {
		cycConcepts = concepts;
	}

	public String getLabel() {

		return label;
	}

	public int getType() {

		return type;
	}

	public void setLabel( String label ) {

		this.label = label;
	}

	public void setType( int type ) {

		this.type = type;
	}

	public void setSynSetIDs( ArrayList<String> synsets ) {

		synSetIDs = synsets;
	}

	public String toString() {

		String str = "(" + label + "," + typeToString( type ) + ",[";

		str += synSetIDs.size() + "/";
		for ( Iterator<String> i = cycConcepts.iterator(); i.hasNext(); ) {
			str += i.next();
			if ( i.hasNext() )
				str += ",";
		}
		str += "])";

		return str;
	}
	


	public String typeToString(int type) {
		String str = "N/A";
		switch (type) {
		case TYPE_UNKNOWN:
			str = "?";
			break;
		case TYPE_NOUN:
			str = "N";
			break;
		case TYPE_VERB_INFINITIVE:
			str = "V";
			break;
		case TYPE_ADJ:
			str = "ADJ";
			break;
		case TYPE_ADV:
			str = "ADV";
			break;
		case TYPE_PARTICLE:
			str = "PART";
			break;
		case TYPE_NUMBER:
			str = "NUMBER";
			break;
		case TYPE_CONJ:
			str = "CONJ";
			break;
		case TYPE_PRONOUN:
			str = "PRON";
			break;
		case TYPE_PREPOSITION:
			str = "PP";
			break;
		case TYPE_ARTICLE:
			str = "DT";
			break;
		case TYPE_PUNCTUATION:
			str = ".";
			break;
		}
		return str;
	}

	/**
	 * Compares two Words for equality {@link Object.equals}
	 */
	public boolean equals( Object w ) {

		if ( ! ( w instanceof Word ) )
			return false;

		if ( label.equalsIgnoreCase( ( (Word) w ).getLabel() ) && ( (Word) w ).getType() == type )
			return true;
		return false;
	}
}
