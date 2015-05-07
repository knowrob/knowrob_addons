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

public class Preposition extends SemanticObject {

	private List<Word> prepositions = new ArrayList<Word>();
	private List<ObjectX> objects = new ArrayList<ObjectX>();

	public Preposition () {

	}

	public Preposition ( Word p, ObjectX o ) {

		prepositions.add( p );
		objects.add( o );
	}
	
	public void setPrepositions(List<Word> newPP) {
		prepositions = newPP;
	}

	public List<ObjectX> getObjects() {

		return objects;
	}

	public List<Word> getPrepositions() {

		return prepositions;
	}

	public void addPreposition( Word pre ) {

		prepositions.add( pre );
	}

	public void addObject( ObjectX o ) {

		objects.add( o );
	}
	
	public void setObjects(List<ObjectX> newObjects) {
		
		objects = newObjects;
	}

	public String toString() {

		String str = "({";
		for ( int i = 0; i < prepositions.size(); i++ ) {
			str += prepositions.get( i ).getLabel();
			if ( i < prepositions.size() - 1 )
				str += ",";
		}
		str += "},{";
		for ( int i = 0; i < objects.size(); i++ ) {
			str += objects.get( i ).toString();
			if ( i < objects.size() - 1 )
				str += ",";
		}
		str += "})";
		return str;
	}

	public boolean equals( Object p ) {
		if (! (p instanceof Preposition))
			return false;
		boolean equals = true;

		equals = equals && prepositions.equals( ((Preposition) p).getPrepositions() );
		equals = equals && objects.equals( ((Preposition) p).getObjects() );
		return equals;
	}
}
