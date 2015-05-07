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
package instruction.postprocessor;

import instruction.opencyc.OpenCyc;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;

import java.util.ArrayList;
import java.util.List;

public class ObjectAmountPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ )
			findObjectAmount( instructions.get( i ), instructions.get( i ) );

	}

	/**
	 * Recursively traverses nested <code>SemanticObject</code>s to find
	 * composed prepositions (e.g. "next to")
	 * 
	 * @param so
	 * @throws Exception
	 */
	private void findObjectAmount( SemanticObject so, SemanticObject parent ) {

		// ==================================================================
		// Instructions
		// ==================================================================
		if ( so instanceof Instruction ) {
			Instruction in = (Instruction) so;

			// Objects
			for ( int i = 0; i < in.getObjects().size(); i++ )
				findObjectAmount( in.getObjects().get( i ), in );

			// Prepositions
			for ( int i = 0; i < in.getPrepositions().size(); i++ )
				findObjectAmount( in.getPrepositions().get( i ), in );
		}

		// ==================================================================
		// Objects
		// ==================================================================
		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			for ( int i = 0; i < o.getName().size(); i++ ) {

				Word n = o.getName().get( i );

				for ( int j = 0; j < n.getCycConcepts().size(); j++ ) {

					String concept = n.getCycConcepts().get( j );

					List<ObjectX> newObjects = transformQuantifier( o, concept );

					if ( ! newObjects.isEmpty() ) {
						if ( parent instanceof Instruction ) {
							( (Instruction) parent ).getObjects().remove( o );
							( (Instruction) parent ).getObjects().addAll( newObjects );
						}
						else if ( parent instanceof Preposition ) {
							( (Preposition) parent ).getObjects().remove( o );
							( (Preposition) parent ).getObjects().addAll( newObjects );
						}

						findObjectAmount( parent, parent );
						break;
					}
				}

			}

			// Prepositions
			for ( int i = 0; i < o.getPrepositions().size(); i++ ) {
				findObjectAmount( o.getPrepositions().get( i ), o );
			}

		}

		// ==================================================================
		// Prepositions
		// ==================================================================
		else if ( so instanceof Preposition ) {
			Preposition pp = (Preposition) so;

			// Objects
			for ( int i = 0; i < pp.getObjects().size(); i++ )
				findObjectAmount( pp.getObjects().get( i ), pp );
		}
	}

	private List<ObjectX> transformQuantifier( ObjectX object, String concept ) {

		List<ObjectX> newObjects = new ArrayList<ObjectX>();

		for ( int i = 0; i < object.getPrepositions().size(); i++ ) {
			Preposition pp = object.getPrepositions().get( i );

			for ( int j = 0; j < pp.getPrepositions().size(); j++ ) {
				Word p = pp.getPrepositions().get( j );

				if ( p.getLabel().equalsIgnoreCase( "of" ) ) {

					// parent object becomes quantifier of child objects

					Quantifier q = new Quantifier();

					try {
						if ( OpenCyc.getInstance().isaInUnivVocMt( concept, "IntegerExtent" )
								|| OpenCyc.getInstance().genlsInUniversalVocabularyMt( concept, "IntegerExtent" ) ) {
							if ( object.getName().size() > 0 ) {
								object.getName().get( 0 ).getCycConcepts().clear();
								object.getName().get( 0 ).getCycConcepts().add( concept );
								q.getAlternatives().add( object.getName().get( 0 ) );
							}
						}
						else if ( OpenCyc.getInstance().isaInUnivVocMt( concept, "UnitOfMeasure" )
								|| OpenCyc.getInstance().isaInUnivVocMt( concept, "PhysicalAmountSlot" )
								|| OpenCyc.getInstance().genlsInUniversalVocabularyMt( concept, "PhysicalAmountSlot" ) ) {
							if ( object.getName().size() > 0 ) {
								q.setMeasure( object.getName().get( 0 ) );
								object.getName().get( 0 ).getCycConcepts().clear();
								object.getName().get( 0 ).getCycConcepts().add( concept );
							}
							q.setAlternatives( object.getQuantifier().getAlternatives() );
						}
						else
							return newObjects;

						for ( int k = 0; k < pp.getObjects().size(); k++ ) {

							pp.getObjects().get( k ).setQuantifier( q );
							newObjects.add( pp.getObjects().get( k ) );
						}

						return newObjects;
					}
					catch ( Exception e ) {
						System.out.println( e.getMessage() );
					}
				}
			}
		}
		return newObjects;
	}

}
