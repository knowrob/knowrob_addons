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
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.List;

import org.opencyc.api.CycApiException;

public class TimeConstraintPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ ) {
			findTimeConstraint( instructions.get( i ), instructions.get( i ), instructions.get( i ) );
		}
	}

	private void findTimeConstraint( SemanticObject so, Instruction parentInstruction, SemanticObject parentSO ) {

		if ( so instanceof Instruction ) {
			Instruction inst = (Instruction) so;

			List<ObjectX> objects = inst.getObjects();
			for ( int i = 0; i < objects.size(); i++ )
				findTimeConstraint( objects.get( i ), parentInstruction, parentSO );

			List<Preposition> pp = inst.getPrepositions();
			for ( int i = 0; i < pp.size(); i++ )
				findTimeConstraint( pp.get( i ), parentInstruction, parentSO );

		}

		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			List<Preposition> pp = o.getPrepositions();
			for ( int i = 0; i < pp.size(); i++ )
				findTimeConstraint( pp.get( i ), parentInstruction, o );
		}

		else if ( so instanceof Preposition ) {
			Preposition pp = (Preposition) so;

			// Explore the Prepositions
			for ( int j = 0; j < pp.getPrepositions().size(); j++ ) {

				if ( pp.getPrepositions().get( j ).getLabel().equalsIgnoreCase( "for" ) ) {

				//	System.out.println( "found prep \"for\"" );
					for ( int k = 0; k < pp.getObjects().size(); k++ ) {

						Word time = null;
						try {
							time = getTimeMeasure( pp.getObjects().get( k ) );
						}
						catch ( Exception e ) {
							System.out.println(e.getMessage());
						}
						

						if ( time != null ) {

							// Build the time constraint
							Quantifier q = new Quantifier();
							q.setMeasure( time );
							q.setAlternatives( pp.getObjects().get( k ).getQuantifier().getAlternatives() );

							if ( q.getAlternatives().size() == 0 )
								q.getAlternatives().add( new Word( Word.TYPE_NUMBER, "1" ) );

							parentInstruction.setTimeConstraint( q );

							// Remove the preposition identified as time constraint from the
							// parent Semantic Object
							if ( parentSO instanceof Instruction )
								( (Instruction) parentSO ).getPrepositions().remove( pp );
							else if ( parentSO instanceof ObjectX )
								( (ObjectX) parentSO ).getPrepositions().remove( pp );
						}
					}
				}
			}
		}
	}

	/**
	 * Checks if the given <code>ObjectX o</code> represents a time measuring
	 * unit and returns the corresponding <code>Word</code>
	 * 
	 * @param o
	 * @return
	 * @throws IOException
	 * @throws CycApiException
	 * @throws UnknownHostException
	 */
	private Word getTimeMeasure( ObjectX o ) throws UnknownHostException, CycApiException, IOException {

		for ( int i = 0; i < o.getName().size(); i++ ) {
			for ( int j = 0; j < o.getName().get( i ).getCycConcepts().size(); j++ ) {
				String c = o.getName().get( i ).getCycConcepts().get( j );
				if ( OpenCyc.getInstance().isaInUnivVocMt( c, "UnitOfTime" ) )
					return o.getName().get( i );
			}
		}
		return null;
	}

}
