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

import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Word;
import java.util.List;

public class PronounResolverPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		ObjectX lastObjectTalkedAbout = null;
		for ( int i = 0; i < instructions.size(); i++ ) {
			lastObjectTalkedAbout = resolvePronouns( instructions.get( i ), lastObjectTalkedAbout );
		}
	}

	public ObjectX resolvePronouns( Instruction inst, ObjectX object ) {

		ObjectX lastObjectTalkedAbout = object;

		List<ObjectX> objects = inst.getObjects();

		for ( int i = 0; i < objects.size(); i++ ) {
			List<Word> objName = objects.get( i ).getName();

			/**
			 * Resolution is ony possible, if the word name consists only of 1 Word
			 * which is a pronoun. lastObjectTalkedAbout must be non-null.
			 */
			if ( objName.size() == 1 && objName.get( 0 ).getType() == Word.TYPE_PRONOUN ) {
				if ( lastObjectTalkedAbout == null ) {
					inst.getObjects().remove( i );
					i -= 1;
					continue;
				}
				else {
					inst.getObjects().remove( i );
					inst.getObjects().add( i, lastObjectTalkedAbout );
				}
			}
			else
				lastObjectTalkedAbout = objects.get( i );

		}

		return lastObjectTalkedAbout;
	}
}
