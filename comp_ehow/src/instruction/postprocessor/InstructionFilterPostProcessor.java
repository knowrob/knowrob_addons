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

import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.util.List;

public class InstructionFilterPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ ) {
			Instruction cur = instructions.get( i );
			Action a = cur.getAction();
			try {
				if ( ! WordNetRDF2.wordExactlyExistsAs( a.getAction().getLabel(), WordNetRDF2.convertPOS(Word.TYPE_VERB_INFINITIVE) ) ) {
				//	System.out.println("Word " + a.getAction().getLabel() + " not found in WordNet");
					instructions.remove( cur );
					i -= 1;
					continue;
				}
			}
			catch ( Exception e ) {
				System.out.println( e.getMessage() );
				instructions.remove( cur );
				i -= 1;
				continue;
			}
		}
	}

}
