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
package instruction.test;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import instruction.semanticObjects.Instruction;
import instruction.syntaxparser.Parser;

public class TestTrainingSet {

	/**
	 * @param args
	 */
	public static void main( String[] args ) {

		Parser parser = new Parser();
	//	InstructionFactory factory = new InstructionFactory( parser );
	//	Disambiguator disambiguator = new Disambiguator();
	//	InstructionPostProcessor postProc = new InstructionPostProcessor();

		File howtos = new File( "howtos" );

		if ( howtos.isDirectory() ) {
			String[] files = howtos.list();
			for ( int i = 0; i < files.length; i++ ) {
				File tmp = new File( files[i] );
				if ( tmp.isDirectory() )
					continue;

				parser.setFileToParse( howtos.getName() + File.separator + files[i] );

				ArrayList<Instruction> howto = new ArrayList<Instruction>();

				for ( Iterator<String> iter = parser.iterator(); iter.hasNext(); ) {
					String sentence = iter.next();
			//		System.out.println( sentence );

					for ( int k = 0; k <= Parser.PRE_PROCESSOR_LOWER_CASE; k += Parser.PRE_PROCESSOR_LOWER_CASE ) {
				//		parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS
				//				| Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY | k );

						//SyntaxTree tree = 
							parser.parse( sentence );
						// tree.printTree();
						try {
				//			List<Instruction> instructions = factory.makeInstructions( tree );
				//			factory.findMeaningsForInstructions( instructions );
				//			postProc.run( instructions );
							// disambiguator.disambiguateInstructions( instructions );
				//			howto.addAll( instructions );
				//			if ( ! instructions.isEmpty() )
				//				break;
						}
						catch ( Exception e ) {
							System.out.println( e.getMessage() );
							e.printStackTrace();
							continue;
						}
					}

				}
				System.out.println( "***************************\nHOW TO "
						+ files[i].replaceAll( "_", " " ).toUpperCase() );
				for ( int j = 0; j < howto.size(); j++ )
					System.out.println( howto.get( j ) );

			}
		}

	}

}
