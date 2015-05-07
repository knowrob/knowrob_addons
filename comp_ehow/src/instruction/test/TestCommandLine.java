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

import java.io.IOException;
import java.util.List;
import instruction.disambiguator.Disambiguator;
import instruction.factory.InstructionFactory;
import instruction.postprocessor.InstructionPostProcessor;
import instruction.semanticObjects.Instruction;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;

public class TestCommandLine {

	
	public static boolean ARG_PRINT_PARSER_TREE = true;
	
	/**
	 * @param args
	 */
	public static void main( String[] args ) {

		Parser parser = new Parser();
		InstructionFactory factory = new InstructionFactory( parser );
		Disambiguator disambiguator = new Disambiguator();
		InstructionPostProcessor postProc = new InstructionPostProcessor();

		System.out.println( "Initializing OpenCyc..." );
		System.out.println( "Ready for Services" );

		try {
			while ( true ) {
				System.out.print(">");
				String s = "";
				int r;
				do {
					r = System.in.read();
					s += (char) r;
				}
				while ( r != (char) '\n' );
				
				parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS
						| Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY );
				SyntaxTree t = parser.parse( s );
				
				if (ARG_PRINT_PARSER_TREE)
					t.printTree();
				
				try {
					
					System.out.print("Making Instructions...");
					List<Instruction> instructions = factory.makeInstructions( t );
					System.out.println("Done.");
					
					System.out.print("Retrieving Meanings of Words...");
		//			factory.findMeaningsForInstructions( instructions );
					System.out.println("Done.");
					
					System.out.print("Running Post-Processor...");
					postProc.run( instructions );
					System.out.println("Done.");
				
					System.out.print("Running Disambiguator...");
					disambiguator.disambiguateInstructions( instructions );
					System.out.println("Done.");
					
					// Print Instructions
					for (int i = 0; i < instructions.size(); i++) {
						System.out.println(instructions.get( i ));
					}
					
				}
				catch ( Exception e ) {
					System.out.println(e.getMessage());
					e.printStackTrace();
				}
				
				
			}
		}
		catch ( IOException e ) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
