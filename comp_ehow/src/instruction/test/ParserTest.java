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

import instruction.disambiguator.Disambiguator;
import instruction.importer.PlanImporter;
import instruction.wrapper.LocalFileWrapper;
import java.io.File;

public class ParserTest {

	/**
	 * @param args
	 */
	public static void main( String[] args ) {
		
		PlanImporter importer = new PlanImporter();

		try {
			importer.initialize();
			System.out.println("Type the path to a HowTo:");
			while ( true ) {
				System.out.print(">");
				String s = "";
				int r;
				do {
					r = System.in.read();
					if (r != (char) '\n' && r != (char) '\r')
						s += (char) r;
				}
				while ( r != (char) '\n' );
				if (s.equals( "train" ))
					importer.getDisambiguator().setRunMode( Disambiguator.MODE_TRAIN );
				else if (s.equals( "apply" ))
					importer.getDisambiguator().setRunMode( Disambiguator.MODE_APPLY );
				else if (s.equals( "save" )) 
					importer.getDisambiguator().save( (new File(".")).getAbsolutePath() + File.separator + "etc/disambiguator.xml" );
				else if (s.equals( "load" ))
					importer.getDisambiguator().load( (new File(".")).getAbsolutePath() + File.separator + "etc/disambiguator.xml" );
				else {
					LocalFileWrapper wrapper = new LocalFileWrapper();
					wrapper.load( new File(".").getCanonicalPath() + File.separator + s );
					importer.importHowto( wrapper );
				}
			}
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
		

	}
}
