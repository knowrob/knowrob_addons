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
package instruction.wrapper;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class LocalFileWrapper implements IHowtoWebsiteWrapper {
	
	String title = "";
	List<String> instructions = new ArrayList<String>();
	String url = null;

	public String getHowtoTitle() {
		return title;
	}

	public List<String> getInstructions() {
		return instructions;
	}

	public void load( String url ) {
		File file = new File( url );
		
		this.url = url;

		try {
			// Read file to parse.
			String parseString = "";
			FileReader reader = new FileReader( file );
			int read = reader.read();

			while ( read >= 0 ) {
				parseString += (char) read;
				read = reader.read();
			}
			reader.close();

			// Parse every sentence since Standford Parser
			// can only cope with single sentences.
			parseString = parseString.replaceAll( "\r", "" );
			String[] sent = parseString.split( "\n" );
			instructions.clear();

			for ( int i = 0; i < sent.length; i++ ) {
				instructions.add( sent[i] );
			}
			
			// Get title of HowTo
			title = file.getName().replaceAll( "_", " " );
			
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	public String getUrl() {

		return url;
	}

}
