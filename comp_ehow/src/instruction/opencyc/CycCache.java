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
package instruction.opencyc;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.jdom.Document;
import org.jdom.Element;
import org.jdom.JDOMException;
import org.jdom.input.SAXBuilder;
import org.jdom.output.XMLOutputter;

public class CycCache {

	public static final String TAG_CACHE = "cyccache";

	public static final String TAG_SYNSET = "synsetid";

	public static final String TAG_CONCEPT = "cycconcept";

	public static final String ATTR_VALUE = "value";

	Map<String, List<String>> map = new HashMap<String, List<String>>();

	/**
	 * Puts a new list of Cyc concept names to be associated with the Synset-ID
	 * <code>synset</code> to the cache
	 * 
	 * @param synset
	 * @param concepts
	 */
	public void tell( String synset, List<String> concepts ) {

		map.put( synset, concepts );
	}

	/**
	 * Checks if the Synset-ID <code>synset</code> already has been cached
	 * 
	 * @param synset
	 * @return <code>null</code> if the Synset-ID has not been found in the
	 *         cache
	 */
	public List<String> ask( String synset ) {

		return map.get( synset );
	}

	/**
	 * Loads the cache file <code>etc/cyccache.xml</code>
	 * 
	 * @throws IOException
	 * @throws
	 * @throws Exception
	 */
	@SuppressWarnings("unchecked")
	public void load() throws IOException {

		File file = new File( "etc/cyccache.xml" );
		if ( ! file.exists() )
			return;

		FileReader reader = new FileReader( file );

		SAXBuilder sax = new SAXBuilder();
		try {
			Document doc;

			doc = sax.build( reader );

			Element cache = doc.getRootElement();

			List<Element> synsetids = cache.getChildren();

			// ==================================================================
			// Iterate over Synset-IDs
			// ==================================================================
			for ( Iterator<Element> i = synsetids.iterator(); i.hasNext(); ) {

				Element synsetid = i.next();
				String id = synsetid.getAttributeValue( ATTR_VALUE );
				if ( id == null )
					continue;

				List<String> conceptNames = new ArrayList<String>();
				List<Element> concepts = synsetid.getChildren();

				// ==================================================================
				// Iterate over associated Cyc concept names
				// ==================================================================
				for ( Iterator<Element> j = concepts.iterator(); j.hasNext(); ) {
					Element concept = j.next();
					String conceptName = concept.getAttributeValue( ATTR_VALUE );
					if ( conceptName == null )
						continue;
					conceptNames.add( conceptName );

				}

				map.put( id, conceptNames );

			}
		}
		catch ( JDOMException e ) {
			e.printStackTrace();
		}

	}

	/**
	 * Saves the cache to the file <code>etc/cyccache.xml</code>
	 * 
	 * @throws Exception
	 */
	public void save() throws Exception {

		File file = new File( "etc/cyccache.xml" );
		if ( ! file.exists() )
			file.createNewFile();

		Element cache = new Element( TAG_CACHE );

		Set<String> synsets = map.keySet();

		// ==================================================================
		// Iterate over all keys (Synset-IDs) contained in the cache
		// ==================================================================
		for ( Iterator<String> i = synsets.iterator(); i.hasNext(); ) {
			Element synset = new Element( TAG_SYNSET );
			String synsetid = i.next();
			synset.setAttribute( ATTR_VALUE, synsetid );

			// ==================================================================
			// Iterate over all Cyc concept names associated with the current
			// Synset-ID
			// ==================================================================
			List<String> concepts = map.get( synsetid );

			for ( Iterator<String> j = concepts.iterator(); j.hasNext(); ) {
				String conceptName = j.next();
				Element concept = new Element( TAG_CONCEPT );
				concept.setAttribute( ATTR_VALUE, conceptName );
				synset.addContent( concept );
			}
			cache.addContent( synset );
		}

		// ==================================================================
		// Create XML document and write it to file
		// ==================================================================
		Document doc = new Document( cache );

		XMLOutputter outPutter = new XMLOutputter( " ", true );
		FileWriter outStream = new FileWriter( file );

		outPutter.output( doc, outStream );
	}

}
