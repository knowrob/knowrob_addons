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
package instruction.wordnet;

import java.util.HashMap;

/**
 * Maps prepositions to corresponding Cyc Concepts
 * 
 * @author Daniel Nyga
 * 
 */
public class PrepositionalMap {
	
	public static final String IN_FRONT_OF = "inFrontOf-Generally";
	public static final String IN = "in-UnderspecifiedContainer";
	public static final String ON = "on-UnderspecifiedSurface";
	public static final String NEXT_TO = "nextToLikeObjects";
	public static final String AT = "at-UnderspecifiedLandmark";
	public static final String OF = "parts-Underspecified";//"physicalParts";
	public static final String TO = "to-UnderspecifiedLocation";
	public static final String FROM = "from-UnderspecifiedLocation";
	public static final String THROUGH = "through-UnderspecifiedPortal";
	public static final String AMONG = "among-Underspecified";
	public static final String INSIDE = "inside-UnderspecifiedRegion";
	public static final String ABOUT = "about-UnderspecifiedRegion";
	public static final String WITHOUT = "without-Underspecified";
	public static final String ALONG = "along-UnderspecifiedPath";
	public static final String AROUND = "around-UnderspecifiedRegion";
	public static final String BY = "by-Underspecified";
	public static final String UNDER = "under-UnderspecifiedLocation";
	public static final String OVER = "over-UnderspecifiedLocation";
	public static final String WITH = "with-UnderspecifiedAgent";
	public static final String INTO = "into-UnderspecifiedContainer";
	public static final String ACROSS = "across-UnderspecifiedRegion";
	public static final String FOR = "for-UnderspecifiedLocation";
	public static final String AFTER = "after-Underspecified";
	public static final String BEFORE = "before-Underspecified";
	public static final String AGAINST = "against-Underspecified";

	/** The map: preposition -> Cyc Concept */
	private static HashMap<String, String> map = null;

	/**
	 * Returns the Cyc Constant that corresponds to the preposition <code>p</code>
	 * 
	 * @param p
	 * @return
	 */
	public static String get( String p ) {

		if ( map == null )
			init();
		return map.get( p.toLowerCase() );
	}

	/**
	 * Initialize the map - only for internal use
	 */
	private static void init() {

		map = new HashMap<String, String>();

		map.put( "in front of", IN_FRONT_OF );
		map.put( "in", IN );
		map.put( "into", INTO );
		map.put( "on", ON );
		map.put( "onto", ON );
		map.put( "next to", NEXT_TO );
		map.put( "at", AT );
		map.put( "of", OF );
		map.put( "to", TO );
		map.put( "from", FROM );
		map.put( "through", THROUGH);
		map.put( "among", AMONG );
		map.put( "inside", INSIDE );
		map.put( "about", ABOUT );
		map.put( "without", WITHOUT );
		map.put( "along", ALONG );
		map.put( "around", AROUND );
		map.put( "by", BY );
		map.put( "under", UNDER );
		map.put( "over", OVER );
		map.put( "with", WITH );
		map.put( "across", ACROSS );
		map.put( "for", FOR );
		map.put( "after", AFTER );
		map.put( "before", BEFORE );
		map.put( "against", AGAINST );
	}
}
