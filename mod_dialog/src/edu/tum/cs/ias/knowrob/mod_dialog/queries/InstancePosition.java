/*
 * Copyright (c) 2010 Moritz Tenorth
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

package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.util.HashMap;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.vis.applets.CommunicationVisApplet;

import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;

public class InstancePosition extends SimpleTopLevelQuery {
	
	public InstancePosition(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		
		Matcher matcher = Pattern.compile("([w|W]here exactly is ([a-zA-Z0-9]*?)\\?)").matcher(q);
		if(matcher.find()) {
			q=matcher.group(2);
			dialog_module.setCurrentObject(DialogModule.toProlog(q));

			
			// visualize communication
			String shortquery = "rdf_triple(orientation, "+DialogModule.toProlog(q)+", Pose),\n" +
			"rdf_triple(m03, Pose, X),\n" +
			"rdf_triple(m13, Pose, Y),\n" +
			"rdf_triple(m23, Pose, Z)";
	        CommunicationVisApplet.visualizeCommunication(shortquery, null, null, "rosie.png");

			
			String query = "rdf_triple(knowrob:orientation, "+DialogModule.toProlog(q)+", Pose)," +
			"rdf_triple(knowrob:m03, Pose, literal(type('http://www.w3.org/2001/XMLSchema#float', X)))," +
			"rdf_triple(knowrob:m13, Pose, literal(type('http://www.w3.org/2001/XMLSchema#float', Y)))," +
			"rdf_triple(knowrob:m23, Pose, literal(type('http://www.w3.org/2001/XMLSchema#float', Z)))";
	        
			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);
			String x = res.get("X").toString(); x=x.substring(1, x.length()-1);
			String y = res.get("Y").toString(); y=y.substring(1, y.length()-1);
			String z = res.get("Z").toString(); z=z.substring(1, z.length()-1);
			

			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, "Position: ["+x+", "+y+", "+z+"]", null, "rosie.png");

			
			return "At position ("+x+", "+y+", "+z+").\n"; 
		}
		
		return null;
	}

}
