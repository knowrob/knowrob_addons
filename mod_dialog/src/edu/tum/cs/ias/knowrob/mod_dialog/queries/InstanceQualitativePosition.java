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

public class InstanceQualitativePosition extends SimpleTopLevelQuery {

	public InstanceQualitativePosition(DialogModule mod) {
		super(mod);
	}

	@Override
	public String match(String q) {
		

		Matcher matcher = Pattern.compile("([w|W]here is ([a-zA-Z0-9]*?)\\?)").matcher(q);
		if(matcher.find()) {
			q=matcher.group(2);
			q=DialogModule.toProlog(q);
			dialog_module.setCurrentObject(q);

			// visualize communication
			String shortquery = "rdf_has(Prop, subPropertyOf, topologicalRelations)," +
			"rdf_triple(A, "+q+", O)";
	        CommunicationVisApplet.visualizeCommunication(shortquery, null, null, "rosie.png");
	
			
			String query = "rdf_has(Prop, rdfs:subPropertyOf, knowrob:topologicalRelations)," +
			"rdf_triple(A, "+q+", O)";

			HashMap<String, Vector<PrologValue>> res = DialogModule.executeJSONPrologQuery(query);

			if(res.get("Prop")==null || res.get("O")==null)
				return null;
			
			String prop = res.get("Prop").toString();
			String obj  = res.get("O").toString();
			
			// visualize communication
	        CommunicationVisApplet.visualizeCommunication(null, prop + ": " + obj, null, "rosie.png");
			
			return prop + " "+obj+".\n"; 
		}
		
		return null;
	}

}
