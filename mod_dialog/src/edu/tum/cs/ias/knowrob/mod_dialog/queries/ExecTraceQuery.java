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

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.tum.cs.ias.knowrob.vis.applets.CommunicationVisApplet;
import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;



public class ExecTraceQuery extends SimpleTopLevelQuery {
	
	public ExecTraceQuery(DialogModule mod) {
		super(mod);
	}

	
	boolean exec_trace_initialized=false;
	void initExecutionTrace() {
		if(!exec_trace_initialized) {
		//	DialogModule.executeLispPrologQuery("(with-execution-trace \"/work/ros/tumros-internal/stacks/cram_demo_apps/rosie_execution_trace_visualization/execution_traces/20101005150335-PICK-AND-PLACE-OBJ.ek\")");
			exec_trace_initialized = true;
		}
		DialogModule.executeLispPrologQuery("(clear-markers)");
		
	}
			
			
	@Override
	public String match(String q) {



		Matcher matcher = Pattern.compile("^([f|F]rom where did you try to pick up an object\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-tsk-location (achieve (object-in-hand ?_ ?_)) ?_ ?_))";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "Over here\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}

//		Where did you stand while picking up an object? (successful)
//		(visualize-tsk-location (achieve (object-in-hand ?_ ?_) ?_ :succeeded))

		matcher = Pattern.compile("^([f|F]rom where did you (successfully)? pick up an object \\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-tsk-location (achieve (object-in-hand ?_ ?_)) ?_ :succeeded)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "There.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}

		
		matcher = Pattern.compile("^([f|F]rom where did you put down an object\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-tsk-location (achieve (object-placed-at ?_ ?_)) ?_ ?_)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "Over there\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}


		matcher = Pattern.compile("^([w|W]here did you detect objects\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-object-detection ?_ ?_)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "At these locations.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}


		matcher = Pattern.compile("^(how did you move while grasping the object\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-robot-trajectory (achieve (object-in-hand ?_ ?_)) ?_ ?_)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "Like this.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}
		

		matcher = Pattern.compile("^(how did you move the arm while grasping the object\\?)").matcher(q);
		if(matcher.find()) {
			
			q=this.dialog_module.getCurrentObject();
			dialog_module.setVoice("dfki-obadiah");
			
			
			// visualize communication
			String query = "(visualize-arm-trajectory (achieve (object-in-hand ?_ ?_)) ?_ ?_)";
	        CommunicationVisApplet.visualizeCommunication(query, null, "", "pr2.jpg");

	        initExecutionTrace();
	        DialogModule.executeLispPrologQuery(query);

			
			String res = "Like that.\n"; 
			CommunicationVisApplet.visualizeCommunication(null, res, "", "pr2.jpg");

			return res;
		}
		
		return null;
	}

	

}
