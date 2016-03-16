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
package instruction.ros;

import ros.*;
import ros.pkg.comp_ehow.srv.EhowToOWL;
import instruction.exporter.owl.OWLExporter;


public class EhowImportService {

	Ros ros;
	NodeHandle n;
	OWLExporter owl_ex;
	
	public EhowImportService() {
		
  		try {

  			// wait for Cyc to be available
  			// MT: does not work at the moment; there are lots of exceptions if a CycConnection
  			// has been tried to open while Cyc was not available and then successfully opened
  			//
//  			System.out.print("Waiting for OpenCyc..");
//  			int i=0;
//  			while(true) {
//	  			try {
//	  				if((i++)%100==0)
//	  					System.out.print(".");
//	  				CycConnection cyc = new CycConnection();
//	  				System.out.println("\nCyc found: "+cyc.connectionInfo());
//	  				cyc.close();
//					break;
//					
//				} catch (UnknownHostException e) { }
//				catch (CycApiException e) { }
//				catch (IOException e) { }
//  			}
  			
  			owl_ex = new OWLExporter();
  			
  			ros = Ros.getInstance();
  			ros.init("ehow_importer");
  			n = ros.createNodeHandle();
  			
			n.advertiseService("/ehow_importer/ehow_to_owl",  new EhowToOWL(),  new EhowToOWLCallback());
			ros.logInfo("Started ehow import service.");
			
			while(n.isValid())
				ros.spinOnce();
	    		
  		} catch(RosException e) {
  			e.printStackTrace();
  		}
	}
	
	/**
	 * 
	 * The callback class for updating the left image in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class EhowToOWLCallback implements ServiceServer.Callback<EhowToOWL.Request, EhowToOWL.Response> {
		
		@Override
		public EhowToOWL.Response call(EhowToOWL.Request req) {

			EhowToOWL.Response res = new EhowToOWL.Response();
			res.owl_instructions="";

			if (req.command != null && req.command.length() > 0) {
				res.owl_instructions = owl_ex.convertHowtoToOWLOntology(req.command);
			}

			return res;
		}
	}

	public static void main(String args[]) {
		new EhowImportService();
	}
}
