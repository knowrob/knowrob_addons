/*
 * Copyright (c) 2017 Asil Kaan Bozcuoglu
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


package org.knowrob.cloud_logger;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class Tester extends AbstractNodeMain {

    ConnectedNode node;
	
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("logger_test_client");
    }
   
    @Override
    public void onStart(final ConnectedNode connectedNode){
        try {
           LoggerClient client = new LoggerClient("6ESR8hTRBepTSj1Vy00kJDJRryUPAqKOHKI8dFwrCxubZfGXCtDHkr6SS8I2bRlP",
                "https://localhost", "/home/asil/Desktop/pem/localhost.crt.pem");
           client.startUserContainer();
           client.connectToUserContainer();
           try {
              Thread.sleep(3000);
           } catch (InterruptedException e) {
              e.printStackTrace();
           }
           String id = client.sendPrologQuery("cram_start_action(knowrob:'CRAMAction', 'DummyContext', 1492785072, PA, ActionInst)", false);
           System.out.println("id" + id);
           client.sendPrologNextSolution(id);
           String newAction = "'" + client.readPrologNextSolution("ActionInst") + "'";
           String id2 = client.sendPrologQuery("cram_finish_action(" + newAction + ", 1492785075)", false);
           client.sendPrologNextSolution(id2);
           System.out.println(client.readPrologNextSolution());
           String id3 = client.sendPrologQuery("rdf_save('/home/ros/user_data/w.owl', [graph('LoggingGraph'), write_xml_base(true), base_uri('http://knowrob.org/kb/knowrob.owl#'), inline(true), sorted(false), anon(false)])", false);
           //String id3 = client.sendPrologQuery("rdf_save_db('/home/ros/user_data/w.owl', 'LoggingGraph')", false);
           client.sendPrologNextSolution(id3);
           System.out.println(client.readPrologNextSolution());
        }
        catch(Exception e)
        {
           e.printStackTrace(System.out);
        }
    }
}
