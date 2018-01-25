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
 *     * Neither the name of the University of Bremen nor the
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

import java.util.UUID;
import java.util.Timer;
import java.util.TimerTask;
import java.lang.Boolean;
import java.lang.Integer;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.openease.client.BridgeClient;
import org.openease.client.EASEError;

//import edu.wpi.rail.jrosbridge.Topic;
//import edu.wpi.rail.jrosbridge.messages.Message;
//import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;
//import edu.wpi.rail.jrosbridge.Ros;
//import edu.wpi.rail.jrosbridge.JRosbridge;

import org.json.JSONObject;

public class LoggerClient {

    BridgeClient oeClient;
    String oeAddress;
    String pathToPOM;

    ServiceRequest prologRequest;
    ServiceResponse prologResponse; 

     /* ********************************
        ********************************
        Service methods
        ********************************
        ******************************** */


    public String sendPrologQuery(String prolog, boolean isIncremental)
    {
          String id = randomStringGenerator();
          
          String[] values = new String[3];
          values[0] = id;
          values[1] = (isIncremental) ? "1" : "0";
	  values[2] = prolog;


	  String json = jsonizerServiceRequests(ServerType.PrologQuery, values);
          prologRequest = new ServiceRequest(json);
          prologResponse = oeClient.getPrologQuery().callServiceAndWait(prologRequest);


          return id;
    }

    public String readPrologQueryMessage()
    {
         return (String)readFromPrologResponse("message");
    }

    public boolean readPrologQueryOk()
    {
         return ((Boolean)readFromPrologResponse("ok")).booleanValue();
    }


    public void sendPrologNextSolution(String id)
    {
          String json = jsonizerServiceRequestsForSending(ServerType.PrologNextSolution, id);

          prologRequest = new ServiceRequest(json);
          prologResponse = oeClient.getPrologNext().callServiceAndWait(prologRequest);

    }

    public String readPrologNextSolution()
    {
         return (String)readFromPrologResponse("solution");
    }

    public String sendAndReadPrologNextSolution(String id)
    {
          String json = jsonizerServiceRequestsForSending(ServerType.PrologNextSolution, id);

          ServiceRequest prologRequestLocal = new ServiceRequest(json);
          ServiceResponse prologResponseLocal = oeClient.getPrologNext().callServiceAndWait(prologRequestLocal);

          return (String)readFromPrologResponse("solution", prologResponseLocal);

    }

    public Object readPrologNextSolution(String field)
    {
         String jsonSolution = readPrologNextSolution();
         JSONObject jsonObj = new JSONObject(jsonSolution);
         return jsonObj.get(field);
    }

    public int readPrologNextSolutionStatus()
    {
         return ((Integer)readFromPrologResponse("status")).intValue();
    }


     
    public void sendPrologFinish(String id)
    {
	  String json = jsonizerServiceRequestsForSending(ServerType.PrologFinish, id);

          prologRequest = new ServiceRequest(json);
          prologResponse = oeClient.getPrologFinish().callServiceAndWait(prologRequest);

    }

    


     /* ********************************
        ********************************
        Constructors and setters-getters
        ********************************
        ******************************** */

    public LoggerClient(String token, String address, String path) throws InterruptedException, EASEError, IOException 
    {
        oeAddress = address;
        pathToPOM = path;
        oeClient = new BridgeClient(token, oeAddress);
        oeClient.setSSLCertificate(Files.newInputStream(Paths.get(pathToPOM)));
        //oeClient.startContainer();
    }

    public LoggerClient(BridgeClient client, String address, String path) throws InterruptedException, EASEError, IOException 
    {
        oeAddress = address;
        pathToPOM = path;

        oeClient = client;
       
        if(!(oeClient.getHost().equals(address)))
           throw new EASEError("Host of BridgeClient and address given as parameter should be equal!");

        oeClient.setSSLCertificate(Files.newInputStream(Paths.get(pathToPOM)));
        //oeClient.startContainer();
    }

    public LoggerClient(String token, String path) throws InterruptedException, EASEError, IOException 
    {
       this(token, "https://data.open-ease.org", path);
    }

    public LoggerClient(BridgeClient client, String path) throws InterruptedException, EASEError, IOException 
    {
       this(client, "https://data.open-ease.org", path);
    }

    
    public void startUserContainer() throws EASEError
    {
       oeClient.startContainer();
    }


    public void connectToUserContainer() throws EASEError
    {
       oeClient.connect();
    }

    public void setOeClient(BridgeClient oeClient)
    {
      this.oeClient = oeClient;
    }

    public BridgeClient getOeClient()
    {
        return oeClient;
    }

    public void setOeAddress(String oeAddress)
    {
        this.oeAddress = oeAddress;
    }

    public String getOeAddress()
    {
        return oeAddress;
    }

    public void setPathToPOM(String pathToPOM)
    {
        this.pathToPOM = pathToPOM;
    }

    public String getPathToPOM()
    {
        return pathToPOM;
    }

     /* ********************************
        ********************************
        Helper methods
        ********************************
        ******************************** */

    private Object readFromPrologResponse(String field)
    {
        return readFromPrologResponse(field, prologResponse);   
    }

    private Object readFromPrologResponse(String field, ServiceResponse prologResponseLocal)
    {
        String jsonString = prologResponseLocal.toString();
        JSONObject jsonObj = new JSONObject(jsonString);

        if(field.equals("solution") || field.equals("message"))
           return jsonObj.getString(field);
        else if (field.equals("ok"))
           return new Boolean(jsonObj.getBoolean(field));
        else if (field.equals("solution"))
           return new Integer(jsonObj.getInt(field));

        return null;
    }

    private String jsonizerServiceRequestsForSending(ServerType mode, String id)
    {
          String[] values = new String[3];
          values[0] = id;	

	  return jsonizerServiceRequests(mode, values);
    }

    private String jsonizerServiceRequests(ServerType mode, String[] values)
    {
       String json = "{";
       if(mode == ServerType.PrologQuery)
       {
          json += "\"id\": \"" + values[0] + "\","
               +  "\"mode\": " + values[1] + ","
               +  "\"query\": \"" + values[2] + "\"";
       }
       else if(mode == ServerType.PrologNextSolution || mode == ServerType.PrologFinish)
       {
          json += "\"id\": \"" + values[0] + "\"";
       }
       else return "";

       json += "}";
       return json;
    }

    private String randomStringGenerator()
    {
	return UUID.randomUUID().toString();
    }
    
    public enum ServerType
    {
       PrologQuery, PrologNextSolution, PrologFinish
    }

}
