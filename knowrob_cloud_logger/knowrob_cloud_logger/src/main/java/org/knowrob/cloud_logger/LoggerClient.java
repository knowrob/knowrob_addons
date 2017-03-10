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

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.openease.client.BridgeClient;
import org.openease.client.EASEError;


public class LoggerClient {

    BridgeClient oeClient;
    String oeAddress;
    String pathToPOM;


    public LoggerClient(String token, String address, String path) throws InterruptedException, EASEError, IOException 
    {
        oeAddress = address;
        pathToPOM = path;

        oeClient = new BridgeClient(token, oeAddress);
        oeClient.setSSLCertificate(Files.newInputStream(Paths.get(pathToPOM)));
        oeClient.startContainer();
        oeClient.connect();
    }

    public LoggerClient(BridgeClient client, String address, String path) throws InterruptedException, EASEError, IOException 
    {
        oeAddress = address;
        pathToPOM = path;

        oeClient = client;
       
        if(!(oeClient.getHost().equals(address)))
           throw new EASEError("Host of BridgeClient and address given as parameter should be equal!");

        oeClient.setSSLCertificate(Files.newInputStream(Paths.get(pathToPOM)));
        oeClient.startContainer();
        oeClient.connect();
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

    /*public static void main(String[] args) throws InterruptedException, EASEError, IOException {
        BridgeClient client = new BridgeClient("E84O1GRNURm0yHsqnSRInjUVXzf58Pz6WIEiMhoSuuoLvtetUzj2idiIcHuKqACf",
                "https://localhost");
        client.setSSLCertificate(Files.newInputStream(Paths.get("/home/moritz/localhost.pem")));
        client.startContainer();
        client.connect();
    }*/
}
