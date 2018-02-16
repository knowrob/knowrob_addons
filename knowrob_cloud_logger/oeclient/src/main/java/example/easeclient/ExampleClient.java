package example.easeclient;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.openease.client.BridgeClient;
import org.openease.client.EASEError;

/**
 * This is an example for the usage of the API client for open-ease.org
 * 
 * @author Moritz Horstmann <mhorst@cs.uni-bremen.de>
 *
 */
public class ExampleClient {
    public static void main(String[] args) throws InterruptedException, EASEError, IOException {
        BridgeClient client = new BridgeClient("E84O1GRNURm0yHsqnSRInjUVXzf58Pz6WIEiMhoSuuoLvtetUzj2idiIcHuKqACf",
                "https://localhost");
        client.setSSLCertificate(Files.newInputStream(Paths.get("/home/moritz/localhost.pem")));
        client.startContainer();
        client.connect();
    }
}
