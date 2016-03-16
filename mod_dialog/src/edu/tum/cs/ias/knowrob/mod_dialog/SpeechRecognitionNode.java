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

package edu.tum.cs.ias.knowrob.mod_dialog;

import edu.cmu.sphinx.jsapi.JSGFGrammar;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import edu.cmu.sphinx.util.props.PropertyException;

import java.io.IOException;
import java.net.URL;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.RuleParse;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;



/**
 * Speech recognition node written by slightly modifying the Sphinx
 * SpeechRecognitionNode.
 * 
 * 
 *
 */
public class SpeechRecognitionNode {
    private Recognizer recognizer;
    private JSGFGrammar jsgfGrammarManager;
    private Microphone microphone;

	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff
	
	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	static Publisher<ros.pkg.std_msgs.msg.String> pub; 

    /**
     * Creates a new SpeechRecognitionNode. 
     *
     * @throws IOException if an I/O error occurs
     * @throws PropertyException if a property configuration occurs
     * @throws InstantiationException if a problem occurs while
     * creating any of the recognizer components.
     */
    public SpeechRecognitionNode() throws 
            IOException, PropertyException, InstantiationException {

        URL url = SpeechRecognitionNode.class.getResource("jsgf.config.xml");
        ConfigurationManager cm = new ConfigurationManager(url);

        // retrive the recognizer, jsgfGrammar and the microphone from
        // the configuration file.

        recognizer = (Recognizer) cm.lookup("recognizer");
        jsgfGrammarManager = (JSGFGrammar) cm.lookup("jsgfGrammar");
        microphone = (Microphone) cm.lookup("microphone");
        
        
        // initialize ROS environment
		try {
			initRos();
			pub = n.advertise("/knowrob/speech_in", new ros.pkg.std_msgs.msg.String(), 100);
			
		} catch (RosException e) {
			e.printStackTrace();	
		}
        
    }

    
    /**
     * Executes the recognition node
     */
    public void execute() throws IOException, GrammarException  {
        System.out.println("JSGF Demo Version 1.0\n");

        System.out.print(" Loading recognizer ...");
        recognizer.allocate();
        System.out.println(" Ready");

        if (microphone.startRecording()) {
            loadAndRecognize("queries");
        } else {
            System.out.println("Can't start the microphone");
        }

        System.out.print("\nDone. Cleaning up ...");
        recognizer.deallocate();

        System.out.println(" Goodbye.\n");
        System.exit(0);
    }

    
	
	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("knowrob_sphinx");
		}
		n = ros.createNodeHandle();

	}
    
    /**
     * Load the grammar with the given grammar name and start
     * recognizing speech using the grammar.  Spoken utterances will
     * be echoed to the terminal.  This method will return when the
     * speaker utters the exit phrase for the grammar. The exit phrase
     * is a phrase in the grammar with the word 'exit' as a tag.
     *
     * @throws IOException if an I/O error occurs
     * @throws GrammarException if a grammar format error is detected
     */
    private void loadAndRecognize(String grammarName) throws
            IOException, GrammarException  {
        jsgfGrammarManager.loadJSGF(grammarName);
        recognizeAndReport();
    }

    /**
     * Performs recognition with the currently loaded grammar.
     * Recognition for potentially multiple utterances until an 'exit'
     * tag is returned.
     *
     * @htrows GrammarException if an error in the JSGF grammar is
     * encountered
     */
    private void recognizeAndReport() throws GrammarException {
        boolean done = false;


        while (!done)  {
            Result result = recognizer.recognize();
            String bestResult = result.getBestFinalResultNoFiller();
            RuleGrammar ruleGrammar = jsgfGrammarManager.getRuleGrammar();
            RuleParse ruleParse = ruleGrammar.parse(bestResult, null);
            if (ruleParse != null) {
                System.out.println("\n  " + sphinxToKnowrob(bestResult) + "\n");

                // publish to ROS topic
                ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
    			m.data = sphinxToKnowrob(bestResult);
    			pub.publish(m);

                done = isExit(ruleParse);
            } 
        }
    }

    /**
     * Map a string to the KnowRob syntax (e.g. replace number words by their numeric values)
     * @param q The string coming from the speech recognition
     * @return String modified to be used for the DialogModule
     */
    String sphinxToKnowrob(String q) {
    	
    	q=q.replaceAll(" one",   "1");
    	q=q.replaceAll(" two",   "2");
    	q=q.replaceAll(" three", "3");
    	q=q.replaceAll(" four",  "4");
    	q=q.replaceAll(" five",  "5");
    	q=q.replaceAll(" six",   "6");
    	q=q.replaceAll(" seven", "7");
    	q=q.replaceAll(" eight", "8");
    	q=q.replaceAll(" nine",  "9");
    	q=q.replaceAll(" zero",  "0");
    	
    	return q + "?";
    }
    
    /**
     * Searches through the tags of the rule parse for an 'exit' tag.
     *
     * @return true if an 'exit' tag is found
     */
    private boolean isExit(RuleParse ruleParse) {
        String[] tags = ruleParse.getTags();

        for (int i = 0; tags != null && i < tags.length; i++) {
            if (tags[i].trim().equals("exit")) {
                return true;
            }
        }
        return  false;
    }

    
    /**
     * Main method for running the jsgf demo.
     * @param args program arguments (none)
     */
    public static void main(String[] args) {
        try {
            SpeechRecognitionNode SpeechRecognitionNode = new SpeechRecognitionNode();
            SpeechRecognitionNode.execute();
        } catch (IOException ioe) {
            System.out.println("I/O Error " + ioe);
        } catch (PropertyException e) {
            System.out.println("Problem configuring recognizer" + e);
        } catch (InstantiationException  e) {
            System.out.println("Problem creating components " + e);
        } catch (GrammarException  e) {
            System.out.println("Problem with Grammar " + e);
        }
    }
}
