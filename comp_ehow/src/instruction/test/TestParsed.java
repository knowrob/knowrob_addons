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
package instruction.test;

import java.io.File;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import instruction.configuration.ConfigurationManager;
import instruction.importer.PlanImporter;
import instruction.opencyc.OpenCyc;
import instruction.wrapper.LocalParseTree;

public class TestParsed {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		ConfigurationManager.loadSettings();

		PlanImporter importer = new PlanImporter();
		try {
			importer.initialize();

			System.out.println("Initializing Plan-Importer...");
			Map<String, List<String>> mappings = ConfigurationManager
					.getMappings();
			Set<String> synsets = mappings.keySet();
			for (Iterator<String> i = synsets.iterator(); i.hasNext();) {
				String synset = i.next();
				List<String> concepts = mappings.get(synset);
				for (Iterator<String> j = concepts.iterator(); j.hasNext();) {
					OpenCyc.getInstance().addMapping(synset, j.next());
				}
			}
			importer.getDisambiguator().load(
					ConfigurationManager.getPathDisambiguator());
			System.out.println("Plan-Importer initialized.");

			int r;
			String s = "";
			do {
				r = System.in.read();
				if (r != (char) '\n' && r != (char) '\r')
					s += (char) r;
			} while (r != (char) '\n');

			String filePath = "D:/ba_workspace/instruction_factory/parsed/"
					+ s.replaceAll(" ", "_");

			if (new File(filePath).exists()) {
				LocalParseTree wrapper = new LocalParseTree();
				wrapper.load(filePath);
				importer.setWrapper(wrapper);
				importer.parseInstructions();
				importer.recognizeAndDisambiguateInstructions();
				importer.convert2CycAssertions();
			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
