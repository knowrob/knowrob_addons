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
package instruction.gui.internal;

import java.util.List;
import javax.swing.JPanel;
import instruction.disambiguator.IDisambiguationOracle;
import instruction.gui.dialog.ConceptSelectionDlg;
import instruction.semanticObjects.Instruction;

public class DisambiguationOracle implements IDisambiguationOracle {

	private JPanel parent = null;
	
	public DisambiguationOracle(JPanel parent) {
		this.parent = parent;
	}
	
	public String retrieveMeaningOfWord(String word, List<String> meanings,
			Instruction instruction) {
		
		ConceptSelectionDlg dlg = new ConceptSelectionDlg(null);
		
		dlg.getTextInstruction().setText("<html><p>" + instruction.getNLSentence().replaceAll(word, "<u>" + word + "</u>") + "</p></html>");
	//	dlg.getLabelWord().setText("\"" + word + "\"");
		dlg.getConceptList().setListData(meanings.toArray(new String[0]));
		dlg.setLocationRelativeTo( parent );
		dlg.setModal(true);
		
		dlg.setVisible(true);
		
		if (dlg.getConceptList().getSelectedValue() == null)
			return null;
		else
			return (String) dlg.getConceptList().getSelectedValue();
	}

}
