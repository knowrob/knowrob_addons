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
package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;
import java.awt.BorderLayout;
import java.awt.ScrollPane;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextPane;

public class InternalViewTab extends InstructionTab {

	private static final long serialVersionUID = 3245495709892820570L;

	public static String TITLE = "Internal Data Structures";
	
	ScrollPane scroll = null;
	DataStructurePanel dsView = null;
	JTextPane text = null;
	JPanel buttonPanel = null;
	JButton next = null;
	JButton prev = null;
	JLabel instCount = null;

	public InternalViewTab() {
		initialize();
	}
	
	public void initialize() {

		setLayout( new BorderLayout() );

		dsView = new DataStructurePanel();
		dsView.init();
	//	text = new JTextPane();
		
	

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentMoved( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentResized( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentShown( ComponentEvent e ) {

				dsView.setInstructions( PlanImporterWrapper.getImporter().getInstructions() );
				dsView.setActiveInstruction( 0 );
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1) + " of " + dsView.getInstructionCount());
				
//				text.setText( "" );
//				List<Instruction> in = PlanImporterWrapper.getImporter().getInstructions();
//				for (Iterator<Instruction> i = in.iterator(); i.hasNext();) {
//					text.setText( text.getText() + "\n\n" + i.next().toString() );
//				}
			}

		} );
		
		buttonPanel = new JPanel();
		prev = new JButton("< Prev");
		prev.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				dsView.setActiveInstruction( (dsView.getActiveInstruction() - 1 < 0 ? dsView.getInstructionCount() - 1 : dsView.getActiveInstruction() - 1) % dsView.getInstructionCount() );
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1) + " of " + dsView.getInstructionCount());
			}
			
		});
		next = new JButton("Next >");
		next.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				dsView.setActiveInstruction( (dsView.getActiveInstruction() + 1) % dsView.getInstructionCount());
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1)  + " of " + dsView.getInstructionCount());
			}
			
		});
		
		instCount = new JLabel("");
		
		buttonPanel.add( prev );
		buttonPanel.add( instCount );
		buttonPanel.add( next );
		
		add(buttonPanel, BorderLayout.NORTH);

		scroll = new ScrollPane(  );
		scroll.add( dsView );
		
		add( scroll, BorderLayout.CENTER );

	}
	
}
