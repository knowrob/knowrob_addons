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
import instruction.syntaxparser.SyntaxTree;

import java.awt.BorderLayout;
import java.awt.ScrollPane;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.util.List;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;


public class TreeViewTab extends JPanel {

	private static final long serialVersionUID = 5838455112176213755L;

	public static String TITLE = "Syntax Tree View";

	ScrollPane scroll = null;
	SyntaxTreePanel treeView = null;
	JPanel buttonPanel = null;
	JButton next = null;
	JButton prev = null;
	JLabel instCount = null;

	public TreeViewTab () {

		initialize();
	}

	public void initialize() {

		setLayout( new BorderLayout() );

		treeView = new SyntaxTreePanel();
		treeView.init();
		
		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {
			}

			public void componentMoved( ComponentEvent e ) {
			}

			public void componentResized( ComponentEvent e ) {
			}
			

			public void componentShown( ComponentEvent e ) {

				List<SyntaxTree> trees = PlanImporterWrapper.getImporter().getSyntaxTrees();
				
				if (trees == null || trees.isEmpty())
					return;
				
			
				treeView.setSyntaxTree( trees );
				treeView.setActiveTree( 0 );
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1) + " of " + treeView.getTreeCount() );

			}

		} );

		scroll = new ScrollPane(  );
		
		scroll.add( treeView );
		
		buttonPanel = new JPanel();
		prev = new JButton("< Prev");
		prev.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				treeView.setActiveTree( (treeView.getActiveTree() - 1 < 0 ? treeView.getTreeCount() - 1 : treeView.getActiveTree() - 1) % treeView.getTreeCount() );
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1) + " of " + treeView.getTreeCount());
			}
			
		});
		next = new JButton("Next >");
		next.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				treeView.setActiveTree( (treeView.getActiveTree() + 1) % treeView.getTreeCount());
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1)  + " of " + treeView.getTreeCount());
			}
			
		});
		
		instCount = new JLabel("");
		
		buttonPanel.add( prev );
		buttonPanel.add( instCount );
		buttonPanel.add( next );
		
		add(buttonPanel, BorderLayout.NORTH);
		
		add( scroll, BorderLayout.CENTER );
	}

}
