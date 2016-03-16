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
package instruction.gui.dialog;

import instruction.gui.tab.ConfigurationTab;
import instruction.gui.tab.CycMappingTab;
import instruction.gui.tab.DialogTab;

import javax.swing.JDialog;
import java.awt.Dimension;
import javax.swing.JPanel;
import java.awt.BorderLayout;
import javax.swing.JTabbedPane;
import java.awt.GridBagLayout;
import javax.swing.JButton;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.Insets;

public class ConfigDlg extends JDialog {

	private static final long serialVersionUID = 4020020290843825826L;
	private JPanel jContentPane = null;
	private JPanel jButtonPanel = null;
	private JButton jButtonOK = null;
	private JButton jButtonCancel = null;
	private JTabbedPane jTabbedPane = null;

	/**
	 * This method initializes
	 * 
	 */
	public ConfigDlg () {

		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 */
	private void initialize() {

		this.setSize( new Dimension( 693, 516 ) );
		this.setContentPane( getJContentPane() );
		this.setTitle( "Plan Importer Settings" );

	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {

		if ( jContentPane == null ) {
			BorderLayout borderLayout = new BorderLayout();
			borderLayout.setHgap( 5 );
			borderLayout.setVgap( 5 );
			jContentPane = new JPanel();
			jContentPane.setLayout( borderLayout );
			jContentPane.add( getJButtonPanel(), BorderLayout.SOUTH );
			jContentPane.add( getJTabbedPane(), BorderLayout.CENTER );
		}
		return jContentPane;
	}

	/**
	 * This method initializes jButtonPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJButtonPanel() {

		if ( jButtonPanel == null ) {
			GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
			gridBagConstraints1.gridx = 1;
			gridBagConstraints1.insets = new Insets( 5, 5, 5, 5 );
			gridBagConstraints1.anchor = GridBagConstraints.CENTER;
			gridBagConstraints1.gridy = 0;
			GridBagConstraints gridBagConstraints = new GridBagConstraints();
			gridBagConstraints.gridx = 0;
			gridBagConstraints.insets = new Insets( 5, 5, 5, 5 );
			gridBagConstraints.anchor = GridBagConstraints.CENTER;
			gridBagConstraints.gridy = 0;
			jButtonPanel = new JPanel();
			jButtonPanel.setLayout( new GridBagLayout() );
			jButtonPanel.setPreferredSize( new Dimension( 0, 50 ) );
			jButtonPanel.add( getJButtonOK(), gridBagConstraints );
			jButtonPanel.add( getJButtonCancel(), gridBagConstraints1 );
		}
		return jButtonPanel;
	}

	/**
	 * This method initializes jButtonOK
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButtonOK() {

		if ( jButtonOK == null ) {
			jButtonOK = new JButton();
			jButtonOK.setText( "OK" );
			jButtonOK.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {

					int numTabs = jTabbedPane.getTabCount();
					for ( int i = 0; i < numTabs; i++ ) {
						Component tab = jTabbedPane.getComponentAt( i );
						if ( tab instanceof DialogTab ) 
							( (DialogTab) tab ).onOK();
						
					}
					ConfigDlg.this.setVisible( false );
				}
			} );
		}
		return jButtonOK;
	}

	/**
	 * This method initializes jButtonCancel
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButtonCancel() {

		if ( jButtonCancel == null ) {
			jButtonCancel = new JButton();
			jButtonCancel.setText( "Cancel" );
			jButtonCancel.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {
					ConfigDlg.this.setVisible( false );
				}
			} );
		}
		return jButtonCancel;
	}

	/**
	 * This method initializes jTabbedPane
	 * 
	 * @return javax.swing.JTabbedPane
	 */
	private JTabbedPane getJTabbedPane() {

		if ( jTabbedPane == null ) {
			jTabbedPane = new JTabbedPane();
			jTabbedPane.addTab( "Settings", new ConfigurationTab() );
			jTabbedPane.addTab( "Cyc Mapping", new CycMappingTab() );
		}
		return jTabbedPane;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
