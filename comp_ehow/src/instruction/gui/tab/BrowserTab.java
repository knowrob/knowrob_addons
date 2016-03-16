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
import instruction.wrapper.IHowtoWebsiteWrapper;

import java.awt.BorderLayout;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.io.File;
import javax.swing.JEditorPane;
import javax.swing.JScrollPane;

public class BrowserTab extends InstructionTab {

	private static final long serialVersionUID = 2094236267788921978L;

	public static String TITLE = "HTML View";

	private static String htmlContent = "<html><head></head><body><table width=\"100%\"><tr><td align=\"center\"><img src=\"file:///IMGSRC\"></td></tr></table></body></html>";
	private static String htmlContentError = "<html><head></head><body><table><tr><td align=\"center\">No Screenshot found for this Howto.</td></tr></table></body></html>";

	JEditorPane browser = null;
	JScrollPane scroll = null;

	public BrowserTab () {

		init();
	}

	@Override
	public void init() {

		setLayout( new BorderLayout() );

		browser = new JEditorPane();
		browser.setContentType( "text/html" );
		browser.setEditable( false );

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent arg0 ) {

			}

			public void componentMoved( ComponentEvent arg0 ) {

			}

			public void componentResized( ComponentEvent arg0 ) {

			}

			public void componentShown( ComponentEvent arg0 ) {

				IHowtoWebsiteWrapper wrapper = PlanImporterWrapper.getImporter().getWrapper();

				if ( wrapper == null )
					browser.setText( htmlContentError );
				else {
					String url = wrapper.getUrl();

					if ( url != null ) {

						File dummy = new File( url );
						if ( dummy.exists() ) {
							String screenshotFile = dummy.getAbsolutePath().replaceAll( "\\\\", "/" );
							screenshotFile += ".jpg";

							File screenshotDummy = new File( screenshotFile );
							if ( screenshotDummy.exists() ) {
					//			System.out.println( htmlContent.replaceAll( "IMGSRC", screenshotFile ) );
								browser.setText( htmlContent.replaceAll( "IMGSRC", screenshotFile ) );
							}
							else {
								browser.setText( htmlContentError );
							}
						}
					}

					else
						browser.setText( htmlContentError );
				}
			}

		} );

		scroll = new JScrollPane( browser );
		add( scroll, BorderLayout.CENTER );
	}
}
