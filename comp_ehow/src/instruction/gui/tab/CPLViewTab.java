package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;
import java.awt.BorderLayout;
import java.awt.Font;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import javax.swing.BorderFactory;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;

public class CPLViewTab extends InstructionTab {

	private static final long serialVersionUID = -7985157346179531359L;

	public static String TITLE = "CPL Plan";

	JScrollPane scroll = null;
	JTextArea text = null;

	public CPLViewTab () {

		initialize();
	}

	public void initialize() {

		setLayout( new BorderLayout() );

		text = new JTextArea();
		text.setEditable( false );
		text.setBorder( BorderFactory.createLoweredBevelBorder() );
		text.setFont( new Font( "DialogInput", Font.PLAIN, 14 ) );
		text.setTabSize( 4 );

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {
			}

			public void componentMoved( ComponentEvent e ) {
			}

			public void componentResized( ComponentEvent e ) {
			}

			public void componentShown( ComponentEvent e ) {

				String rpl = PlanImporterWrapper.getImporter().getCPLPlan();

				if ( rpl == null )
					text.setText( "No CPL Plan available" );
				else
					text.setText( PlanImporterWrapper.getImporter().getCPLPlan() );

				SwingUtilities.invokeLater( new Runnable() {
					public void run() {

						scroll.getVerticalScrollBar().setValue( 0 );
					}
				} );
			}

		} );

		scroll = new JScrollPane( text );
		add( scroll, BorderLayout.CENTER );
	}

}
