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
package instruction.syntaxparser;

import instruction.syntaxparser.SyntaxElement;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

public class SyntaxTree {

	SyntaxElement element;

	SyntaxTree parent;
	ArrayList<SyntaxTree> children = new ArrayList<SyntaxTree>();

	// Internal fields
	private int tree_depth;
	private int curr_depth;

	public ArrayList<SyntaxTree> getChildren() {

		return children;
	}

	public SyntaxElement getElement() {

		return element;
	}

	public void setElement( SyntaxElement element ) {

		this.element = element;
	}

	public void addChild( SyntaxTree child ) {

		children.add( child );
	}

	public void setParent( SyntaxTree parent ) {

		this.parent = parent;
	}

	public void printTree() {

		printTreeInternal( 0, this );
	}

	public SyntaxTree getParent() {

		return parent;
	}

	public ArrayList<SyntaxTree> getSiblings() {

		if ( parent == null ) {
			ArrayList<SyntaxTree> tmp = new ArrayList<SyntaxTree>();
			tmp.add( this );
			return tmp;
		}
		else
			return parent.getChildren();
	}

	private void _getdepth( SyntaxTree node ) {

		if ( node != null ) {
			curr_depth++;
			if ( curr_depth > tree_depth )
				tree_depth = curr_depth;
			for ( Iterator<SyntaxTree> i = node.getChildren().iterator(); i.hasNext(); )
				_getdepth( i.next() );
			curr_depth--;
		}
	}

	public int getdepth( ) {

		tree_depth = 0;
		_getdepth( this );
		return tree_depth;
	}

	private void printTreeInternal( int level, SyntaxTree tree ) {

		for ( int i = 0; i < level; i++ )
			System.out.print( "\t" );

	//	if ( tree.getElement() != null )
	//		System.out.println( "(" + tree.getElement().getType() + ", " + tree.getElement().getName() + ")" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			printTreeInternal( level + 1, (SyntaxTree) i.next() );
		}
	}

	public String toString() {

		StringBuilder str = new StringBuilder();
		toStringInternal( 0, this, str );
		return str.toString();
	}

	private void toStringInternal( int level, SyntaxTree tree, StringBuilder str ) {

		for ( int i = 0; i < level; i++ )
			str.append( "\t" );

		if ( tree.getElement() != null )
			str.append( "(" + tree.getElement().getType() + ", " + tree.getElement().getName() + ")" + "\n" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			toStringInternal( level + 1, (SyntaxTree) i.next(), str );
		}
	}

	public void writeToFile( FileWriter writer ) throws IOException {

		// writer.append("<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>");
		writeToFileInternal( 0, this, writer );
	}

	private void writeToFileInternal( int level, SyntaxTree tree, FileWriter writer ) throws IOException {

		for ( int i = 0; i < level; i++ )
			writer.append( "\t" );

		if ( tree.getElement() != null )
			writer.append( "<" + tree.getElement().getType() + " " + tree.getElement().getName() + ">\n" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			writeToFileInternal( level + 1, (SyntaxTree) i.next(), writer );
		}

		// writer.append("<" + tree.getElement().getType());
	}

	public String buildSentence() {

		StringBuffer str = new StringBuffer();
		buildSentenceInternal( str );
		return str.toString();
	}

	private void buildSentenceInternal( StringBuffer str ) {

		if ( getElement() != null && ! getElement().getName().equals( "" ) )
			str.append( getElement().getName() + " " );

		if ( getChildren().size() > 0 ) {
			for ( Iterator i = getChildren().iterator(); i.hasNext(); ) {
				SyntaxTree t = (SyntaxTree) i.next();
				t.buildSentenceInternal( str );
			}
		}
	}

}
