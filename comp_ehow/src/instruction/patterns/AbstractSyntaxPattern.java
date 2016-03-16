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
package instruction.patterns;

import java.util.ArrayList;

import instruction.semanticObjects.SemanticObject;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;

/**
 * Abstract super-class for Syntax Tree Patterns
 * @author daniel
 *
 */
public abstract class AbstractSyntaxPattern {
	
	SyntaxTree tree;
	String pattern;
	
	/**
	 * Creates the Syntax Pattern
	 * @param p The Pattern String
	 */
	public AbstractSyntaxPattern(String p)  {
		pattern = p;
	}
	
	/**
	 * Initializes the Pattern by parsing the Pattern String
	 * into a Syntax Tree.
	 * @param p an initialized Parser Object
	 */
	public final void init(Parser p) {
		ArrayList<String> tokens = p.tokenize(pattern);
		tree = p.generateTree(null, tokens);
	}
	
	/**
	 * Returns the parsed Syntax Tree.
	 * @return
	 */
	public final SyntaxTree getPattern() {
		return tree;
	}
	
	public final String getPatternString() {
		return pattern;
	}
	
	/**
	 * Transforms a Set of Semantic Objects into a new set of Semantic Objects
	 * according to the syntactic structure of the Pattern
	 * @return
	 */
	public abstract ArrayList<SemanticObject> doModelTransformation(ArrayList<SemanticObject> obj);
}
