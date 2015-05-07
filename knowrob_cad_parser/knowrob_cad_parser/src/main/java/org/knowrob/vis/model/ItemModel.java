/*
 * Copyright (c) 2012 Stefan Profanter
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


package org.knowrob.vis.model;

import org.knowrob.vis.model.parser.ModelParser;

/**
 * Holds information about the 3D Model of an ItemBase object.
 * 
 * @author Stefan Profanter
 * 
 */
public class ItemModel {

	/**
	 * Physical file path to the model
	 */
	private String		path;

	/**
	 * The parser for the model.
	 */
	private ModelParser	parser;

	/**
	 * Constructor. The model will immediately be parsed and hold in RAM.
	 * 
	 * @param path
	 *            Path to the model
	 */
	public ItemModel(String path) {
		this.path = path;
		if (!parseModel())
			parser = null;
	}

	/**
	 * Get associated model parser which contains the model for this item
	 * 
	 * @return the model parser
	 */
	public ModelParser getParser() {
		return parser;
	}

	/**
	 * Get physical file path of model
	 * 
	 * @return physical file path
	 */
	public String getPath() {
		return path;
	}

	/**
	 * Selects automagically the correct parser for the file (by file extension) and parses the
	 * model with the selected parser.
	 * 
	 * @return true if successfully parsed, false otherwise
	 */
	public boolean parseModel() {
		if (path == null)
			return false;
		Class<? extends ModelParser> cl = ModelParser.findParser(path);
		if (cl == null) {
			System.out.println("No parser found for: " + path);
			return false;
		}
		try {
			parser = cl.newInstance();
		} catch (InstantiationException e) {
			System.out.println("Couldn't instantiate parser for: " + path);
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			System.out.println("Couldn't instantiate parser for: " + path);
			e.printStackTrace();
		}
		return parser.parseModel(path);
	}

	/**
	 * Set associated model parser which contains the model for this item
	 * 
	 * @param parser
	 *            the model parser
	 */
	public void setParser(ModelParser parser) {
		this.parser = parser;
	}

	/**
	 * Set physical file path of model
	 * 
	 * @param path
	 *            physical file path
	 */
	public void setPath(String path) {
		this.path = path;
	}

}
