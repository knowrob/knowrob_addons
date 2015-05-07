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

import java.util.HashMap;
import java.util.Vector;

import org.knowrob.prolog.PrologInterface;

/**
 * Static class to obtain model properties such as path to model file, width, height, depth.
 * 
 * 
 * @author Stefan Profanter
 * @see ItemModel
 */
public class Properties {

	/**
	 * Instantiate an ItemModel from prolog identifier of Instance or Class. The path to the model
	 * will be gathered with prolog an this model will then be parsed.
	 * 
	 * @param identifier
	 *            Class or Instance identifier
	 * @return ItemModel instance if model parsed successfully. null otherwise.
	 */
	public static ItemModel getModelOfItem(String identifier) {
		String path = null;
		try {
			if (!identifier.startsWith("'") || !identifier.endsWith("'")) {
				identifier = "'" + identifier + "'";
			}
			HashMap<String, Vector<String>> nfo = PrologInterface.executeQuery("get_model_path("+ identifier + ",P)");

			if (nfo!=null && nfo.get("P") != null && nfo.get("P").size() > 0) {
				String str = nfo.get("P").get(0);

				if (str == null)
					return null;

				if (str.startsWith("'") && str.endsWith("'")) {
					str = str.substring(1, str.length() - 1);
				}
				path = str;

			}

		} catch (Exception e) {
			System.err.println("Calling get_model_path for Identifier " + identifier + " failed:");
			e.printStackTrace();
			return null;
		}

		if (path == null)
			return null;
		return new ItemModel(path);
	}

	/**
	 * Get x-Dimension (width) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return x-Dimension or -1 if model not found
	 */
	public static float xDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalWidth();
	}

	/**
	 * Get y-Dimension (depth) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return y-Dimension or -1 if model not found
	 */
	public static float yDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalDepth();
	}

	/**
	 * Get z-Dimension (height) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return z-Dimension or -1 if model not found
	 */
	public static float zDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalHeight();
	}

}
