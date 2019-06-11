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
 * Contributors: Stefan Profanter - initial API and implementation
*/

package org.knowrob.vis.model.parser;

import java.io.File;
import java.util.HashMap;
import java.util.Iterator;

import com.sun.j3d.utils.geometry.GeometryInfo;

import org.knowrob.vis.model.util.ResourceRetriever;
import org.knowrob.vis.model.Model;
import org.knowrob.vis.model.util.Vertex;

/**
 * Base class for all ModelParsers. Used to parse models from file an draw them onto the Processing
 * applet.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class ModelParser {

	/**
	 * Contains for each valid file extension the appropriate parser. Used to automatically
	 * determine the correct parser for a file.
	 */
	private static final HashMap<String, Class<? extends ModelParser>>	extensionAssignment	= new HashMap<String, Class<? extends ModelParser>>();
	static {
		/*
		 * Add here other parsers with file extension
		 */
		extensionAssignment.put("dae", ColladaParser.class);
		extensionAssignment.put("kmz", ColladaParser.class);
		extensionAssignment.put("mesh", CustomParser.class);
		extensionAssignment.put("ply", PlyParser.class);
	}

	/**
	 * This is a buffer for already parsed models. If a model is loaded through parseModel method it
	 * will be stored in this HashMap. It it is loaded a second time, instead of parsing it again it
	 * will be taken from this HashMap
	 */
	private static HashMap<String, Model>								modelBuffer			= new HashMap<String, Model>();

	/**
	 * Finds the appropriate parser for the given filename by comparing the file extension.
	 * 
	 * @param filename
	 *            Filename to find parser for
	 * @return the Parser Class for this file
	 */
	public static Class<? extends ModelParser> findParser(String filename) {
		return extensionAssignment.get(getExtension(filename));
	}

	/**
	 * Gets the filename extension of a filename. Usually the last 3 characters
	 * 
	 * @param filename
	 *            The filename
	 * @return the extension without dot
	 */
	protected static String getExtension(String filename) {
		return filename.substring(filename.lastIndexOf('.') + 1);
	}

	/**
	 * Contains mesh of the model
	 */
	protected Model	model;

	/**
	 * Checks if the file extension is valid for this class
	 * 
	 * @param filename
	 *            file to check
	 * @return true if extension is valid for this class. false otherwise
	 */
	protected boolean checkExtension(String filename) {
		if (!isValidExtension(filename)) {
			System.out.println("Unknown file extension for class: " + this.getClass().getName()
					+ " " + filename + "\n Must be one of: " + getValidExtensions());
			return false;
		}
		return true;
	}

	/**
	 * Get the object containing the model (meshes)
	 * 
	 * @return the model member
	 */
	public Model getModel() {
		return model;
	}

	/**
	 * Returns a comma separated string with all valid file extensions of this class
	 * 
	 * @return List of file extensions: eg "kmz, dae,"
	 */
	protected String getValidExtensions() {
		Iterator<String> it = extensionAssignment.keySet().iterator();
		String extensions = "";
		while (it.hasNext()) {
			String key = it.next();
			Object val = extensionAssignment.get(key);
			if (val.getClass() == this.getClass())
				extensions += key + ", ";
		}
		return extensions;
	}

	/**
	 * Checks if the given filename has a valid extension for the extended class.
	 * 
	 * @param filename
	 *            Filename to check
	 * @return true if extension is valid
	 */
	protected boolean isValidExtension(String filename) {
		if (extensionAssignment.get(getExtension(filename)) == this.getClass())
			return true;
		return false;
	}

	/**
	 * This function will be called from parseModel if the model isn't in the Buffer
	 * 
	 * @param filename
	 *            Physical file of model to parse
	 * @return true if model parsed successfully. false otherwise
	 */
	protected abstract boolean loadModel(String filename);

	/**
	 * Main function to parse the model from the given filename. If model has already been parsed,
	 * it will be taken from modelBuffer instead of parsing it again.
	 * 
	 * @param filename
	 *            Physical file of model to parse
	 * @return true if model parsed successfully. false otherwise
	 */
	public boolean parseModel(String filename) {
		Model m = modelBuffer.get(filename);
		boolean retVal = true;

		if (m == null) {
			model = new Model();
			// get / download file and determine local file path
			String file = ResourceRetriever.retrieve(filename).getAbsolutePath();

			if (!checkExtension(file)) {
				return false;
			}

			if ((new File(file)).exists() == false) {
				System.err.println("ERROR: Can't load model. File not found: " + file + "\n");
				return false;
			}

			retVal = loadModel(file);
			if (!retVal)
				return false;
			modelBuffer.put(filename, model);
			model.getGroup().initialize(model.getTextureBasePath());
		} else
			model = m;
		return retVal;
	}

	/**
	 * Split polygon into triangles. Build in GeometryInfo class is used. New vertex indices are
	 * then returned.
	 * 
	 * @param vertices
	 *            list of all vertices for polygon
	 * @return list of vertex indices for triangles
	 */
	public static int[] polygonTriangulation(Vertex vertices[]) {
		float coord[] = new float[vertices.length * 3];
		for (int i = 0; i < vertices.length; i++) {
			coord[i * 3] = vertices[i].x;
			coord[i * 3 + 1] = vertices[i].y;
			coord[i * 3 + 2] = vertices[i].z;
		}

		GeometryInfo gi = new GeometryInfo(GeometryInfo.POLYGON_ARRAY);
		gi.setCoordinates(coord);

		int[] stripCounts = { coord.length / 3 };
		gi.setStripCounts(stripCounts);
		gi.convertToIndexedTriangles();

		return gi.getCoordinateIndices();
	}

}
