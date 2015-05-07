/*
 * Copyright (c) 2012-14 Stefan Profanter, Andrei Stoica
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
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012; Andrei Stoica -
 * minor refactor during the Google Summer of Code 2014
*/

package org.knowrob.vis.model.parser;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.InputStreamReader;

import org.knowrob.vis.model.util.Group;
import org.knowrob.vis.model.util.Mesh;
import org.knowrob.vis.model.util.Triangle;
import org.knowrob.vis.model.util.Vertex;

/**
 * 
 * Parser for simple file format, where first all vertices are listed and after <tt>#</tt> 3 values
 * per line for each triangle indicating the vertex index.
 * 
 * Example:
 * 
 * <pre>
 * 0.5	0.4	0.5
 * 0.4	0.3	1.3
 * ....
 * #
 * 0	3	4
 * 0	1	2
 * 1	2	4
 * ....
 * </pre>
 * 
 * @author Stefan Profanter
 * @author Andrei Stoica - added edges update after parsing the triangles
 */
public class CustomParser extends ModelParser {

	/* (non-Javadoc)
	 * @see org.knowrob.vis.model.parser.ModelParser#loadModel(java.lang.String)
	 */
	@Override
	protected boolean loadModel(String filename) {
		try {
			// Open the file that is the first
			// command line parameter
			FileInputStream fstream = new FileInputStream(filename);
			// Get the object of DataInputStream
			DataInputStream in = new DataInputStream(fstream);
			@SuppressWarnings("resource")
			// necessary to close resources at the outer layer only
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;

			Group g = new Group(model);
			model.setGroup(g);
			Mesh m = new Mesh();
			g.setMesh(m);

			boolean isTriangles = false;
			// Read File Line By Line
			while ((strLine = br.readLine()) != null) {
				if (strLine.compareTo("#") == 0)
					isTriangles = true;
				else {
					String[] parts = strLine.split("\t");

					if (!isTriangles) {
						if (parts.length != 5)
							throw new RuntimeException("Vertex: invalid number of data: "
									+ parts.length);

						float x = Float.valueOf(parts[0]);
						float y = Float.valueOf(parts[1]);
						float z = Float.valueOf(parts[2]);

						model.getVertices().add(new Vertex(x, y, z));
					} else {
						if (parts.length != 3)
							throw new RuntimeException("Triangle: invalid number of data: "
									+ parts.length);
						int p1 = Integer.valueOf(parts[0]);
						int p2 = Integer.valueOf(parts[1]);
						int p3 = Integer.valueOf(parts[2]);

						Triangle t = new Triangle();
						t.getPosition()[0] = model.getVertices().get(p1);
						t.getPosition()[1] = model.getVertices().get(p2);
						t.getPosition()[2] = model.getVertices().get(p3);

						t.updateCentroid();
						t.updateEdges();
						m.getTriangles().add(t);
					}
				}

			}
			// Close the input stream
			in.close();

		} catch (Exception e) {// Catch exception if any
			System.err.println("Error: " + e.getMessage());
			return false;
		}

		return true;

	}
}
