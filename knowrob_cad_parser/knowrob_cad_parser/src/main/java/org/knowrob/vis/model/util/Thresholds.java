/*
 * Copyright (c) 2014 Andrei Stoica
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
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
 * Contributors: Andrei Stoica - initial API and implementation during Google Summer of Code 2014
*/

package org.knowrob.vis.model.util;

/**
 * Class that implements some parameters used in the CAD mesh parsing and initialization of
 * components and their neighboring relations according to the model geometry.
 * 
 * @author Andrei Stoica
 */
public class Thresholds {
	/**
	 * Maximum distance tolerance under the Euclidean geometry for two 3D points to be considered
	 * one and the same. This parameter is used in analyzing the neighboring relations among the
	 * triangles that form the faces of the mesh.
	 */
	public static final float	DISTANCE_TOLERANCE		= 1e-5f;

	/**
	 * Maximum angle tolerance (in degrees) for deciding whether or not two lines are parallel which
	 * is applied to the acute angle formed by the directions of the two lines.
	 */
	public static final float	ANGLE_TOLERANCE			= 0.25f;

	/**
	 * Flag for using a fast direct neighbor detection based only on checking if exactly two
	 * vertices of two triangles have exactly the same coordinates. This method is quick but not
	 * very efficient for distorted irregular meshes like the ones scanned from real life objects.
	 * The alternative is to use a slow but very accurate method which checks whether two triangle
	 * share only one support line for one each of each of them, such that partial overlays are also
	 * detected.
	 */
	public static final boolean	FAST_NEIGHBOR_DETECTION	= false;
}