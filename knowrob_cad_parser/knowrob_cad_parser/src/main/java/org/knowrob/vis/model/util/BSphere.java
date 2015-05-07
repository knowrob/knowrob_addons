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
package org.knowrob.vis.model.util;

import javax.vecmath.Vector3f;

/**
 * Class which represents a bounding sphere by center and radius.
 * 
 * @author Stefan Profanter
 * 
 */
public class BSphere {
	/**
	 * radius
	 */
	private final float		r;
	/**
	 * center
	 */
	private final Vector3f	center;

	/**
	 * Create new bounding sphere with radius <tt>r</tt> and center <tt>center</tt>
	 * 
	 * @param r
	 *            radius of bounding sphere
	 * @param center
	 *            center of bounding sphere
	 */
	public BSphere(float r, Vector3f center) {
		this.r = r;
		this.center = center;
	}

	/**
	 * Get center of bounding sphere
	 * 
	 * @return the center
	 */
	public Vector3f getCenter() {
		return (Vector3f) center.clone();
	}

	/**
	 * Get radius of bounding sphere.
	 * 
	 * @return the radius
	 */
	public float getR() {
		return r;
	}

	/**
	 * Get area of bounding sphere.
	 * 
	 * @return area
	 */
	public float getArea() {

		return (float) (4f * Math.PI * (r * r));
	}

	/**
	 * Get volume of bounding sphere
	 * 
	 * @return volume
	 */
	public float getVolume() {

		return (float) ((4 / 3 * Math.PI) * r * r * r);
	}
}
