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
 * refactored implementation during Google Summer of Code 2014
*/

package org.knowrob.vis.model.util;

import java.awt.Color;
import java.io.Serializable;

import processing.core.PImage;

/**
 * Appearance of a triangle or a line. May contain texture or simply a color.
 * 
 * @author Stefan Profanter
 * @author Andrei Stoica - added copy constructor for copying appearances
 */
public class Appearance implements Serializable {
	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 3878380145216484383L;

	/**
	 * the Filename of the texture-Image
	 */
	private String				imageFileName;

	/**
	 * an Object reference of a loaded image. Default is null.
	 */
	private PImage				imageReference;

	/**
	 * the color of the Triangle if the Triangle has no texture Set to null if no fill needed
	 */
	private Color				colorFill;

	/**
	 * the color stroke for Triangle or Line Set to null if no stroke needed
	 */
	private Color				colorLine;

	/**
	 * Thickness of the stroke
	 */
	private int					strokeWeight;

	/**
	 * Empty constructor for appearance objects
	 */
	public Appearance() {
		imageFileName = null;
		imageReference = null;
		colorFill = null;
		colorLine = null;
		strokeWeight = 2;
	};

	/**
	 * Copy constructor for the Appearance objects. Performs a deep copy of all the data fields from
	 * the passed object to the one called.
	 * 
	 * @param toCopy
	 *            object to copy to newly created instance
	 */
	public Appearance(final Appearance toCopy) {
		if (toCopy.colorFill != null) {
			this.colorFill = toCopy.colorFill;
		}
		if (toCopy.colorLine != null) {
			this.colorLine = toCopy.colorLine;
		}
		if (toCopy.imageFileName != null) {
			this.imageFileName = toCopy.imageFileName;
		}
		if (toCopy.imageReference != null) {
			this.imageReference = toCopy.imageReference;
		}
		if (toCopy.strokeWeight != 2) {
			this.strokeWeight = toCopy.strokeWeight;
		}
	}

	/**
	 * @return the colorFill
	 */
	public Color getColorFill() {
		return colorFill;
	}

	/**
	 * @return the colorLine
	 */
	public Color getColorLine() {
		return colorLine;
	}

	/**
	 * @return the imageFileName
	 */
	public String getImageFileName() {
		return imageFileName;
	}

	/**
	 * @return the imageReference
	 */
	public PImage getImageReference() {
		return imageReference;
	}

	/**
	 * @return the strokeWeight
	 */
	public int getStrokeWeight() {
		return strokeWeight;
	}

	/**
	 * @param colorFill
	 *            the colorFill to set
	 */
	public void setColorFill(Color colorFill) {
		this.colorFill = colorFill;
	}

	/**
	 * @param colorLine
	 *            the colorLine to set
	 */
	public void setColorLine(Color colorLine) {
		this.colorLine = colorLine;
	}

	/**
	 * @param imageFileName
	 *            the imageFileName to set
	 */
	public void setImageFileName(String imageFileName) {
		this.imageFileName = imageFileName;
	}

	/**
	 * @param imageReference
	 *            the imageReference to set
	 */
	public void setImageReference(PImage imageReference) {
		this.imageReference = imageReference;
	}

	/**
	 * @param strokeWeight
	 *            the strokeWeight to set
	 */
	public void setStrokeWeight(int strokeWeight) {
		this.strokeWeight = strokeWeight;
	}
}
