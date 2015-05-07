/*
 * Copyright (c) 2013 Stefan Profanter
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
*/

package org.knowrob.vis.model.util;

import java.awt.Color;

/**
 * General settings used for drawing triangles, lines and vertices. This class is used to override
 * the default appearance of a triangle.
 * 
 * @author Stefan Profanter
 * 
 */
public class DrawSettings {
	/**
	 * Default constructor. Initializes DrawSettings with default values which don't override the
	 * appearance.
	 */
	public DrawSettings() {
		this.drawType = DrawType.FILL;
		overrideColor = null;
	}

	/**
	 * Override color. If set, it overrides the appearance color.
	 */
	private Color	overrideColor;
	/**
	 * Type of drawing (only for triangles): filled, lines, points
	 */
	public DrawType	drawType;
	/**
	 * Line width for draw type lines
	 */
	private int		lineWidth	= 1;
	/**
	 * Forces the annotation or triangle to draw, even if it is unselected from the annotation list.
	 */
	public boolean	forceDraw	= false;

	/**
	 * Increase the line width for draw type lines.
	 */
	public void incLineWidth() {
		lineWidth++;
	}

	/**
	 * Decrease the line width for draw type lines (minimum is 1).
	 */
	public void decLineWidth() {
		if (lineWidth == 1)
			return;
		lineWidth--;
	}

	/**
	 * Returns the current line width.
	 * 
	 * @return current line width.
	 */
	public int getLineWidth() {
		return lineWidth;
	}

	/**
	 * @param overrideColor
	 *            the overrideColor to set
	 */
	public void setOverrideColor(Color overrideColor) {
		this.overrideColor = overrideColor;
	}

	/**
	 * @param lineWidth
	 *            the lineWidth to set
	 */
	public void setLineWidth(int lineWidth) {
		this.lineWidth = lineWidth;
	}

	@Override
	public Object clone() {
		DrawSettings ds = new DrawSettings();
		ds.overrideColor = overrideColor == null ? null : new Color(overrideColor.getRed(),
				overrideColor.getGreen(), overrideColor.getBlue(), overrideColor.getAlpha());
		ds.drawType = drawType;
		ds.lineWidth = lineWidth;
		ds.forceDraw = forceDraw;
		return ds;

	}

	/**
	 * Clones the calling draw settings object and sets the override color of the cloned object to
	 * the given one. The cloned object is then returned.
	 * 
	 * @param overrideColor
	 *            The new override color for the cloned object.
	 * @return A cloned copy of this, where override color is set to the provided one.
	 */
	public DrawSettings getTemporaryOverride(Color overrideColor) {
		DrawSettings ds = (DrawSettings) this.clone();
		ds.overrideColor = overrideColor;
		return ds;
	}

	/**
	 * @return the overrideColor
	 */
	public Color getOverrideColor() {
		return overrideColor;
	}

}
