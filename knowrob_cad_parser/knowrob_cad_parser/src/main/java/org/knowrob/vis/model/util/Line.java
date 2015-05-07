/*
 * Copyright (c) 2012 Stefan Profanter
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
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
*/
package org.knowrob.vis.model.util;

import processing.core.PGraphics;

/**
 * A simple line of a CAD model which connects two vertices and has a specific color/appearance.
 * 
 * @author Stefan Profanter
 * 
 */
public class Line extends DrawObject {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 6189622478733284349L;

	/**
	 * Default constructor
	 */
	public Line() {
		super(2);
	}

	/**
	 * Draw the line onto the applet.
	 * 
	 * @param g
	 *            Applet to draw on
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		applyColor(g, drawSettings);
		if (drawSettings != null && drawSettings.drawType == DrawType.POINTS) {
			for (int i = 0; i < position.length; i++) {
				if (position[i].overrideColor != null) {
					g.stroke(position[i].overrideColor.getRed(),
							position[i].overrideColor.getGreen(),
							position[i].overrideColor.getBlue());
					g.noFill();
				} else if (drawSettings.getOverrideColor() == null && position[i].color != null) {
					g.stroke(position[i].color.getRed(), position[i].color.getGreen(),
							position[i].color.getBlue());
					g.noFill();
				}
				g.point(position[i].x, position[i].y, position[i].z);
			}
		} else
			g.line(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y,
					position[1].z);
	}
}
