/*
 * Copyright (c) 2015 Asil Kaan Bozcuoglu
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

package org.knowrob.summary;

import java.io.*;
import java.util.StringTokenizer;
import java.lang.Double;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class PDF_factoryTestClient  extends AbstractNodeMain 
{
	ConnectedNode node;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("PDF_factory_test_client");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		/*String[] objects = new String[3];

		double[] x_coord = new double[3];
		double[] y_coord = new double[3];
		double[] width = new double[3];
		double[] height = new double[3];

		double[] x_wall = new double[6];
		double[] y_wall = new double[6];

		objects[0] = "table";
		objects[1] = "door";
		objects[2] = "chair";

		x_coord[0] = 1;
		x_coord[1] = 2;
		x_coord[2] = 3;

		y_coord[0] = 4;
		y_coord[1] = 5;
		y_coord[2] = 6;

	
		width[0] = 1;
		width[1] = 2;
		width[2] = 3;

		height[0] = 4;
		height[1] = 5;
		height[2] = 6;		
		
		x_wall[0] = 1;
		x_wall[1] = 2;
		x_wall[2] = 3;
		x_wall[3] = 4;
		x_wall[4] = 5;
		x_wall[5] = 6;

		y_wall[0] = 1;
		y_wall[1] = 2;
		y_wall[2] = 3;
		y_wall[3] = 4;
		y_wall[4] = 5;
		y_wall[5] = 6;*/

		String[][] objects = new String[3][6];
		objects[0][0] = "1#table";
		objects[1][0] = "1#door";
		objects[2][0] = "1#chair";
		
		objects[0][4] = "0'1'2'1";
		objects[1][4] = "0'1'2'2";
		objects[2][4] = "0'1'2'3";

		objects[0][5] = "0'1'2'4";
		objects[1][5] = "0'1'2'5";
		objects[2][5] = "0'1'2'6";

		objects[0][1] = "0'1'2'1";
		objects[1][1] = "0'1'2'2";
		objects[2][1] = "0'1'2'3";

		objects[0][2] = "0'1'2'4";
		objects[1][2] = "0'1'2'5";
		objects[2][2] = "0'1'2'6";

		objects[0][3] = "0'1'2'4";
		objects[1][3] = "0'1'2'5";
		objects[2][3] = "0'1'2'6";		
		

		String[][] walls = new String[6][4];
		walls[0][0] = "0'1'2'1";
		walls[1][0] = "0'1'2'2";
		walls[2][0] = "0'1'2'3";
		walls[3][0] = "0'1'2'4";
		walls[4][0] = "0'1'2'5";
		walls[5][0] = "0'1'2'6";

		walls[0][1] = "0'1'2'1";
		walls[1][1] = "0'1'2'2";
		walls[2][1] = "0'1'2'3";
		walls[3][1] = "0'1'2'4";
		walls[4][1] = "0'1'2'5";
		walls[5][1] = "0'1'2'6";

		walls[0][2] = "0'1'2'1";
		walls[1][2] = "0'1'2'2";
		walls[2][2] = "0'1'2'3";
		walls[3][2] = "0'1'2'4";
		walls[4][2] = "0'1'2'5";
		walls[5][2] = "0'1'2'6";

		walls[0][3] = "0'1'2'1";
		walls[1][3] = "0'1'2'2";
		walls[2][3] = "0'1'2'3";
		walls[3][3] = "0'1'2'4";
		walls[4][3] = "0'1'2'5";
		walls[5][3] = "0'1'2'6";
		
		PDFFactory pdf = new PDFFactory();

		pdf.createLatex(objects, walls);

	}

}
