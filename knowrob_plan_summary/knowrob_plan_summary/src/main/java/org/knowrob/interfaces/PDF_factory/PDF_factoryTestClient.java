package org.knowrob.interfaces.PDF_factory;

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
		
		PDF_factory pdf = new PDF_factory();

		pdf.createLatex(objects, walls);

	}

}
