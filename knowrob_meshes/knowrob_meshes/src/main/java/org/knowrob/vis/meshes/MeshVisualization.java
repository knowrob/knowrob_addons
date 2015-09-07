/*
 * Copyright (c) 2013-15 Moritz Tenorth, Arne Stefes, Daniel Beßler, Zhou Fang
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

package org.knowrob.vis.meshes;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;
import java.util.List;
import java.text.DecimalFormat;

import javax.vecmath.Vector3d;

import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.owl.OWLThing;
import org.knowrob.prolog.PrologInterface;
import org.knowrob.tfmemory.TFMemory;
import org.knowrob.vis.MarkerVisualization;

import tfjava.StampedTransform;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;
import geometry_msgs.Pose;

public class MeshVisualization {

	public static void addDesignatorContourMesh(String markerId, Designator designator, String timepoint, String colorStr[]) {
		try {
			Vector3d color = new Vector3d(
				Double.valueOf(colorStr[0]),
				Double.valueOf(colorStr[1]),
				Double.valueOf(colorStr[2])
			);
			List<Vector3d> contourPointsCamRel = ContourMesh.getContourPoints(designator);
			List<Vector3d> contourPoints = new LinkedList<Vector3d>();
			
			// FIXME: Hacky method for getting time since TF data recorded that way for pizza rolling
			double posix_ts = MarkerVisualization.get().parseTime_d(timepoint)*1000000000;
			Time time = new Time();
			time.secs = (int)posix_ts;
			time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

			// FIXME: use parameter instead
			String sourceFrame = "/head_mount_kinect2_rgb_optical_frame";
			System.err.println("Displaying contour mesh for timepoint: " + timepoint);
                        System.err.println(time);
			
			// transform to /map
			for(Vector3d camP : contourPointsCamRel) {
				StampedTransform tr = TFMemory.getInstance().lookupTransform(sourceFrame, MarkerVisualization.getReferenceFrame(), time);
				if(tr==null) {
					System.out.println("TF data missing for '" + sourceFrame + "' " + timepoint + " missing in mongo.");
					return;
				}
				Vector3d p_out = new Vector3d();
				double[] middleTransform = new double[3];
				 
				tr.transformVector(camP, p_out);
				p_out.get(middleTransform);
				middleTransform[0] = -1 * middleTransform[0] - 1.02;
				middleTransform[1] = -1 * middleTransform[1] + 0.27;
				middleTransform[2] = -1 * middleTransform[2] + 0.1;
				p_out.set(middleTransform);
				contourPoints.add(p_out);
			}
			
			// compute extends
			Vector3d min = new Vector3d(contourPoints.get(0));
			Vector3d max = new Vector3d(contourPoints.get(0));
			for(Vector3d p : contourPoints) {
				if(p.x<min.x) min.x=p.x;
				else if(p.x>max.x) max.x=p.x;
				if(p.y<min.y) min.y=p.y;
				else if(p.y>max.y) max.y=p.y;
				if(p.z<min.z) min.z=p.z;
				else if(p.z>max.z) max.z=p.z;
			}

			Marker m = MarkerVisualization.get().getMarker(markerId);
			if(m==null) {
				m = MarkerVisualization.get().createMarker();
				m.setType(Marker.CUBE);
				m.getColor().setR(new Double(color.x).floatValue());
				m.getColor().setG(new Double(color.y).floatValue());
				m.getColor().setB(new Double(color.z).floatValue());
				m.getColor().setA(1.0f);
			}
	
			m.getPose().getPosition().setX(0.5*(min.x+max.x));
			m.getPose().getPosition().setY(0.5*(min.y+max.y));
			m.getPose().getPosition().setZ(0.5*(min.z+max.z));

			m.getPose().getOrientation().setW(1.0);
			m.getPose().getOrientation().setX(0.0);
			m.getPose().getOrientation().setY(0.0);
			m.getPose().getOrientation().setZ(0.0);
			
			m.getScale().setX(max.x-min.x);
			m.getScale().setY(max.y-min.y);
			m.getScale().setZ(max.z-min.z);
	
			MarkerVisualization.get().putMarker(markerId,m);
			MarkerVisualization.get().publishMarkers();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	public static double getDesignatorContourSize(Designator designator) {
		try {
			List<Vector3d> contourPointsCamRel = ContourMesh.getContourPoints(designator);
			
			Vector3d min = new Vector3d(contourPointsCamRel.get(0));
			Vector3d max = new Vector3d(contourPointsCamRel.get(0));
			for(Vector3d p : contourPointsCamRel) {
				if(p.x<min.x) min.x=p.x;
				else if(p.x>max.x) max.x=p.x;
				if(p.y<min.y) min.y=p.y;
				else if(p.y>max.y) max.y=p.y;
				if(p.z<min.z) min.z=p.z;
				else if(p.z>max.z) max.z=p.z;
			}
			
			double size = Math.abs(max.x - min.x) * Math.abs( max.y - min.y);// * Math.abs(max.z - min.z);
			return size *10000;
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return 0.0;
	}
	
	public static void addDesignatorCheckerboardMesh(String markerId, Designator designator) {
		// TODO
		try {
			ColladaMesh m = CheckerBoardMesh.createCheckerBoardMesh(designator);
			//ProfileCOMMON profile = m.setPhongMaterial(
			//		new double[] {0.0, 0.0, 0.0, 1.0},
			//		new double[] {0.137255, 0.403922, 0.870588, 1},
			//		new double[] {0.5, 0.5, 0.5, 1});
			
			//String imgPath = "../kitchen/food-drinks/pizza/pizza_sauce_DIFF.png";
			//m.addDiffuseTexturePhong(profile, "tomato-sauce-diff", "UVMap", imgPath);
			
			//String meshPath = m.marshal("blue-cube-" + new Long(System.currentTimeMillis()).toString());
			
			//addMeshMarker(meshPath, "'"+meshPath+"'", position, rotation, scale);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
