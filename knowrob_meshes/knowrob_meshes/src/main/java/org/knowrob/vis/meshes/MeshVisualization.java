
package org.knowrob.vis.meshes;

import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ros.message.Time;
import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.tfmemory.TFMemory;
// import org.knowrob.vis.MarkerObject;
// import org.knowrob.vis.MarkerPublisher;

import org.knowrob.tfmemory.StampedTransform;
import visualization_msgs.Marker;

public class MeshVisualization
{
	public static class ContourExtend {
		public final double[] min;
		public final double[] max;
		public ContourExtend(Vector3d min, Vector3d max) {
			this.min = new double[] {min.x,min.y,min.z};
			this.max = new double[] {max.x,max.y,max.z};
		}
	}
	
	public static ContourExtend getContourExtends(Designator designator, String timepoint) {
		List<Vector3d> contourPointsCamRel = ContourMesh.getContourPoints(designator);
		List<Vector3d> contourPoints = new LinkedList<Vector3d>();
		
		// FIXME: Hacky method for getting time since TF data recorded that way for pizza rolling
		double posix_ts = parseTime_d(timepoint)*1000000000;
		Time time = new Time();
		time.secs = (int)posix_ts;
		time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

		// FIXME: use parameter instead
		String sourceFrame = "/head_mount_kinect2_rgb_optical_frame";
		
		// transform to /map
		for(Vector3d camP : contourPointsCamRel) {
			StampedTransform tr = TFMemory.getInstance().lookupTransform(sourceFrame, "/map", time);
			if(tr==null) {
				System.out.println("TF data missing for '" + sourceFrame + "' " + timepoint + " missing in mongo.");
				return null;
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
		
		return new ContourExtend(min,max);
	}

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
			double posix_ts = parseTime_d(timepoint)*1000000000;
			Time time = new Time();
			time.secs = (int)posix_ts;
			time.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));

			// FIXME: use parameter instead
			String sourceFrame = "/head_mount_kinect2_rgb_optical_frame";
			
			// transform to /map
			for(Vector3d camP : contourPointsCamRel) {
				StampedTransform tr = TFMemory.getInstance().lookupTransform(sourceFrame, "/map", time);
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

// 			MarkerObject m = MarkerPublisher.get().getMarker(markerId);
// 			if(m==null) {
// 				m = MarkerPublisher.get().createMarker(markerId);
// 				m.setType(Marker.CUBE);
// 				m.setColor(new double[] {color.x, color.y, color.z, 1.0});
// 			}
// 
// 			m.setScale(new double[] {
// 				max.x-min.x,
// 				max.y-min.y,
// 				max.z-min.z
// 			});
// 			m.setTranslation(new double[] {
// 				0.5*(min.x+max.x),
// 				0.5*(min.y+max.y),
// 				0.5*(min.z+max.z)
// 			});
// 			m.setOrientation(new double[] {1.0,0.0,0.0,0.0});
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
			//ColladaMesh m = CheckerBoardMesh.createCheckerBoardMesh(designator);
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
	
	public static double parseTime_d(String timepoint) {
		String x[] = timepoint.split("timepoint_");
		// Also allow input strings without 'timepoint_' prefix
		String ts = (x.length==1 ? x[0] : x[1]);
		return Double.valueOf(ts.replaceAll("[^0-9.]", ""));
	}
}
