package org.knowrob.video;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Date;

import javax.imageio.ImageIO;

import org.knowrob.interfaces.mongo.MongoRosMessage;
import org.knowrob.interfaces.mongo.types.ISODate;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.mongodb.BasicDBObject;

/**
 * @author asil@cs.uni-bremen.de
 */
public class VideoFactory extends AbstractNodeMain {
	private ConnectedNode node;
	
	//private MongoRosMessage<sensor_msgs.Image> backgroundMessage = null;
	private Publisher<std_msgs.String> backgroundMessage = null;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		//backgroundMessage = new MongoRosMessage<sensor_msgs.Image>(sensor_msgs.Image._TYPE, "background_images");
		//backgroundMessage.connect(connectedNode);
		backgroundMessage = connectedNode.newPublisher("background_images", std_msgs.String._TYPE);
	}
	
	public boolean waitOnPublisher() {
		try {
			while(backgroundMessage ==null) {
				Thread.sleep(200);
			}
			return true;
		} catch (InterruptedException e) {
			e.printStackTrace();
			return false;
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_openease/video");
	}

	public boolean checkExperimentVideoFolderExist(String expPath)
	{
		File folder = new File(expPath);
		if(!(folder.exists() && folder.isDirectory()))
			return false;
		else return true;
		
	}
	
	public String[] collectVideos(String rootPath)
	{
		if (rootPath.startsWith("'")) rootPath = rootPath.substring(1, rootPath.length());
		if (rootPath.endsWith("'"))   rootPath = rootPath.substring(0, rootPath.length() - 1);
		LinkedList<String> vids = new LinkedList<String>();
		collectVideos(rootPath, vids);
		return vids.toArray(new String[vids.size()]);
	}
	
	private void collectVideos(String rootPath, LinkedList<String> out)
	{
		File f0 = new File(rootPath);
		if(!f0.canRead()) {
			System.err.println("Unable to read episode data at " + rootPath + ". Wrong file permissions?");
			return;
		}
		File[] subdirs = f0.listFiles();
		System.out.println(rootPath);
		if(subdirs==null) return;
		for(File f1 : subdirs) {
			System.out.println(f1.getAbsolutePath());
			if(f1.getName().equals("videos")) {
				File[] listOfVids = f1.listFiles();
				for(File v : listOfVids) {
					out.add(v.getAbsolutePath().replaceFirst("/episodes", "/knowrob_data"));
				}
			}
			else if(f1.isDirectory()) {
				collectVideos(f1.toString(), out);
			}
		}
	}
	
	/*
	public boolean publishBackground(BasicDBObject mngObj) {
		// wait for publisher to be ready
		if(!waitOnPublisher()) return false;
		
		return backgroundMessage.publish(mngObj);
	}
	*/
	
	// HACK
	public boolean publishBackground(double t) {
		// wait for publisher to be ready
		if(!waitOnPublisher()) return false;

		double t0 = 1432891729.012;
		double t1 = 1432891993.388;
		double count = 338.0;
		double step = (t1-t0)/count;
		int x = (int)Math.floor((t-t0)/step);
		
		try {
			final std_msgs.String str_msg = backgroundMessage.newMessage();
			
			String frameName = "frame";
			if(x < 1000) frameName += "0";
			if(x < 100) frameName += "0";
			if(x < 10) frameName += "0";
			frameName += new Integer(x).toString();
			frameName += ".jpg";
			
			str_msg.setData("/knowrob/knowrob_data/logs/augmented_reality/kinect_frames/"+frameName);
			
			backgroundMessage.publish(str_msg);
		}
		catch(Exception exc) {
			exc.printStackTrace();
		}
		
		return true;
	}
}
