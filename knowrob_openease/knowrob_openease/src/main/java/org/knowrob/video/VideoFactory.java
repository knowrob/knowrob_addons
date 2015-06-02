package org.knowrob.video;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
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
	private String path_of_video = "/home/ros/summary_data/video";
	private String absolute_path_prefix = "/home/ros";
	private ConnectedNode node;
	
	//private MongoRosMessage<sensor_msgs.Image> backgroundMessage = null;
	private Publisher<std_msgs.String> backgroundMessage = null;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		path_of_video = node.getParameterTree().getString("/knowrob/videos/path", path_of_video);
		absolute_path_prefix = node.getParameterTree().getString("/knowrob/videos/absolute/path/prefix", absolute_path_prefix);
		
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
	public String[] giveAddressOfVideos(String expName)
	{
		ArrayList<String> urls = new ArrayList<String>();

		String path_of_exp_video;
		if(path_of_video.charAt(path_of_video.length() - 1) == '/')
			path_of_exp_video = path_of_video + expName;
		else
			path_of_exp_video = path_of_video + "/" + expName;
		path_of_exp_video = path_of_exp_video.replaceAll("'", "");
		if(checkExperimentVideoFolderExist(path_of_exp_video))
		{
			File folder = new File(path_of_exp_video);
			
			for (final File file : folder.listFiles())
			{
				if(!file.isDirectory())
				{
					String fileName = file.getName();
					String extension = "";
					int i = fileName.lastIndexOf('.');
					if(i > 0)
						extension = fileName.substring(i + 1);
					if (extension.equals("mp4"))
					{
						String relativePath = file.getAbsolutePath().replaceAll(absolute_path_prefix, "");
						urls.add(relativePath);
					}
				}
			}
			return urls.toArray(new String[urls.size()]);
		}
		return null;
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
