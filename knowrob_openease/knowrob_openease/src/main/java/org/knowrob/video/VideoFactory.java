package org.knowrob.video;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Date;

import javax.imageio.ImageIO;

import org.knowrob.interfaces.mongo.types.ISODate;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.Image;

import com.mongodb.BasicDBObject;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.CvMat;
import org.bytedeco.javacpp.opencv_core.IplImage;
import static org.bytedeco.javacpp.opencv_imgcodecs.*;



import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.FileWriter;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;



/**
 * @author asil@cs.uni-bremen.de
 */
public class VideoFactory extends AbstractNodeMain implements MessageListener<sensor_msgs.Image> {
	private String path_of_video = "/home/ros/summary_data/video";
        private String path_of_web_video_server = "/home/ros/devel/lib/web_video_server/bin";
	private String absolute_path_prefix = "/home/ros";
        private String path_of_recorded_video= "/home/ros/user_data";
	private ConnectedNode node = null;
	
	private Publisher<std_msgs.String> backgroundMessage = null;

	private Publisher<sensor_msgs.Image> frameReSender = null;	
	private Subscriber<sensor_msgs.Image> frameReceiver = null;
	
	private boolean recordingStarted = false;
	private String recordingTime = "";
	private long frameCounter = 0;
	
	private int in_fps = 15;
	private int out_fps = 30;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		
		frameReceiver = connectedNode.newSubscriber("/openease/video/frame", sensor_msgs.Image._TYPE);
		frameReceiver.addMessageListener(this);
		
		path_of_video = node.getParameterTree().getString("/knowrob/videos/path", path_of_video);
		path_of_web_video_server = node.getParameterTree().getString("/web_video_server/path", path_of_web_video_server);
		absolute_path_prefix = node.getParameterTree().getString("/knowrob/videos/absolute/path/prefix", absolute_path_prefix);

		frameReSender = connectedNode.newPublisher("snapshots", sensor_msgs.Image._TYPE);
		backgroundMessage = connectedNode.newPublisher("background_images", std_msgs.String._TYPE);
	

	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_openease/video");
	}
	
	///////////////////////////////////
	//////// Listening on /openease/video/frame topic and generate video from messages
	///////////////////////////////////
	
	
	public void setVideoFPS(int in_fps, int out_fps) {
		this.in_fps = in_fps;
		this.out_fps = out_fps;
	}
	
	public boolean startRecording() {
		if(isRecording()) return false;
		recordingStarted = true;
		recordingTime = "video_created"; 
		frameCounter = 0;
		
		final File targetDir = new File(path_of_recorded_video + "/" +recordingTime);
		String cmd = new StringBuilder().append("rm -r ").
			append(targetDir.getAbsolutePath()).toString();
		executeCommand(cmd);
		
		
		System.err.println("Start recording video: " + recordingTime);
		return true;
	}
	
	public boolean stopRecording() {

		if(!isRecording()) return false;
		recordingStarted = false;
		System.err.println("Number of video frames: " + frameCounter);

		final File tmpDir = new File(path_of_recorded_video);
		final File targetDir = new File(path_of_recorded_video + "/" +recordingTime);
		final File videoServerBinDir = new File(path_of_web_video_server);


		if(!tmpDir.exists())
		{
			String cmd = new StringBuilder().append("mkdir ").
			   append(tmpDir.getAbsolutePath()).toString();
			executeCommand(cmd);
		}
		if(!targetDir.exists())
		{
			String cmd = new StringBuilder().append("mkdir ").
			   append(targetDir.getAbsolutePath()).toString();
			executeCommand(cmd);
		}
                if(targetDir.exists() && frameCounter>0) {
			String cmd = new StringBuilder().
			   append("mencoder \"mf://*.jpg\" -mf type=jpg:fps=").append(in_fps).
			   append(" -o video.mpg -speed 1 -ofps ").append(out_fps).
			   append(" -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=2500 -oac copy -of mpeg").toString();
			executeCommand(new String[]{"/bin/bash", "-c", cmd}, targetDir);

			String conversion_cmd = new StringBuilder().
			   append("avconv -i video.mpg -vf scale=\"trunc(iw/2)*2:trunc(ih/2)*2\" -c:v libx264 video.mp4").toString();
			executeCommand(new String[]{"/bin/bash", "-c", conversion_cmd}, targetDir);
		}

		return true;
	}
	
	public void executeCommand(String command) {
	    Process p;
	    System.out.println(command);
	    try {
	        p = Runtime.getRuntime().exec(command);
	        p.waitFor();
	        BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
	        String line = reader.readLine();           
	        while (line != null) {
	        	node.getLog().info(line + "\n");
			line = reader.readLine();
	        }
	    }
	    catch (Exception exc) {
    		node.getLog().error("Unable to run process.", exc);
	    }
	}


        public void executeCommand(String[] cmd, File directory) {
		StringBuffer theRun = null;
		try {
			Process process = Runtime.getRuntime().exec(cmd, null, directory);

			BufferedReader reader = new BufferedReader(
				new InputStreamReader(process.getInputStream()));
			int read;
			char[] buffer = new char[4096];
			StringBuffer output = new StringBuffer();
			while ((read = reader.read(buffer)) > 0) {
				theRun = output.append(buffer, 0, read);
			}
			reader.close();
			process.waitFor();

		} catch (IOException e) {
			throw new RuntimeException(e);
		} catch (InterruptedException e) {
			throw new RuntimeException(e);
		}
		System.out.println(theRun.toString().trim());
	}
	
	public boolean isRecording() {
		return recordingStarted;
	}
	
        @Override
	public void onNewMessage(Image frame) {
		if(!isRecording()) {
			node.getLog().debug("Ignoring video frame: " + frame.getHeader().getStamp().toString());
			return;
		}

		final File tmpDir = new File(path_of_recorded_video);
		final File targetDir = new File(path_of_recorded_video + "/" +recordingTime);

		if(!tmpDir.exists())
		{
			String cmd = new StringBuilder().append("mkdir ").
			   append(tmpDir.getAbsolutePath()).toString();
			executeCommand(cmd);
		}
		if(!targetDir.exists()) {
	    		try { targetDir.mkdir(); }
	    		catch(Exception exc) {
	    			node.getLog().error("Unable to create video directory.", exc);
	    			stopRecording();
	    			return;
	    		}
		}

		final File targetFile = new File(targetDir, ""+frameCounter+".jpg");
		try {
			final CvMat cvFrame = CVBridge.getMappedCvMat(frame);
			cvSaveImage(targetFile.getAbsolutePath(), cvFrame);
			frameCounter += 1;
		}
		catch (Exception exc) {
    		        node.getLog().error("Failed to write frame.", exc);
		}
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

	public boolean checkExperimentVideoFolderExist(String expPath)
	{
		File folder = new File(expPath);
		if(!(folder.exists() && folder.isDirectory()))
			return false;
		else return true;
		
	}
	
	public static String[] getVideoURLs(String cat, String exp)
	{
		LinkedList<String> urls = new LinkedList<String>();
		File videoDir = new File("/episodes/"+cat+"/"+exp+"/videos");
		if(videoDir.exists()) {
			for (final File file : videoDir.listFiles()) {
				if (file.getName().startsWith(".")) continue;
				urls.add("/knowrob/knowrob_data/"+cat+"/"+exp+"/videos/"+file.getName());
			}
		}
		if(urls.isEmpty())
			return null;
		else
			return urls.toArray(new String[urls.size()]);
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
					if (fileName.startsWith(".")) continue;
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
	
	// TODO: move to openease package
// 	public static String[] getVideoURLs(String cat, String exp)
// 	{
// 		LinkedList<String> urls = new LinkedList<String>();
// 		File expDir = new File("/episodes/"+cat+"/"+exp);
// 		if(expDir.exists()) {
// 			for (final File episodeDir : expDir.listFiles()) {
// 				if(!episodeDir.isDirectory()) continue;
// 				File videoDir = new File(episodeDir, "videos");
// 				if(videoDir.exists()) {
// 					for (final File vidFile : videoDir.listFiles()) {
// 						urls.add("/knowrob/knowrob_data/"+cat+"/"+exp+"/"+episodeDir.getName()+"/videos/"+vidFile.getName());
// 					}
// 				}
// 				for (final File vidFile : episodeDir.listFiles()) {
// 					if(vidFile.isDirectory()) continue;
// 					String ext = vidFile.getName().substring(vidFile.getName().indexOf(".") + 1);
// 					if (ext.equalsIgnoreCase("avi") ||
// 					    ext.equalsIgnoreCase("mpg") ||
// 					    ext.equalsIgnoreCase("mp4") ||
// 					    ext.equalsIgnoreCase("mpeg") ||
// 					    ext.equalsIgnoreCase("flv") ||
// 					    ext.equalsIgnoreCase("mov") ||
// 					    ext.equalsIgnoreCase("mkv")) {
// 						urls.add("/knowrob/knowrob_data/"+cat+"/"+exp+"/"+episodeDir.getName()+"/" + vidFile.getName());
// 					}
// 				}
// 			}
// 		}
// 		if(urls.isEmpty())
// 			return null;
// 		else
// 			return urls.toArray(new String[urls.size()]);
// 	}
}
