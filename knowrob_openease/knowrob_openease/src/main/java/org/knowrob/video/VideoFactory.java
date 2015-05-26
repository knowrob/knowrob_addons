package org.knowrob.video;

import java.io.File;
import java.util.ArrayList;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * @author asil@cs.uni-bremen.de
 */
public class VideoFactory extends AbstractNodeMain {
	private String path_of_video = "/home/ros/summary_data/video";
	private String absolute_path_prefix = "/home/ros";
	private ConnectedNode node;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		path_of_video = node.getParameterTree().getString("/knowrob/videos/path", path_of_video);
		absolute_path_prefix = node.getParameterTree().getString("/knowrob/videos/absolute/path/prefix", absolute_path_prefix);
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

}
