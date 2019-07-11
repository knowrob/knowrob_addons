package org.knowrob.gaussian;

import java.io.*;

public class MixedGaussianInterface
{
	static { System.loadLibrary("MultiVarGauss"); }
	public native void createMixedGaussians(String outputPath);
	public native void createMultiVarGaussians(String inputPath, String outputPath);
	public native void analyzeCluster(String inputPath, String outputPath);
	public native double[] analyzeTrials(String posPath, String negPath, String outputPath, int posClusters, int negClusters);
	public native double[] likelyLocationClosest(String posPath, String negPath, int posClusters, int negClusters);
	
	public void createHeatMaps()
	{
		//String learning_path = RosUtilities.rospackFind("knowrob_learning");
		String learning_path = "/home/ros/src/knowrob_addons/knowrob_learning";
		String script_path = learning_path + "/scripts";
		
		try
		{
			Process p_clean = Runtime.getRuntime().exec("rm " + "/home/ros/user_data/heatmap_mixture.pdf" + " " + "/home/ros/user_data/heatmap_mixture.jpg"
				+ "/home/ros/user_data/heatmap_multivar.pdf" + " " + "/home/ros/user_data/heatmap_multivar.jpg"
				, null, new File(script_path));
                        p_clean.waitFor();
			Process p_plot = Runtime.getRuntime().exec(script_path + "/plot.sh", null, new File(script_path));
                        p_plot.waitFor();
			Process p_convert1 = Runtime.getRuntime().exec("convert -density 800 " + "/home/ros/user_data/heatmap_mixture.pdf" + " " + "/home/ros/user_data/heatmap_mixture.jpg"
				, null, new File(script_path));
                        p_convert1.waitFor();
			Process p_convert2 = Runtime.getRuntime().exec("convert -density 800 " + "/home/ros/user_data/heatmap_multivar.pdf" + " " + "/home/ros/user_data/heatmap_multivar.jpg"
				, null, new File(script_path));
			p_convert2.waitFor();
		}
		catch (Exception e)
		{
		       e.printStackTrace();
		}

	}

	public void writeFeature(float[][] features)
	{
		if(features[0].length == 4)
		{
			try
			{
				PrintWriter writer = new PrintWriter( "/home/ros/user_data/grasp_positions.json", "UTF-8");
				
				for(int i = 0; i < features.length; i++)
				{
					if(features[i][0] == 0)
					{
						writer.println("[False, " + features[i][1] + ", " +  features[i][2] + ", " + features[i][3] + "]");
					}
					else
					{
						writer.println("[True, " + features[i][1] + ", " +  features[i][2] + ", " + features[i][3] + "]");
					}
				}
				writer.close();			


			}
			catch (Exception e)
			{
			       e.printStackTrace();
			}



		}
		else System.out.println("Feature size is not correct!");
	}

        public void writeSimpleFeatures(float[][] features, String filename)
	{
		if(features[0].length == 7)
		{
                        try
			{
				PrintWriter writer;

				if(filename.startsWith("/home/")) writer = new PrintWriter( filename, "UTF-8");
				else writer = new PrintWriter( "/home/ros/user_data/" + filename, "UTF-8");
				writer.println("REL-X,REL-Y,REL-Z,REL-THETA");
				
				for(int i = 0; i < features.length; i++)
				{
					 //z = qz / sqrt(1-qw*qw)
					float theta = 180 *  features[i][6] / (float)Math.sqrt(1 - features[i][3] * features[i][3]);
					writer.println(features[i][0] + ", " + features[i][1] + ", " + features[i][2] + ", " + theta);
				}
				writer.close();			


			}
			catch (Exception e)
			{
			       e.printStackTrace();
			}



		}
		else System.out.println("Feature size is not correct!");
	}	

	public void writeFeature(float[][] floatFeatures, String[][] stringFeatures)
	{
		if(floatFeatures.length == stringFeatures.length && floatFeatures[0].length == 4 && stringFeatures[0].length == 2)
		{
			try
			{
				PrintWriter writer = new PrintWriter( "/home/ros/user_data/grasp_features.csv", "UTF-8");

				writer.println("REL-X,REL-Y,REL-Z,REL-THETA,ACTION,OBJECT");

				for(int i = 0; i < floatFeatures.length; i++)
				{
					writer.println(floatFeatures[i][0] + ", " + floatFeatures[i][1] + ", " +  floatFeatures[i][2] + ", " + floatFeatures[i][3] 
							+ ", " + stringFeatures[i][0] + ", " + stringFeatures[i][1]);
				}

				writer.close();
			}
			catch (Exception e)
			{
			       e.printStackTrace();
			}
		}
		else System.out.println("Feature size is not correct!");
	}

}
