package org.knowrob.reinforcement;

import com.google.common.base.Preconditions;
import com.google.common.base.Joiner;

import java.io.*;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;
import java.lang.Thread;

import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.message.Duration;
import org.ros.rosjava_geometry.Quaternion; 
import org.ros.message.MessageListener; 
import org.ros.tf_jni_ros.Buffer;
import pr2_controllers_msgs.Pr2GripperCommand;
import org.ros.namespace.GraphName; 
import org.ros.node.AbstractNodeMain; 
import org.ros.node.ConnectedNode; 
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Vector3;
import org.ros.exception.RemoteException;

import geometry_msgs.TransformStamped;
import geometry_msgs.PoseStamped;
import geometry_msgs.Twist;
import sensor_msgs.JointState;
import moveit_msgs.GetKinematicSolverInfo;
import moveit_msgs.GetKinematicSolverInfoRequest;
import moveit_msgs.GetKinematicSolverInfoResponse;
import moveit_msgs.GetPositionIK;
import moveit_msgs.GetPositionIKRequest;
import moveit_msgs.GetPositionIKResponse;
import moveit_msgs.GetPositionFK;
import moveit_msgs.GetPositionFKRequest;
import moveit_msgs.GetPositionFKResponse;
import moveit_msgs.JointLimits;
import moveit_msgs.JointConstraint;
import moveit_msgs.RobotState;
import pr2_controllers_msgs.Pr2GripperCommand;
import trajectory_msgs.JointTrajectory;
import trajectory_msgs.JointTrajectoryPoint;
import tf2_msgs.TFMessage;

import burlap.mdp.core.state.State;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.core.action.Action;
import burlap.mdp.singleagent.environment.EnvironmentOutcome;

import static org.knowrob.reinforcement.KitchenWorld.VAR_BODY;
import static org.knowrob.reinforcement.KitchenWorld.VAR_ARM;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_ONE;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_TWO;

import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_X_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_X_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_Z_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_Z_N;

import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_X_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_X_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_Y_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_Y_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_Z_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_Z_N;

import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_ROLL_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_ROLL_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_PITCH_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_PITCH_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_YAW_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_YAW_N;

import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_SPREAD_ANGLE_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_SPREAD_ANGLE_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_ARM_CLOSING_GRIPPER;

public class KitchenEnvironment extends AbstractNodeMain implements Environment 
{
	KitchenState current_state;
	KitchenTF terminal_function;
	KitchenRF reward_function;
	

	Subscriber<tf2_msgs.TFMessage> subscriber;
	Subscriber<sensor_msgs.JointState> js_subscriber;
	Publisher<pr2_controllers_msgs.Pr2GripperCommand> gripper_publisher;
	Publisher<geometry_msgs.Twist> body_publisher;
	Publisher<trajectory_msgs.JointTrajectory> traj_publisher;
	Publisher<sensor_msgs.JointState> state_creator;
	Publisher<trajectory_msgs.JointTrajectoryPoint> traj_point_creator;
	Publisher<moveit_msgs.JointConstraint> constraint_creator;

	String gazebo;
	String world;
	String controllers;
	String logger;

	private static final String IK_SERVICE_INFO = "pr2_right_arm_kinematics/get_ik_solver_info";
	private static final String IK_SERVICE_INFO_TYPE = "moveit_msgs/GetKinematicSolverInfo";

	private static final String IK_SERVICE_NAME = "pr2_right_arm_kinematics/get_ik";
	private static final String IK_SERVICE_NAME_TYPE = "moveit_msgs/GetPositionIK";

	private static final String FK_SERVICE_INFO = "pr2_right_arm_kinematics/get_fk_solver_info";
	private static final String FK_SERVICE_INFO_TYPE = "moveit_msgs/GetKinematicSolverInfo";

	private static final String FK_SERVICE_NAME = "pr2_right_arm_kinematics/get_fk";
	private static final String FK_SERVICE_NAME_TYPE = "moveit_msgs/GetPositionFK";

	static List<String> jointNames;
	static List<String> linkNames;
	static List<JointLimits> jointLimits; 


	Buffer tf_buffer;
	Time now;

	tf2_msgs.TFMessage last_tfs; 
	sensor_msgs.JointState last_msg;

	ServiceClient<GetKinematicSolverInfoRequest, GetKinematicSolverInfoResponse> ik_client_info;
	ServiceClient<GetPositionIKRequest, GetPositionIKResponse> ik_client_name;
	ServiceClient<GetKinematicSolverInfoRequest, GetKinematicSolverInfoResponse> fk_client_info;
	ServiceClient<GetPositionFKRequest, GetPositionFKResponse> fk_client_name;
	

	int direction = 0;


	static HashMap<String, Process> processMap = new HashMap<String, Process>();

	double goalFingerOneX, goalFingerOneY, goalFingerOneZ, goalFingerTwoX, goalFingerTwoY, goalFingerTwoZ;

	ConnectedNode node;


	public KitchenEnvironment(double goalFingerOneX, double goalFingerOneY, double goalFingerOneZ, double goalFingerTwoX, double goalFingerTwoY, double goalFingerTwoZ) 
	{

		System.out.println(runSimulationAndSolvers());
		try
		{
			Thread.sleep(30000);
		}
		catch (Exception e)
		{
			System.out.println("couldnt sleep");
		}

		this.goalFingerOneX = goalFingerOneX;
		this.goalFingerOneY = goalFingerOneY;
		this.goalFingerOneY = goalFingerOneY;

		
		this.goalFingerTwoX = goalFingerTwoX;
		this.goalFingerTwoY = goalFingerTwoY;		
		this.goalFingerTwoZ = goalFingerTwoZ;	
	}

	public KitchenEnvironment() 
	{
		this(0,0,0,0,0,0);
	}

	@Override 
	public GraphName getDefaultNodeName() { 
		return GraphName.of("environment_node").join(GraphName.newAnonymous()); 
	} 
 
	@Override 
	public void onStart(ConnectedNode connectedNode) { 
		final int queue_size = 10000; 
		tf_buffer = new Buffer();
		node = connectedNode;

		subscriber = connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE); 
		subscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() { 
			@Override 
			public void onNewMessage(tf2_msgs.TFMessage message) {
				last_tfs = message;
				final List<TransformStamped> tfs = message.getTransforms();
				for (TransformStamped tf : tfs) 
				{
					tf_buffer.setTransform(tf, "gazebo", true);
				}
			} 
		}, queue_size); 

		js_subscriber = connectedNode.newSubscriber("/joint_states", sensor_msgs.JointState._TYPE); 
		js_subscriber.addMessageListener(new MessageListener<sensor_msgs.JointState>() { 
			@Override 
			public void onNewMessage(sensor_msgs.JointState message) { 
				last_msg = message; 
			} 
		}, queue_size); 

		gripper_publisher = connectedNode.newPublisher("/r_gripper_controller/command", pr2_controllers_msgs.Pr2GripperCommand._TYPE); 
		body_publisher = connectedNode.newPublisher("/base_controller/command", geometry_msgs.Twist._TYPE);
		traj_publisher = connectedNode.newPublisher("/r_arm_controller/command", trajectory_msgs.JointTrajectory._TYPE);
		traj_point_creator = connectedNode.newPublisher("/const0", trajectory_msgs.JointTrajectoryPoint._TYPE);
		state_creator = connectedNode.newPublisher("/const", sensor_msgs.JointState._TYPE);
		constraint_creator = connectedNode.newPublisher("/const1", moveit_msgs.JointConstraint._TYPE);

		this.createService(connectedNode);

		terminal_function = new KitchenTF(goalFingerOneX, goalFingerOneY, goalFingerOneZ, goalFingerTwoX, goalFingerTwoY, goalFingerTwoZ, 0.1); 
		reward_function = new KitchenRF(goalFingerOneX, goalFingerOneY, goalFingerOneZ, goalFingerTwoX, goalFingerTwoY, goalFingerTwoZ);
		

		List<TransformStamped> current_poses = getCurrentPoses();
		current_state = new KitchenState(current_poses.get(0), current_poses.get(1), current_poses.get(2), current_poses.get(3));
	} 

	public String runSimulationAndSolvers()
	{
		String output1 = "";
		String output2 = "";
		String output3 = "";
		String output4 = "";

		try
		{
			String[] args = new String[1];
			args[0] = "";

			String[] arg_for_tf_logging = new String[8];
			arg_for_tf_logging[0] = "-t";
			arg_for_tf_logging[1] = "/tf";
			arg_for_tf_logging[2] = "-m";
			arg_for_tf_logging[3] = "localhost:27017";
			arg_for_tf_logging[4] = "-n";
			arg_for_tf_logging[5] = "tflogger";
			arg_for_tf_logging[6] = "-c";
			arg_for_tf_logging[7] = "roslog.tf" + direction;

			output1 = roslaunch("pr2_gazebo", "pr2_empty_world.launch", args);
			Thread.sleep(5000);
			output2 = roslaunch("iai_kitchen", "gazebo_spawn_kitchen.launch", args);
			Thread.sleep(5000);
			output3 = roslaunch("knowrob_learning", "fake_localization.launch", args);
			output3 = roslaunch("pr2_arm_kinematics", "pr2_ik_rarm_node.launch", args);
			Thread.sleep(5000);
			output4 = rosrun("mongodb_log", "mongodb_log_tf", arg_for_tf_logging);
			Thread.sleep(5000);
		}	
		catch(Exception e)
		{
			System.out.println(e.getCause().getMessage());
		}
		
		gazebo = output1;
		world = output2;
		controllers = output3;
		logger = output4;

		return output1 + "\n" + output2 + "\n" + output3 + "\n" + output4;
	}

	public void createService(ConnectedNode node)
	{

		try
		{		
			ik_client_info = node.newServiceClient(IK_SERVICE_INFO, IK_SERVICE_INFO_TYPE);
			ik_client_name = node.newServiceClient(IK_SERVICE_NAME, IK_SERVICE_NAME_TYPE);

			fk_client_info = node.newServiceClient(FK_SERVICE_INFO, FK_SERVICE_INFO_TYPE);
			fk_client_name = node.newServiceClient(FK_SERVICE_NAME, FK_SERVICE_NAME_TYPE);

			Thread.sleep(100);
		}
		catch (Exception e) 
		{
			e.printStackTrace();	
    		}
	}


	@Override
	public State currentObservation()
	{
		List<TransformStamped> current_poses = getCurrentPoses();
		current_state = new KitchenState(current_poses.get(0), current_poses.get(1), current_poses.get(2), current_poses.get(3));
		return current_state;
	}

	@Override
	public EnvironmentOutcome executeAction(Action a)
	{	
		return applyAction(a, "/map");
	}

	@Override
	public boolean isInTerminalState()
	{
		return terminal_function.isTerminal(current_state);
	}

	@Override
	public double lastReward()
	{
		return reward_function.getLastReward();
	}

	@Override
	public void resetEnvironment()
	{
		
		direction++;
		kill(gazebo);
		kill(world);
		kill(controllers);
		kill(logger);

		try
		{
			Thread.sleep(30000);
		}
		catch (Exception e)
		{
			System.out.println("couldnt sleep");
		}
		tf_buffer.clear();
		last_tfs = null;
		last_msg = null;
		System.gc();


		System.out.println(runSimulationAndSolvers());

		try
		{
			Thread.sleep(30000);
		}
		catch (Exception e)
		{
			System.out.println("couldnt sleep");
		}

		onStart(node);
	}

	public void setTF(KitchenTF terminal_function)
	{
		this.terminal_function = terminal_function;
	}

	public void sleep(int sleepTime)
	{
		try
		{
			Thread.sleep(sleepTime);
		}
		catch(Exception e)
		{
			System.out.println("failed");
		}
	}

	public KitchenTF getTF()
	{
		return terminal_function;
	}

	public void setRF(KitchenRF reward_function)
	{
		this.reward_function = reward_function;
	}

	public KitchenRF getRF()
	{
		return reward_function;
	}


	public EnvironmentOutcome applyAction(Action a, String target_frame_id)
	{
		EnvironmentOutcome outcome;
		boolean isFailedToExecute = false;

		System.out.println(a.actionName());
		
		if(a.actionName().startsWith("body_action"))
		{
			Twist body_cmd = body_publisher.newMessage();
			if(a.actionName().equals("body_action_x_p"))
			{
				body_cmd.getLinear().setX(5);
			}
			else if(a.actionName().equals("body_action_x_n"))
			{
				body_cmd.getLinear().setX(-5);
			}
			else if(a.actionName().equals("body_action_z_p"))
			{
				body_cmd.getLinear().setY(5);
			}
			else if(a.actionName().equals("body_action_z_n"))
			{
				body_cmd.getLinear().setY(-5);
			}
			else if(a.actionName().equals("body_action_left"))
			{
				body_cmd.getLinear().setX(5);
				body_cmd.getAngular().setZ(1.8);
			}
			else if(a.actionName().equals("body_action_right"))
			{
				body_cmd.getLinear().setX(5);
				body_cmd.getAngular().setZ(-1.8);
			}

			for(int i = 0; i < 100000; i++) body_publisher.publish(body_cmd);
			try
			{
				Thread.sleep(3000);
			}
			catch (Exception e)
			{
				//isFailedToExecute = true;
				System.out.println("couldnt sleep");
			}
			
			for(int i = 0; i < 100000; i++) body_publisher.publish(body_cmd);
		}
		else if(a.actionName().startsWith("arm_action_x") || a.actionName().startsWith("arm_action_y") || a.actionName().startsWith("arm_action_z") ||
			a.actionName().startsWith("arm_action_roll") || a.actionName().startsWith("arm_action_pitch") || a.actionName().startsWith("arm_action_yaw"))
		{
			GetPositionIKRequest req = ik_client_name.newMessage();

			ik_client_info.call(ik_client_info.newMessage(), new ServiceResponseListener<GetKinematicSolverInfoResponse>() {
			    @Override
			    public void onSuccess(GetKinematicSolverInfoResponse message) {
				jointNames = message.getKinematicSolverInfo().getJointNames();
				linkNames = message.getKinematicSolverInfo().getLinkNames();
				jointLimits = message.getKinematicSolverInfo().getLimits();

			    }

			    @Override
			    public void onFailure(RemoteException arg0) {
				//isFailedToExecute = true;
				System.out.println("failed to get solver info");
			    }
			});

			PoseStamped desiredPose = req.getIkRequest().getPoseStamped();
			TransformStamped desiredPoseTS = readTF("torso_lift_link", "r_wrist_roll_link", last_tfs);

			desiredPose.getHeader().setFrameId("torso_lift_link");
			desiredPose.getPose().getPosition().setX(desiredPoseTS.getTransform().getTranslation().getX());
			desiredPose.getPose().getPosition().setY(desiredPoseTS.getTransform().getTranslation().getY());
			desiredPose.getPose().getPosition().setZ(desiredPoseTS.getTransform().getTranslation().getZ());

			desiredPose.getPose().getOrientation().setX(desiredPoseTS.getTransform().getRotation().getX());
			desiredPose.getPose().getOrientation().setY(desiredPoseTS.getTransform().getRotation().getY());
			desiredPose.getPose().getOrientation().setZ(desiredPoseTS.getTransform().getRotation().getZ());
			desiredPose.getPose().getOrientation().setW(desiredPoseTS.getTransform().getRotation().getW());

			req.getIkRequest().setTimeout(new Duration(5.0));
			req.getIkRequest().setIkLinkName("r_wrist_roll_link");
			req.getIkRequest().setPoseStamped(desiredPose);
			//req.getIkRequest().setGroupName("right_arm");

			Quaternion q_current = new Quaternion(req.getIkRequest().getPoseStamped().getPose().getOrientation().getX(), 
							req.getIkRequest().getPoseStamped().getPose().getOrientation().getY(),
							req.getIkRequest().getPoseStamped().getPose().getOrientation().getZ(), 
							req.getIkRequest().getPoseStamped().getPose().getOrientation().getW());
			Quaternion q_result;

			if(a.actionName().equals("arm_action_x_p"))
			{
				System.out.println("arm_action_x_p:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getX());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setX(req.getIkRequest().getPoseStamped().getPose().getPosition().getX() + 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_x_n"))
			{
				System.out.println("arm_action_x_n:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getX());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setX(req.getIkRequest().getPoseStamped().getPose().getPosition().getX() - 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_y_p"))
			{
				System.out.println("arm_action_y_p:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getY());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setY(req.getIkRequest().getPoseStamped().getPose().getPosition().getY() + 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_y_n"))
			{
				System.out.println("arm_action_y_n:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getY());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setY(req.getIkRequest().getPoseStamped().getPose().getPosition().getY() - 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_z_p"))
			{
				System.out.println("arm_action_z_p:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getZ());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setZ(req.getIkRequest().getPoseStamped().getPose().getPosition().getZ() + 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_z_n"))
			{
				System.out.println("arm_action_z_n:" + req.getIkRequest().getPoseStamped().getPose().getPosition().getZ());

				req.getIkRequest().getPoseStamped().getPose().getPosition().setZ(req.getIkRequest().getPoseStamped().getPose().getPosition().getZ() - 0.04);
				q_result = q_current;
			}
			else if(a.actionName().equals("arm_action_roll_p"))
			{
				Vector3 xaxis = new Vector3(1,0,0);
				Quaternion q_rotation = Quaternion.fromAxisAngle(xaxis, 0.3);

				q_result = q_current.multiply(q_rotation);
 
			}
			else if(a.actionName().equals("arm_action_roll_n"))
			{
				Vector3 xaxis = new Vector3(1,0,0);
				Quaternion q_rotation = Quaternion.fromAxisAngle(xaxis, -0.3);

				q_result = q_current.multiply(q_rotation);
 
			}
			else if(a.actionName().equals("arm_action_pitch_p"))
			{
				Vector3 yaxis = new Vector3(0,1,0);
				Quaternion q_rotation = Quaternion.fromAxisAngle(yaxis, 0.3);

				q_result = q_current.multiply(q_rotation);
			}
			else if(a.actionName().equals("arm_action_pitch_n"))
			{
				Vector3 yaxis = new Vector3(0,1,0);
				Quaternion q_rotation = Quaternion.fromAxisAngle(yaxis, -0.3);

				q_result = q_current.multiply(q_rotation);
			}
			else if(a.actionName().equals("arm_action_yaw_p"))
			{
				Vector3 zaxis = new Vector3(0,0,1);
				Quaternion q_rotation = Quaternion.fromAxisAngle(zaxis, 0.3);

				q_result = q_current.multiply(q_rotation);
			}
			else
			{
				Vector3 zaxis = new Vector3(0,0,1);
				Quaternion q_rotation = Quaternion.fromAxisAngle(zaxis, -0.3);

				q_result = q_current.multiply(q_rotation);
			}

			req.getIkRequest().getPoseStamped().getPose().getOrientation().setX(q_result.getX());
			req.getIkRequest().getPoseStamped().getPose().getOrientation().setY(q_result.getY());
			req.getIkRequest().getPoseStamped().getPose().getOrientation().setZ(q_result.getZ());
			req.getIkRequest().getPoseStamped().getPose().getOrientation().setY(q_result.getW());

			//req.getIkRequest().getConstraints().setJointConstraints(new ArrayList<JointConstraint>(jointLimits.size()));

			req.getIkRequest().getRobotState().setJointState(state_creator.newMessage());
			req.getIkRequest().getRobotState().getJointState().setPosition(new double[jointLimits.size()]);
			for(int i = 0; i< jointLimits.size(); i++)
			{
				req.getIkRequest().getRobotState().getJointState().getName().add(jointNames.get(i));
				req.getIkRequest().getRobotState().getJointState().getPosition()[i] = (jointLimits.get(i).getMinPosition() + jointLimits.get(i).getMaxPosition())/2.0;
			}

			/*JointConstraint ankleConst = constraint_creator.newMessage();
			ankleConst.setJointName("r_wrist_roll_joint");
			
			for(int k = 0; k < last_msg.getName().size(); k++)
			{				
				if(last_msg.getName().get(k).equals("r_wrist_roll_joint"))
				{
					ankleConst.setPosition(last_msg.getPosition()[k]);
				}
				
			}
			ankleConst.setToleranceAbove(0);
			ankleConst.setToleranceBelow(0);
			ankleConst.setWeight(0.75);
			req.getIkRequest().getConstraints().getJointConstraints().add(ankleConst);*/

			ik_client_name.call(req, new ServiceResponseListener<GetPositionIKResponse>() {
			    @Override
			    public void onSuccess(GetPositionIKResponse message) {
				System.out.println("success to get ik solution:" + message.getErrorCode().getVal());
				
				RobotState solution = message.getSolution();
				JointTrajectory goal = traj_publisher.newMessage();
				goal.getHeader().setFrameId("torso_lift_link");			
				
				JointTrajectoryPoint goal_point = traj_point_creator.newMessage();
				goal_point.setTimeFromStart(new Duration(5.0));
				goal_point.setPositions(new double[solution.getJointState().getName().size()]);
				goal_point.setVelocities(new double[solution.getJointState().getName().size()]);
				goal_point.setEffort(new double[solution.getJointState().getName().size()]);
				goal_point.setAccelerations(new double[solution.getJointState().getName().size()]);
				for(int i = 0; i < solution.getJointState().getName().size(); i++)
				{
					/*if(!solution.getJointState().getName().get(i).equals("r_wrist_roll_joint"))
					{*/
						goal.getJointNames().add(solution.getJointState().getName().get(i));
						goal_point.getPositions()[i] = solution.getJointState().getPosition()[i];
						goal_point.getVelocities()[i] = 0;
						goal_point.getEffort()[i] = 0;
						goal_point.getAccelerations()[i] = 0;
					//}
				}
				goal.getPoints().add(goal_point);
				traj_publisher.publish(goal);
			    }

			    @Override
			    public void onFailure(RemoteException arg0) {
				System.out.println("failed to get ik move:" + arg0.getMessage());
				//isFailedToExecute = true;
			    }
			});

			try
			{
				Thread.sleep(5000);
			}
			catch (Exception e)
			{
				//isFailedToExecute = true;
				System.out.println("couldnt sleep");
			}
		}
		else if( a.actionName().startsWith("arm_spread_angle") || a.actionName().equals("arm_closing_gripper"))
		{
			if(a.actionName().equals("arm_spread_angle_p"))
			{

				GetPositionFKRequest req = fk_client_name.newMessage();

				fk_client_info.call(fk_client_info.newMessage(), new ServiceResponseListener<GetKinematicSolverInfoResponse>() {
				    @Override
				    public void onSuccess(GetKinematicSolverInfoResponse message) {
					jointNames = message.getKinematicSolverInfo().getJointNames();
					linkNames = message.getKinematicSolverInfo().getLinkNames();
					jointLimits = message.getKinematicSolverInfo().getLimits();
				    }

				    @Override
				    public void onFailure(RemoteException arg0) {
					System.out.println("failed to get solver info 2");
				    }
				});


				req.getHeader().setFrameId("torso_lift_link");
				req.setFkLinkNames(new ArrayList(1));
				req.getFkLinkNames().add("r_wrist_roll_link");

				req.getRobotState().getJointState().setPosition(new double[jointLimits.size()]);
				req.getRobotState().getJointState().setName(new ArrayList(jointNames.size()));

				for(int i = 0; i < jointNames.size(); i++)
				{
					req.getRobotState().getJointState().getName().add(jointNames.get(i));

					if(jointNames.get(i).equals("r_wrist_roll_joint"))
					{
						for(int k = 0; k < last_msg.getName().size(); k++)
						{
							
							if(last_msg.getName().get(k).equals("r_wrist_roll_joint"))
								req.getRobotState().getJointState().getPosition()[i] = last_msg.getPosition()[k] + 0.1;
						}
					}
					else
					{
						for(int k = 0; k < last_msg.getName().size(); k++)
						{
							
							if(last_msg.getName().get(k).equals(jointNames.get(i)))
								req.getRobotState().getJointState().getPosition()[i] = last_msg.getPosition()[k];
						}
					}
				}

				fk_client_name.call(req, new ServiceResponseListener<GetPositionFKResponse>() {
				    @Override
				    public void onSuccess(GetPositionFKResponse message) {
					System.out.println("success to get fk move");
				    }

				    @Override
				    public void onFailure(RemoteException arg0) {
					System.out.println("failed to get fk move");
					//isFailedToExecute = true;
				    }
				});

				RobotState solution = req.getRobotState();
				JointTrajectory goal = traj_publisher.newMessage();
				goal.getHeader().setFrameId("torso_lift_link");			
				
				JointTrajectoryPoint goal_point = traj_point_creator.newMessage();
				goal_point.setTimeFromStart(new Duration(5.0));
				goal_point.setPositions(new double[solution.getJointState().getName().size()]);
				goal_point.setVelocities(new double[solution.getJointState().getName().size()]);
				goal_point.setEffort(new double[solution.getJointState().getName().size()]);
				goal_point.setAccelerations(new double[solution.getJointState().getName().size()]);
				for(int i = 0; i < solution.getJointState().getName().size(); i++)
				{
					goal.getJointNames().add(solution.getJointState().getName().get(i));
					goal_point.getPositions()[i] = solution.getJointState().getPosition()[i];
					goal_point.getVelocities()[i] = 0;
					goal_point.getEffort()[i] = 0;
					goal_point.getAccelerations()[i] = 0;
				}
				goal.getPoints().add(goal_point);
				traj_publisher.publish(goal);

				try
				{
					Thread.sleep(5000);
				}
				catch (Exception e)
				{
					//isFailedToExecute = true;
					System.out.println("couldnt sleep");
				}

			}
			else if(a.actionName().equals("arm_spread_angle_n"))
			{
				GetPositionFKRequest req = fk_client_name.newMessage();

				fk_client_info.call(fk_client_info.newMessage(), new ServiceResponseListener<GetKinematicSolverInfoResponse>() {
				    @Override
				    public void onSuccess(GetKinematicSolverInfoResponse message) {
					jointNames = message.getKinematicSolverInfo().getJointNames();
					linkNames = message.getKinematicSolverInfo().getLinkNames();
					jointLimits = message.getKinematicSolverInfo().getLimits();
				    }

				    @Override
				    public void onFailure(RemoteException arg0) {
					System.out.println("failed to get solver info 2");
				    }
				});


				req.getHeader().setFrameId("torso_lift_link");
				req.setFkLinkNames(new ArrayList(1));
				req.getFkLinkNames().add("r_wrist_roll_link");

				req.getRobotState().getJointState().setPosition(new double[jointLimits.size()]);
				req.getRobotState().getJointState().setName(new ArrayList(jointNames.size()));

				for(int i = 0; i < jointNames.size(); i++)
				{
					req.getRobotState().getJointState().getName().add(jointNames.get(i));

					if(jointNames.get(i).equals("r_wrist_roll_joint"))
					{
						for(int k = 0; k < last_msg.getName().size(); k++)
						{
							
							if(last_msg.getName().get(k).equals("r_wrist_roll_joint"))
								req.getRobotState().getJointState().getPosition()[i] = last_msg.getPosition()[k] - 0.1;
						}
					}
					else
					{
						for(int k = 0; k < last_msg.getName().size(); k++)
						{
							
							if(last_msg.getName().get(k).equals(jointNames.get(i)))
								req.getRobotState().getJointState().getPosition()[i] = last_msg.getPosition()[k];
						}
					}
				}
				fk_client_name.call(req, new ServiceResponseListener<GetPositionFKResponse>() {
				    @Override
				    public void onSuccess(GetPositionFKResponse message) {
					System.out.println("success to get fk move");
				    }

				    @Override
				    public void onFailure(RemoteException arg0) {
					System.out.println("failed to get fk move");
					//isFailedToExecute = true;
				    }
				});

				try
				{
					Thread.sleep(5000);
				}
				catch (Exception e)
				{
					//isFailedToExecute = true;
					System.out.println("couldnt sleep");
				}

			}
			else
			{
				Pr2GripperCommand gripper_cmd = gripper_publisher.newMessage();
				gripper_cmd.setPosition(0.0);
				gripper_cmd.setMaxEffort(100.0);
				gripper_publisher.publish(gripper_cmd);
			
				try
				{
					Thread.sleep(5000);
				}
				catch(Exception e)
				{
					//isFailedToExecute = true;
					System.out.println("sleep exception");
				}	

			}
		}

		try
		{
			Thread.sleep(8000);
		}
		catch(Exception e)
		{
			//isFailedToExecute = true;
			System.out.println("sleep exception");
		}
		List<TransformStamped> current_poses = getCurrentPoses();
		KitchenState next_state = new KitchenState(current_poses.get(0), current_poses.get(1), current_poses.get(2), current_poses.get(3));

		outcome = new EnvironmentOutcome(current_state, a, next_state, reward_function.reward(current_state, a, next_state), terminal_function.isTerminal(next_state));
		current_state = next_state;
		return outcome;	
	}


	public TransformStamped readTF(String targetLink, String link, tf2_msgs.TFMessage msg)
	{
		TransformStamped current;

		do		
		{
			current = tf_buffer.lookupTransform(targetLink, link, last_tfs.getTransforms().get(0).getHeader().getStamp());
		}
		while(!tf_buffer.canTransform(targetLink, link, last_tfs.getTransforms().get(0).getHeader().getStamp()));

		
		return current;
	}

	private List<TransformStamped> getCurrentPoses(tf2_msgs.TFMessage msg)
	{
		ArrayList<TransformStamped> initial_tfs = new ArrayList<TransformStamped>();

		initial_tfs.add(readTF("map", "base_footprint", msg)); 
		initial_tfs.add(readTF("map", "r_wrist_roll_link", msg));
		initial_tfs.add(readTF("map", "r_gripper_r_finger_link", msg)); 
		initial_tfs.add(readTF("map", "r_gripper_l_finger_link", msg));

		return initial_tfs;
	}

	public List<TransformStamped> getCurrentPoses()
	{
		return getCurrentPoses(last_tfs);
	}
    
   
	/**
	 * Runs the given launch file of the given ROS package 
	 * @param pkg name of the ROS package
	 * @param launch_file name of the launch file
	 * @param args Arguments
	 * @return Handle to this process
	 * @throws IOException 
	 */
	public static String roslaunch(String pkg, String launch_file, String[] arg) throws IOException {
		String handle = null;
		try
		{
			String args = Joiner.on(" ").join(arg);
			Process p = Runtime.getRuntime().exec("roslaunch " + pkg + " " + launch_file + " " + args);
			handle = UUID.randomUUID().toString();
			processMap.put(handle, p);
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
		return handle;
	}

	/**
	 * Finds a ROS package using rospack and returns its path. 
	 * @param pkg name of the ROS package
	 * @param binary name of the binary
	 * @param args Arguments
	 * @return Handle to this process
	 * @throws IOException 
	 */
	public static String rosrun(String pkg, String binary, String[] arg) throws IOException {
		String handle = null;
		try
		{
		  String args = Joiner.on(" ").join(arg);
		  Process p = Runtime.getRuntime().exec("rosrun " + pkg + " " + binary + " " + args);
		  
		  handle = UUID.randomUUID().toString();
		  processMap.put(handle, p);
		}
		catch (Exception e)
		{
		  e.printStackTrace();
		}
		return handle;
	}
	
	/**
	 * Kill a process based on its UUID (which has been returned 
	 * by the RosUtilities.rosrun() method
	 * @param UUID of the process to be killed
	 * @param binary name of the binary
	 */
	public static void kill(String handle) {
		processMap.get(handle).destroy();
	}
}
