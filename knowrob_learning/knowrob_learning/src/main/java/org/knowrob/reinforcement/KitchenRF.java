package org.knowrob.reinforcement;

import java.lang.Math;
import geometry_msgs.TransformStamped;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;
import burlap.mdp.core.action.Action;
import burlap.mdp.singleagent.model.RewardFunction;

import static org.knowrob.reinforcement.KitchenWorld.VAR_BODY;
import static org.knowrob.reinforcement.KitchenWorld.VAR_ARM;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_ONE;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_TWO;

import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_X_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_X_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_Z_P;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_Z_N;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_RIGHT;
import static org.knowrob.reinforcement.KitchenWorld.ACTION_BODY_LEFT;

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

public class KitchenRF implements RewardFunction {

	public static final double DISTANCE_CONSTANT = 0.1;

	double goalFingerOneX;
	double goalFingerOneY;
	double goalFingerOneZ;

	double goalFingerTwoX;
	double goalFingerTwoY;
	double goalFingerTwoZ;

	double lastReward;

	public KitchenRF(double goalFingerOneX, double goalFingerOneY, double goalFingerOneZ, double goalFingerTwoX, double goalFingerTwoY, double goalFingerTwoZ)
	{
		this.goalFingerOneX = goalFingerOneX;
		this.goalFingerOneY = goalFingerOneY;
		this.goalFingerOneZ = goalFingerOneZ;

		this.goalFingerTwoX = goalFingerTwoX;
		this.goalFingerTwoY = goalFingerTwoY;
		this.goalFingerTwoZ = goalFingerTwoZ;

		lastReward = 0.0;
	}

	@Override
	public double reward(State s, Action a, State sprime) 
	{
		TransformStamped body_current = (TransformStamped)s.get(VAR_BODY);
		double body_current_x = body_current.getTransform().getTranslation().getX();
		double body_current_y = body_current.getTransform().getTranslation().getY();
		double body_current_z = body_current.getTransform().getTranslation().getZ();

		TransformStamped body_next = (TransformStamped)sprime.get(VAR_BODY);
		double body_next_x = body_next.getTransform().getTranslation().getX();
		double body_next_y = body_next.getTransform().getTranslation().getY();
		double body_next_z = body_next.getTransform().getTranslation().getZ();

		double square_current = Math.pow(body_current_x, 2) + Math.pow(body_current_y, 2) + Math.pow(body_current_z, 2);
		double square_next = Math.pow(body_next_x, 2) + Math.pow(body_next_y, 2) + Math.pow(body_next_z, 2);
		double square_goal = Math.pow(goalFingerOneX, 2) + Math.pow(goalFingerOneY, 2) + Math.pow(goalFingerOneZ, 2);

		double distance = Math.pow(Math.abs(square_current - square_goal), 0.5);
		double distance_candidate = Math.pow(Math.abs(square_next - square_goal), 0.5);	

		if(distance_candidate <= DISTANCE_CONSTANT)
		{
			lastReward = 1000.0;
		}
		else if(distance_candidate < distance)
		{
			lastReward = (distance - distance_candidate) * 100;
		}
		else if(distance_candidate >= distance)
		{
			lastReward = (distance - distance_candidate) * 100;
		}
		else lastReward = -5.0;

		System.out.println("Reward gained: " + lastReward);
		return lastReward;

		/*TransformStamped finger_one = (TransformStamped)s.get(VAR_FINGER_ONE);
		TransformStamped finger_two = (TransformStamped)s.get(VAR_FINGER_TWO);

		double finger_one_x = finger_one.getTransform().getTranslation().getX();
		double finger_one_y = finger_one.getTransform().getTranslation().getY();
		double finger_one_z = finger_one.getTransform().getTranslation().getZ();

		double finger_two_x = finger_two.getTransform().getTranslation().getX();
		double finger_two_y = finger_two.getTransform().getTranslation().getY();
		double finger_two_z = finger_two.getTransform().getTranslation().getZ();


		TransformStamped finger_one_next = (TransformStamped)sprime.get(VAR_FINGER_ONE);
		TransformStamped finger_two_next = (TransformStamped)sprime.get(VAR_FINGER_TWO);

		double finger_one_next_x = finger_one_next.getTransform().getTranslation().getX();
		double finger_one_next_y = finger_one_next.getTransform().getTranslation().getY();
		double finger_one_next_z = finger_one_next.getTransform().getTranslation().getZ();

		double finger_two_next_x = finger_two_next.getTransform().getTranslation().getX();
		double finger_two_next_y = finger_two_next.getTransform().getTranslation().getY();
		double finger_two_next_z = finger_two_next.getTransform().getTranslation().getZ();

		double square_current_one = Math.pow(finger_one_x, 2) + Math.pow(finger_one_y, 2) + Math.pow(finger_one_z, 2);
		double square_current_two = Math.pow(finger_two_x, 2) + Math.pow(finger_two_y, 2) + Math.pow(finger_two_z, 2);

		double square_goal_one = Math.pow(goalFingerOneX, 2) + Math.pow(goalFingerOneY, 2) + Math.pow(goalFingerOneZ, 2);
		double square_goal_two = Math.pow(goalFingerTwoX, 2) + Math.pow(goalFingerTwoY, 2) + Math.pow(goalFingerTwoZ, 2);

		double square_next_one = Math.pow(finger_one_next_x, 2) + Math.pow(finger_one_next_y, 2) + Math.pow(finger_one_next_z, 2);
		double square_next_two = Math.pow(finger_two_next_x, 2) + Math.pow(finger_two_next_y, 2) + Math.pow(finger_two_next_z, 2);

		double distance = Math.pow(Math.abs(square_current_one - square_goal_one), 0.5);
		double distance_candidate = Math.pow(Math.abs(square_next_one - square_goal_one), 0.5);		
	
		

		if(distance_candidate <= DISTANCE_CONSTANT)
		{
			lastReward = 1000.0;
		}
		else if(distance_candidate < distance)
		{
			lastReward = (distance - distance_candidate) * 100;
		}
		else if(distance_candidate >= distance)
		{
			lastReward = (distance - distance_candidate) * 100;
		}
		else lastReward = -5.0;

		System.out.println("Reward gained: " + lastReward);
		return lastReward;*/
				
		/*if(Math.abs(finger_one_x - finger_one_next_x) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_y - finger_one_next_y) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_z - finger_one_next_z) <= DISTANCE_CONSTANT &&
			Math.abs(finger_two_x - finger_two_next_x) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_y - finger_two_next_y) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_z - finger_two_next_z) <= DISTANCE_CONSTANT)
		{
			lastReward = -10.0;
			return lastReward;
		}
		else if(Math.abs(finger_one_x - goalFingerOneX) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_y - goalFingerOneY) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_z - goalFingerOneZ) <= DISTANCE_CONSTANT ||
			Math.abs(finger_two_x - goalFingerTwoX) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_y - goalFingerTwoY) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_z - goalFingerTwoZ) <= DISTANCE_CONSTANT)
		{
			lastReward = 1000.0;
			return lastReward;
		}
		else if(Math.abs(finger_one_x - goalFingerOneX) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_y - goalFingerOneY) <= DISTANCE_CONSTANT && 
			Math.abs(finger_one_z - goalFingerOneZ) <= DISTANCE_CONSTANT)
		{
			if(Math.abs(square_goal_two - square_next_two) < Math.abs(square_goal_two - square_current_two))
				lastReward = 50.0;
			else
				lastReward = -1.0;
			return lastReward;
		}
		else if(Math.abs(finger_two_x - goalFingerTwoX) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_y - goalFingerOneY) <= DISTANCE_CONSTANT && 
			Math.abs(finger_two_z - goalFingerOneZ) <= DISTANCE_CONSTANT)
		{
			if(Math.abs(square_goal_two - square_next_two) < Math.abs(square_goal_two - square_current_two))
				lastReward = 50.0;
			else
				lastReward = -1.0;
			return lastReward;
		}
		else if(distance <= 6.0 && distance_candidate < distance)
		{
			if(!a.actionName().startsWith("body_action"))
 				lastReward = 5.0;
			else lastReward = -5.0;
			return lastReward;
		}
		else if(distance <= 6.0 && distance_candidate >= distance)
		{
			lastReward = -5.0;
			return lastReward;
		}
		else if(distance > 6.0 && distance_candidate < distance)
		{
			if(a.actionName().startsWith("body_action"))
 				lastReward = 5.0;
			else lastReward = 0.0;
			return lastReward;
		}
		else if(distance > 6.0 && distance_candidate >= distance)
		{
			if(a.actionName().startsWith("body_action"))
 				lastReward = -1.0;
			else lastReward = 0.0;
			return lastReward;
		}

		lastReward = -1.0;
		System.out.println("Reward gained: " + lastReward);
		return lastReward;*/
	}

	public double getLastReward()
	{
		return lastReward;
	}
}
