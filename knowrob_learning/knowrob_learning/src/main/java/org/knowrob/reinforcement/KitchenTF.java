package org.knowrob.reinforcement;

import java.lang.Math;
import geometry_msgs.TransformStamped;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;
import burlap.mdp.core.TerminalFunction;

import static org.knowrob.reinforcement.KitchenWorld.VAR_BODY;
import static org.knowrob.reinforcement.KitchenWorld.VAR_ARM;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_ONE;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_TWO;

public class KitchenTF implements TerminalFunction 
{
	//public static final double FINGER_GRIP_CONSTANT = 0.1;


	double goalFingerOneX;
	double goalFingerOneY;
	double goalFingerOneZ;

	double goalFingerTwoX;
	double goalFingerTwoY;
	double goalFingerTwoZ;

	double offset;

	public KitchenTF(double goalFingerOneX, double goalFingerOneY, double goalFingerOneZ, double goalFingerTwoX, double goalFingerTwoY, double goalFingerTwoZ,
			double offset)
	{
		this.goalFingerOneX = goalFingerOneX;
		this.goalFingerOneY = goalFingerOneY;
		this.goalFingerOneZ = goalFingerOneZ;

		this.goalFingerTwoX = goalFingerTwoX;
		this.goalFingerTwoY = goalFingerTwoY;
		this.goalFingerTwoZ = goalFingerTwoZ;

		this.offset = offset;
	}

	@Override
	public boolean isTerminal(State s) {

		KitchenState ks = (KitchenState) s;

		TransformStamped body_current = (TransformStamped)s.get(VAR_BODY);
		double body_current_x = body_current.getTransform().getTranslation().getX();
		double body_current_y = body_current.getTransform().getTranslation().getY();
		double body_current_z = body_current.getTransform().getTranslation().getZ();

		double square_current = Math.pow(body_current_x, 2) + Math.pow(body_current_y, 2) + Math.pow(body_current_z, 2);
		double square_goal = Math.pow(goalFingerOneX, 2) + Math.pow(goalFingerOneY, 2) + Math.pow(goalFingerOneZ, 2);
		double distance = Math.pow(Math.abs(square_current - square_goal), 0.5);

		if(distance <= offset)
		{
			return true;
		}

		/*KitchenState ks = (KitchenState) s;

		TransformStamped finger_one = (TransformStamped)ks.get(VAR_FINGER_ONE);
		TransformStamped finger_two = (TransformStamped)ks.get(VAR_FINGER_TWO);

		

		double finger_one_x = finger_one.getTransform().getTranslation().getX();
		double finger_one_y = finger_one.getTransform().getTranslation().getY();
		double finger_one_z = finger_one.getTransform().getTranslation().getZ();

		double finger_two_x = finger_two.getTransform().getTranslation().getX();
		double finger_two_y = finger_two.getTransform().getTranslation().getY();
		double finger_two_z = finger_two.getTransform().getTranslation().getZ();

		double square_one = Math.pow(finger_one_x, 2) + Math.pow(finger_one_y, 2) + Math.pow(finger_one_z, 2);
		double square_two = Math.pow(finger_two_x, 2) + Math.pow(finger_two_y, 2) + Math.pow(finger_two_z, 2);

		double square_goal_one = Math.pow(goalFingerOneX, 2) + Math.pow(goalFingerOneY, 2) + Math.pow(goalFingerOneZ, 2);
		double square_goal_two = Math.pow(goalFingerTwoX, 2) + Math.pow(goalFingerTwoY, 2) + Math.pow(goalFingerTwoZ, 2);
		
		double distance_one = Math.pow(Math.abs(square_goal_one - square_one), 0.5);
		double distance_two = Math.pow(Math.abs(square_goal_two - square_two), 0.5);

		System.out.println(distance_one + " -- " + distance_two);

		if(distance_one <= offset || distance_two <= offset)
		{
			return true;
		}
		else
		{
			double distance = Math.pow(Math.abs(square_one - square_two), 0.5);

			System.out.println("Distance between fingers:" + distance);
			if(distance <= offset)
				return false;
		}*/

		return false;
	}

}
