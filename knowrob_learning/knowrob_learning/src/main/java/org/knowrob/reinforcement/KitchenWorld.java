package org.knowrob.reinforcement;

import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.shell.visual.VisualExplorer;
import burlap.visualizer.StatePainter;
import burlap.visualizer.StateRenderLayer;
import burlap.visualizer.Visualizer;

public class KitchenWorld implements DomainGenerator
{
	public static final String VAR_BODY = "body";
	public static final String VAR_ARM = "arm";
	public static final String VAR_FINGER_ONE = "finger_one";
	public static final String VAR_FINGER_TWO = "finger_two";

	public static final String ACTION_BODY_X_P = "body_action_x_p";
	public static final String ACTION_BODY_X_N = "body_action_x_n";
	public static final String ACTION_BODY_Z_P = "body_action_z_p";
	public static final String ACTION_BODY_Z_N = "body_action_z_n";
	public static final String ACTION_BODY_LEFT = "body_action_left";
	public static final String ACTION_BODY_RIGHT = "body_action_right";

	public static final String ACTION_ARM_X_P = "arm_action_x_p";
	public static final String ACTION_ARM_X_N = "arm_action_x_n";
	public static final String ACTION_ARM_Y_P = "arm_action_y_p";
	public static final String ACTION_ARM_Y_N = "arm_action_y_n";
	public static final String ACTION_ARM_Z_P = "arm_action_z_p";
	public static final String ACTION_ARM_Z_N = "arm_action_z_n";

	public static final String ACTION_ARM_ROLL_P = "arm_action_roll_p";
	public static final String ACTION_ARM_ROLL_N = "arm_action_roll_n";
	public static final String ACTION_ARM_PITCH_P = "arm_action_pitch_p";
	public static final String ACTION_ARM_PITCH_N = "arm_action_pitch_n";
	public static final String ACTION_ARM_YAW_P = "arm_action_yaw_p";
	public static final String ACTION_ARM_YAW_N = "arm_action_yaw_n";

	public static final String ACTION_ARM_SPREAD_ANGLE_P = "arm_spread_angle_p";
	public static final String ACTION_ARM_SPREAD_ANGLE_N = "arm_spread_angle_n";

	public static final String ACTION_ARM_CLOSING_GRIPPER = "arm_closing_gripper";


	public SADomain generateDomain() {
		SADomain domain = new SADomain(); 

		domain.addActionTypes(
			new UniversalActionType(ACTION_BODY_X_P),
			new UniversalActionType(ACTION_BODY_X_N),
			new UniversalActionType(ACTION_BODY_Z_P),
			new UniversalActionType(ACTION_BODY_Z_N)
			/*new UniversalActionType(ACTION_BODY_LEFT),
			new UniversalActionType(ACTION_BODY_RIGHT),
			new UniversalActionType(ACTION_ARM_X_P),
			new UniversalActionType(ACTION_ARM_X_N),
			new UniversalActionType(ACTION_ARM_Y_P),
			new UniversalActionType(ACTION_ARM_Y_N),
			new UniversalActionType(ACTION_ARM_Z_P),
			new UniversalActionType(ACTION_ARM_Z_N)
			new UniversalActionType(ACTION_ARM_ROLL_P),
			new UniversalActionType(ACTION_ARM_ROLL_N),
			new UniversalActionType(ACTION_ARM_PITCH_P),
			new UniversalActionType(ACTION_ARM_PITCH_N),
			new UniversalActionType(ACTION_ARM_YAW_P),
			new UniversalActionType(ACTION_ARM_YAW_N),
			new UniversalActionType(ACTION_ARM_SPREAD_ANGLE_P),
			new UniversalActionType(ACTION_ARM_SPREAD_ANGLE_N),
			new UniversalActionType(ACTION_ARM_CLOSING_GRIPPER)*/);

		return domain;
	}

}
