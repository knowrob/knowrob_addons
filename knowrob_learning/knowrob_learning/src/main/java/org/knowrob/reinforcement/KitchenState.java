package org.knowrob.reinforcement;

import geometry_msgs.TransformStamped;

import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;

import java.util.Arrays;
import java.util.List;

import static org.knowrob.reinforcement.KitchenWorld.VAR_BODY;
import static org.knowrob.reinforcement.KitchenWorld.VAR_ARM;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_ONE;
import static org.knowrob.reinforcement.KitchenWorld.VAR_FINGER_TWO;

public class KitchenState implements MutableState
{

	TransformStamped torso;
	TransformStamped arm;
	TransformStamped finger_one;
	TransformStamped finger_two; 

	private final static List<Object> keys = Arrays.<Object>asList(VAR_BODY, VAR_ARM, VAR_FINGER_ONE, VAR_FINGER_TWO);


	@Override
	public MutableState set(Object variableKey, Object value) {
		if(variableKey.equals(VAR_BODY)){
			this.torso = (TransformStamped) value;
		}
		else if(variableKey.equals(VAR_ARM)){
			this.arm = (TransformStamped) value;
		}
		else if(variableKey.equals(VAR_FINGER_ONE)){
			this.finger_one = (TransformStamped) value;
		}
		else if(variableKey.equals(VAR_FINGER_TWO)){
			this.finger_two = (TransformStamped) value;
		}
		else{
			throw new UnknownKeyException(variableKey);
		}
		return this;
	}

	@Override
	public List<Object> variableKeys() {
		return keys;
	}

	@Override
	public Object get(Object variableKey) {
		if(variableKey.equals(VAR_BODY)){
			return torso;
		}
		else if(variableKey.equals(VAR_ARM)){
			return arm;
		}
		else if(variableKey.equals(VAR_FINGER_ONE)){
			return finger_one;
		}
		else if(variableKey.equals(VAR_FINGER_TWO)){
			return finger_two;
		}
		else{
			throw new UnknownKeyException(variableKey);
		}
	}

	@Override
	public KitchenState copy() {
		return new KitchenState(torso, arm, finger_one, finger_two);
	}

	@Override
	public String toString() {
		return StateUtilities.stateToString(this);
	}

	public KitchenState() 
	{
	}

	public KitchenState(TransformStamped torso, TransformStamped arm, TransformStamped finger_one, TransformStamped finger_two) 
	{
		this.torso = torso;
		this.arm = arm;
		this.finger_one = finger_one;
		this.finger_two = finger_two;
	}


	public void setTorso(TransformStamped torso)
	{
		this.torso = torso;
	}

	public TransformStamped getTorso()
	{
		return torso;
	}

	public void setArm(TransformStamped arm)
	{
		this.arm = arm;
	}

	public TransformStamped getArm()
	{
		return arm;
	}

	public void setFingerOne(TransformStamped finger_one)
	{
		this.finger_one = finger_one;
	}

	public TransformStamped getFingerOne()
	{
		return finger_one;
	}

	public void setFingerTwo(TransformStamped finger_two)
	{
		this.finger_two = finger_two;
	}

	public TransformStamped getFingerTwo()
	{
		return finger_two;
	}
}
