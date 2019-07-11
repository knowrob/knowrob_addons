package org.knowrob.reinforcement;

import java.lang.Thread;
import java.util.Date;

import com.google.common.collect.Lists;

import geometry_msgs.TransformStamped;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.internal.loader.CommandLineLoader;
	
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import burlap.mdp.singleagent.SADomain;

import java.awt.*;
import java.util.List;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

public class BasicBehavior extends AbstractNodeMain
{

	ConnectedNode node;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("basic_behavior_test_client");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		KitchenWorld gwdg;
		KitchenTF tf;
		KitchenEnvironment env;
		KitchenState initialState;
		SADomain domain;

		StateConditionTest goalCondition;
		HashableStateFactory hashingFactory;

		gwdg = new KitchenWorld();
		domain = gwdg.generateDomain();

		tf = new KitchenTF(0.5, 0.05, 0.0, 0.5, 0.05, 0.0, 0.1);
		goalCondition = new TFGoalCondition(tf);

		env = new KitchenEnvironment(0.5, 0.05, 0.0, 0.5, 0.05, 0.0);
		String[] arg = new String[1];
		arg[0] = "org.knowrob.reinforcement.KitchenEnvironment";
		runNode(env, arg);
		env.setTF(tf);
		try
		{
			Thread.sleep(3000);
		}
		catch(Exception e)
		{
			System.out.println("failed");
		}


		//Domain generatedDomain = wrapper.generateDomain();
		//((FactoredModel)domain.getModel()).setRf(env.getRF());

		List<TransformStamped> initial_poses = env.getCurrentPoses();
		initialState = new KitchenState(initial_poses.get(0), initial_poses.get(1), initial_poses.get(2), initial_poses.get(3));

		hashingFactory = new SimpleHashableStateFactory();
	
		LearningAgent agent = new SarsaLam(domain, 0.99, hashingFactory, 0., 0.5, 0.3);

		for(int i = 0; i < 50; i++)
		{
			Episode e = agent.runLearningEpisode(env);

			e.write("/home/asil/" + "sarsa_" + i);
			System.out.println(i + ": " + e.maxTimeStep());

			//reset environment for next learning episode
			env.resetEnvironment();
		}

	}

	
	public static String getTimestamp(DBObject o) {
		return "timepoint_" + helperGetTimestamp(o);
	}

	public static double getTimestamp(DBObject start, DBObject end) {
		return helperGetTimestamp(end) - helperGetTimestamp(start);
	}

	private static double helperGetTimestamp(DBObject o) {

	    DBObject val = (DBObject) o.get("transforms");
	    val = (DBObject) val.get("header");

	    Date dt = (Date) val.get("stamp");

	    return dt.getTime()/1000.0;
	}
		
    
    public static void runNode(AbstractNodeMain node, String[] args) {
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
        NodeConfiguration nodeConfiguration = loader.build();

        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(node, nodeConfiguration);
    }
}
