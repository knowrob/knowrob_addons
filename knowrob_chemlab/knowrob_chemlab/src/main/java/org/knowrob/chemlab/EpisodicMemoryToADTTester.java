/*
 * Copyright (C) 2016 Asil Kaan Bozcuoglu.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.knowrob.chemlab;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * @author asil@cs.uni-bremen.de
 */
public class EpisodicMemoryToADTTester extends AbstractNodeMain
{

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_acat_adt/test");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		EpisodicMemoryToADT tester = new EpisodicMemoryToADT("/home/asil/catkin_ws/src/knowrob_addons/knowrob_chemlab/owl", "/home/asil/catkin_ws/src/knowrob_addons/knowrob_chemlab/dictionaries", "/home/asil/catkin_ws/src/knowrob_addons/knowrob_chemlab/owl", "http://knowrob.org/kb/acat.owl", "http://knowrob.org/kb/acat_example.owl", "chem_coll_1");

		tester.generateADT("http://knowrob.org/kb/knowrob.owl#PipettingAction");
	}


}
