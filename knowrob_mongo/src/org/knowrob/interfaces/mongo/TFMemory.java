/* 
 * Copyright (c) 2013 Moritz Tenorth, Sjoerd van den Dries
 * 
 * Based on the TFListener class by Sjoerd v.d. Dries in tfjava
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
*/

package org.knowrob.interfaces.mongo;

import ros.communication.*;

import ros.pkg.geometry_msgs.msg.TransformStamped;
import tfjava.Frame;
import tfjava.Stamped;
import tfjava.StampedTransform;
import tfjava.TimeCache;
import tfjava.TransformStorage;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;

import org.knowrob.interfaces.mongo.util.ISO8601Date;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;

import java.text.SimpleDateFormat;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.LinkedList;

/**
 * A client that listens to the /tf topic, stores transforms in a buffer and allows transformation
 * lookups from one frame to another.
 * 
 * The listener runs in a seperate thread. All tf messages published on the /tf topic are stored in
 * a buffer, first sorted by child frame, then by parent frame, then by time stamp. This allows fast
 * lookup of transformations. Tf's that are MAX_STORAGE_TIME older than the newest tf in the corresponding
 * time cache are ignored.
 * 
 * To calculate a transformation from some source frame S to a target frame T at time t, TFListener uses a graph
 * search to find the best path from S to T. At the moment, 'best' means that the largest difference between
 * the time stamps of the transformations on the path and time t is minimized. If the tf graph is a tree, as is
 * the case with original C++-implementation of tf, the graph will simply return the only path available (if any).
 * 
 * TFlistener is implemented as a singleton, which guarantees that at any time at most one client per system is
 * listening to the /tf topic.
 * 
 * @author Sjoerd van den Dries
 * @version March 4, 2011
 */
public class TFMemory {
    
    /** Maximum buffer storage time */
    public static final long MAX_STORAGE_TIME = (new Duration(10, 0)).totalNsecs(); 
    
    /** The singleton instance */
    protected static TFMemory instance;
    
    /** Map that maps frame IDs (names) to frames */    
    protected HashMap<String, Frame> frames;

    /** TF name prefix, currently not used (TODO) */
    protected String tfPrefix = "";
    

    
    /* **********************************************************************
     * *                           INITIALIZATION                           *
     * ********************************************************************** */ 
    
    /**
     * Returns the TFListener instance.
     */    
    public synchronized static TFMemory getInstance() {
    	
        if (instance == null) {
            instance = new TFMemory();
        }
        return instance;
    }
    
    /**
     * Class constructor.
     */    
    protected TFMemory() {
    	
        frames = new HashMap<String, Frame>();    
	    
	}	

	
    /* **********************************************************************
     * *                            TF LISTENER                             *
     * ********************************************************************** */	
	
    protected boolean setTransforms(String json_transforms) {

    	JSONArray tfs = JSONArray.fromObject(json_transforms);


    	for (int i = 0; i < tfs.size(); i++) {

    		setTransform(tfs.getJSONObject(i));
    		
    		
    	}


    	return true;
    }
    
	/**
	 * Converts transform (a geometry msg) to a TransformStorage object and adds it to the buffer.
	 */    
    protected boolean setTransform(JSONObject tf_stamped) {
    	
    	// read JSON string
    	JSONObject json_header = tf_stamped.getJSONObject("header");
    	JSONObject json_transform = tf_stamped.getJSONObject("transform");
    	
	    // resolve the frame ID's
        String childFrameID = assertResolved(tfPrefix, tf_stamped.getString("child_frame_id"));
	    String frameID = assertResolved(tfPrefix, json_header.getString("frame_id"));
 
	    
	    boolean errorExists = false;
	    if (childFrameID == frameID) {
	        System.err.println("TF_SELF_TRANSFORM: Ignoring transform with frame_id and child_frame_id  \"" + childFrameID + "\" because they are the same");
	        errorExists = true;
	    }

	    if (childFrameID == "/") { //empty frame id will be mapped to "/"
	        System.err.println("TF_NO_CHILD_FRAME_ID: Ignoring transform because child_frame_id not set ");
	        errorExists = true;
	    }

	    if (frameID == "/") { //empty parent id will be mapped to "/"
	        System.err.println("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"" + childFrameID + "\" because frame_id not set");
	        errorExists = true;
	    }

	    if (errorExists) return false;	    
	    
	    // lookup or insert child frame
	    Frame frame = lookupOrInsertFrame(childFrameID);
	    
	    // convert tf message to JTransform datastructure
	    double trans_x = json_transform.getJSONObject("translation").getDouble("x");
	    double trans_y = json_transform.getJSONObject("translation").getDouble("y");
	    double trans_z = json_transform.getJSONObject("translation").getDouble("z");
	    
	    double rot_w = json_transform.getJSONObject("rotation").getDouble("w");
	    double rot_x = json_transform.getJSONObject("rotation").getDouble("x");
	    double rot_y = json_transform.getJSONObject("rotation").getDouble("y");
	    double rot_z = json_transform.getJSONObject("rotation").getDouble("z");
	    
	    // add frames to map
	    Frame childFrame = lookupOrInsertFrame(childFrameID);
	    Frame parentFrame = lookupOrInsertFrame(frameID);
	    
	     
	    ISO8601Date timestamp = ISO8601Date.parse(json_header.getJSONObject("stamp").getString("$date"));
	    
	    TransformStorage tf = new TransformStorage(new Vector3d(trans_x, trans_y, trans_z),
	                                new Quat4d(rot_x, rot_y, rot_z, rot_w),
	                                timestamp.toNanoSeconds(),
	                                parentFrame, childFrame);
	    
	    
	    // try to insert tf in corresponding time cache. If result is FALSE, the tf contains old data.
	    if (!frame.insertData(tf)) {
	        System.err.println("TF_OLD_DATA ignoring data from the past for frame \"" + childFrameID + "\" at time " + ((double)tf.getTimeStamp() / 1E9));
            return false;
	    }

        return true;
	}
	
    /**
     * Looks up and returns the frame belonging to the given frame ID.
     * If the frame does not exist yet, it is first added to the map.
     */    
	protected Frame lookupOrInsertFrame(String frameID) {
        Frame frame = frames.get(frameID);
        if (frame == null) {
            frame = new Frame(frameID, MAX_STORAGE_TIME);
            frames.put(frameID, frame);
        }	    
        return frame;
	}
    
	
    public static java.util.Date parseDate( String input ) throws java.text.ParseException {

        //NOTE: SimpleDateFormat uses GMT[-+]hh:mm for the TZ which breaks
        //things a bit.  Before we go on we have to repair this.
        SimpleDateFormat df = new SimpleDateFormat( "yyyy-MM-dd'T'HH:mm:ssz" );
        
        //this is zero time so we need to add that TZ indicator for 
        if ( input.endsWith( "Z" ) ) {
            input = input.substring( 0, input.length() - 1) + "GMT-00:00";
        } else {
            int inset = 6;
        
            String s0 = input.substring( 0, input.length() - inset );
            String s1 = input.substring( input.length() - inset, input.length() );

            input = s0 + "GMT" + s1;
        }
        
        return df.parse( input );
        
    }
	
    /* **********************************************************************
     * *                         TRANSFORM METHODS                          *
     * ********************************************************************** */ 
	
	/**
	 * Transforms a stamped point to the given target frame, and returns the result in stampedOut.
	 */	
	public void transformPoint(String targetFrameID, Stamped<Point3d> stampedIn, Stamped<Point3d> stampedOut) {
	    StampedTransform transform = lookupTransform(targetFrameID, stampedIn.frameID, stampedIn.timeStamp);
	    transform.transformPoint(stampedIn.getData(), stampedOut.getData());
	    stampedOut.frameID = targetFrameID;
	    stampedOut.timeStamp = stampedIn.timeStamp;
	}
	
    /**
     * Transforms a stamped point to the given target frame and time, based on a given fixed frame, and
     * returns the result in stampedOut.
     */	
	public void transformPoint(String targetFrameID, Time targetTime, Stamped<Point3d> stampedIn,
	                                            String fixedFrameID, Stamped<Point3d> stampedOut) {
	    StampedTransform transform = lookupTransform(targetFrameID, targetTime, stampedIn.frameID, stampedIn.timeStamp, fixedFrameID);
        transform.transformPoint(stampedIn.getData(), stampedOut.getData()); 
        stampedOut.frameID = targetFrameID;
        stampedOut.timeStamp = stampedIn.timeStamp;
	}
    /**
     * Transforms a stamped pose to the given target frame, and returns the result in stampedOut.
     */ 	
	public void transformPose(String targetFrameID, Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
	    StampedTransform transform = lookupTransform(targetFrameID, stampedIn.frameID, stampedIn.timeStamp);
	    transform.transformPose(stampedIn.getData(), stampedOut.getData());	    
        stampedOut.frameID = targetFrameID;
        stampedOut.timeStamp = stampedIn.timeStamp;
	}
	 
	/**
     * Transforms a stamped pose to the given target frame and time, based on a given fixed frame, and
     * returns the result in stampedOut.
     */ 	
	public void transformPose(String targetFrameID, Time targetTime, Stamped<Matrix4d> stampedIn,
	                                            String fixedFrameID, Stamped<Matrix4d> stampedOut) {
	    StampedTransform transform = lookupTransform(targetFrameID, targetTime, stampedIn.frameID, stampedIn.timeStamp, fixedFrameID);
	    transform.transformPose(stampedIn.getData(), stampedOut.getData());
        stampedOut.frameID = targetFrameID;
        stampedOut.timeStamp = stampedIn.timeStamp;
    }
	
    /* **********************************************************************
     * *                          LOOKUP METHODS                            *
     * ********************************************************************** */    

	/**
	 * Returns the transform from the specified source frame to the target frame at a given time; returns
	 * null if no transformation could be found.
	 */
	public StampedTransform lookupTransform(String targetFrameID, String sourceFrameID, Time time) {
        // resolve the source and target IDs
	    String resolvedTargetID = assertResolved(tfPrefix, targetFrameID);
        String resolvedSourceID = assertResolved(tfPrefix, sourceFrameID);
        
        // if source and target are the same, return the identity transform
        if (resolvedSourceID == resolvedTargetID) {
            StampedTransform out = StampedTransform.getIdentity();
            out.timeStamp = time;
            out.frameID = resolvedSourceID;
            out.childFrameID = resolvedTargetID;            
            return out;
        }

        // lookup source and target frame
        Frame sourceFrame = frames.get(resolvedSourceID);
        Frame targetFrame = frames.get(resolvedTargetID);
        
        if (sourceFrame == null) {
            System.err.println("Cannot transform: source frame \"" + resolvedSourceID + "\" does not exist.");
            return null;
        }    
            
        if (targetFrame == null) {
            System.err.println("Cannot transform: target frame \"" + resolvedTargetID + "\" does not exist.");
            return null;
        } 
	    
        // list that will contain transformations from source frame to some frame F        
	    LinkedList<TransformStorage> inverseTransforms = new LinkedList<TransformStorage>();
	    // list that will contain transformations from frame F to target frame
	    LinkedList<TransformStorage> forwardTransforms = new LinkedList<TransformStorage>();
	    
	    // fill the lists using lookupLists. If it returns FALSE, no transformation could be found.
	    if (!lookupLists(targetFrame, sourceFrame, time.totalNsecs(), inverseTransforms, forwardTransforms)) {
	        // TODO give warning
	        System.err.println("Cannot transform: source + \"" + resolvedSourceID + "\" and target \""
	                        + resolvedTargetID + "\" are not connected.");
	        return null;
	    }        
	    
	    // create an identity transform with the correct time stamp
	    StampedTransform out = StampedTransform.getIdentity();	    
	    out.timeStamp = time;
	    
        // multiply all transforms from source frame to frame F TODO: right?
        for(TransformStorage t : inverseTransforms) {           
            out.mul(StorageToStampedTransform(t));
        }
	    
	    // multiply all transforms from frame F to target frame TODO: right?
	    for(TransformStorage t : forwardTransforms) {	        
	        out.mul(StorageToStampedTransform(t).invert(), out);
        }	    
	    
	    // return transform
	    return out;
	}
	
	/**
     * Returns the transform from the specified source frame at sourceTime to the target frame at a given
     * targetTime, based on a given fixed frame; returns null if no transformation could be found.
     */
	public StampedTransform lookupTransform(String targetID, Time targetTime, String sourceID, Time sourceTime, String fixedID) {	    
	    // lookup transform from source to fixed frame, at sourceTime
	    StampedTransform t1 = lookupTransform(fixedID, sourceID, sourceTime);
	    // lookup transform from fixed frame to target frame, at targetTime
	    StampedTransform t2 = lookupTransform(targetID, fixedID, targetTime);
	    
	    // if either of the two transformations did not succeed, return null
	    if (t1 == null || t2 == null) return null;	    
	    
	    // multiply transformation t2 with t1, and return
	    t2.mul(t1);
	    return t2;
	}
	
	/**
	 * Performs a bi-directional best-first graph search on the tf graph to try to find a path from sourceFrame
	 * to targetFrame, at the given time. One priority queue is used to keep a sorted list of all search nodes
	 * (from both directions, ordered descending by their potential of contributing to a good solution). At
	 * the moment, the cost of the path from A to B is defined as the largest absolute difference between the
	 * time stamps of the transforms from A to B and the given time point. This corresponds to searching for a
	 * transform path that needs the least amount of inter- and extrapolation.  
	 * 
	 * Note: often in search, if we talk about expanding a search node, we say that the node expands and its
	 * _children_ are added to the queue. Yet, the tf graph is stored by linking child frames to their _parent_
	 * frames, not the other way around. So, if a search node is expanded, the _parent_ frames are added to the
	 * queue. This may be a bit confusing.
	 */	
    protected boolean lookupLists(Frame targetFrame, Frame sourceFrame, long time,
            LinkedList<TransformStorage> inverseTransforms, LinkedList<TransformStorage> forwardTransforms) {
        
        // wrap the source and target frames in search nodes
        SearchNode<Frame> sourceNode = new SearchNode<Frame>(sourceFrame);
        SearchNode<Frame> targetNode = new SearchNode<Frame>(targetFrame);
        
        // set beginning of forward path (from source)
        sourceNode.backwardStep = sourceNode;
        // set beginning of backward path (form target)
        targetNode.forwardStep = targetNode;        
        
        // create a hash map that map frames to search nodes. This is necessary to keep track of
        // which frames have already been visited (and from which direction).
        HashMap<Frame, SearchNode<Frame>> frameToNode = new HashMap<Frame, SearchNode<Frame>>();
        
        // add source and target search nodes to the map
        frameToNode.put(sourceFrame, sourceNode);
        frameToNode.put(targetFrame, targetNode);
        
        // create a priority queue, which will hold the search nodes ordered by cost (descending)
        PriorityQueue<SearchNode<Frame>> Q = new PriorityQueue<SearchNode<Frame>>();
        
        // at the source and target search nodes to the queue
        Q.add(sourceNode); 
        Q.add(targetNode);
        
        // perform the search
        while(!Q.isEmpty()) {
            // poll most potential search node from queue
            SearchNode<Frame> frameNode = Q.poll();
            Frame frame = frameNode.content;
            
            // if the node is both visited from the source and from the target node, a path has been found
            if (frameNode.backwardStep != null && frameNode.forwardStep != null) {
                // found the best path from source to target through FRAME.
                
                // create inverse list (from source to FRAME)
                SearchNode<Frame> node = frameNode;                
                while(node.content != sourceNode.content) {                    
                    inverseTransforms.addLast(node.backwardStep.content.getData(time, node.content));
                    node = node.backwardStep;
                }
                
                // create forward list (from FRAME to target)
                node = frameNode;
                while(node.content != targetNode.content) {
                    forwardTransforms.addLast(node.forwardStep.content.getData(time, node.content));
                    node = node.forwardStep;
                }
                return true;
            }
            
            // expand search node
            for(Frame parentFrame : frame.getParentFrames()) {                
                SearchNode<Frame> parentFrameNode = frameToNode.get(parentFrame);
                                
                boolean addToQueue = false;
                if (parentFrameNode == null) {
                    // node was not yet visited
                    parentFrameNode = new SearchNode<Frame>(parentFrame);                    
                    frameToNode.put(parentFrame, parentFrameNode);
                    addToQueue = true;
                } else {
                    // node is already visited
                    if ((parentFrameNode.backwardStep == null && frameNode.forwardStep == null)
                            || (parentFrameNode.forwardStep == null && frameNode.backwardStep == null)) {
                        // node was visited, but from other direction.
                        // create new search node that represents this frame, visited from both sides
                        // this allows the other search node of this frame to still be expanded first                        
                        parentFrameNode = new SearchNode<Frame>(parentFrameNode);
                        addToQueue = true;                        
                    }
                }                
                                
                // add search node belonging to parent frame to the queue
                if (addToQueue) {
                    // determine cost (based on max absolute difference in time stamp) 
                    TimeCache cache = frame.getTimeCache(parentFrame);
                    parentFrameNode.cost = Math.max((double)cache.timeToNearestTransform(time),
                                                Math.max(parentFrameNode.cost, frameNode.cost));
                    // if visiting forward (from source), set backward step to remember path 
                    if (frameNode.backwardStep != null) parentFrameNode.backwardStep = frameNode;
                    // if visiting backward (from target), set forward step to remember path
                    if (frameNode.forwardStep != null) parentFrameNode.forwardStep = frameNode;
                    // add node to queue
                    Q.add(parentFrameNode);
                }
            }
        }    
        
        // target and source frames are not connected.        
        return false;
    }
    
    /**
     * Wrapper search node that can be used for bi-directional best-first search. 
     * Keeps track of search path by maintaining links to parent nodes, in both directions
     * (i.e., from source and from target node).
     * 
     * @author Sjoerd van den Dries
     * @param <V> Content type of the search node
     */    
    protected class SearchNode<V> implements Comparable<SearchNode<V>> {        
        /** Content of search node */
        V content;
        /** Cost of path up and until this search node */
        double cost;
        /** Refers to parent node in forward path */
        SearchNode<V> backwardStep;
        /** Refers to parent node in backward path */
        SearchNode<V> forwardStep;        
        
        /** Default constructor; sets specified content and cost to 0, steps to null. */
        SearchNode(V content) {
            this.content = content;
            this.cost = 0;
            this.backwardStep = null;
            this.forwardStep = null;
        }
        
        /** Copy constructor */
        SearchNode(SearchNode<V> orig) {
            this.content = orig.content;
            this.cost = orig.cost;
            this.backwardStep = orig.backwardStep;
            this.forwardStep = orig.forwardStep;
        }
        
        /** Comparator method: low cost < high cost. */
        public int compareTo(SearchNode<V> other) {
            if (this.cost < other.cost) return -1;
            if (this.cost > other.cost) return 1;
            return 0;
        }        
        
    }
	
    /* **********************************************************************
     * *                          HELPER METHODS                            *
     * ********************************************************************** */	
	
//	/**
//	 * Converts the given TransformStamped message to the TransformStorage datastructure
//	 */	
//	protected TransformStorage transformStampedMsgToTF(TransformStamped msg) {
//	    ros.pkg.geometry_msgs.msg.Vector3 tMsg = msg.transform.translation;
//	    ros.pkg.geometry_msgs.msg.Quaternion rMsg = msg.transform.rotation;	
//	    
//	    // add frames to map
//	    Frame childFrame = lookupOrInsertFrame(msg.child_frame_id);
//	    Frame parentFrame = lookupOrInsertFrame(msg.header.frame_id);
//	    
//	    return new TransformStorage(new Vector3d(tMsg.x, tMsg.y, tMsg.z),
//	                                new Quat4d(rMsg.x, rMsg.y, rMsg.z, rMsg.w),
//	                                msg.header.stamp.totalNsecs(),
//	                                parentFrame, childFrame);
//	}
	
    /**
     * Converts the given TransformStorage datastructure to a TransformStamped message
     */ 	
	protected TransformStamped TFToTransformStampedMsg(TransformStorage tf) {
	    Vector3d tTF = tf.getTranslation();
	    Quat4d rTF = tf.getRotation();
	    
	    // convert quaternion and translation vector to corresponding messages
	    ros.pkg.geometry_msgs.msg.Vector3 tMsg = new ros.pkg.geometry_msgs.msg.Vector3();
	    ros.pkg.geometry_msgs.msg.Quaternion rMsg = new ros.pkg.geometry_msgs.msg.Quaternion();
	    tMsg.x = tTF.x; tMsg.y = tTF.y; tMsg.z = tTF.z;
        rMsg.x = rTF.x; rMsg.y = rTF.y; rMsg.z = rTF.z; rMsg.w = rTF.w;
        
        // create TransformStamped message
	    TransformStamped msg = new TransformStamped();	    
	    msg.header.frame_id = tf.getParentFrame().getFrameID();
	    msg.header.stamp = new Time(tf.getTimeStamp());
	    msg.child_frame_id = tf.getChildFrame().getFrameID();
	    msg.transform.translation = tMsg;
	    msg.transform.rotation = rMsg;
	    
	    return msg;
	}
	
	/**
	 * Converts the TransformStorage datastructure (represented by quaternion and vector) to
	 * the StampedTransform datastructure (represented by a 4x4 matrix)
	 */
	protected StampedTransform StorageToStampedTransform(TransformStorage ts) {
	    return new StampedTransform(ts.getTranslation(), ts.getRotation(), new Time(ts.getTimeStamp()), 
	                    ts.getParentFrame().getFrameID(), ts.getChildFrame().getFrameID());
	}
	
	/**
	 * Returns the resolved version of the given frame ID, and asserts a debug message if the name
	 * was not fully resolved.
     */ 	
	private String assertResolved(String prefix, String frameID) {
	  if (!frameID.startsWith("/"))	    
	      System.err.println("TF operating on not fully resolved frame id " + frameID +", resolving using local prefix " + prefix);
	  return resolve(prefix, frameID);
	}
	
	/**
     * Returns the resolves version of the given frame ID.
	 */	
	private static String resolve(String prefix, String frameID)	{			    
	    if (frameID.startsWith("/")) {
			return frameID;			
		}	  
		
		if (prefix.length() > 0) {
			if (prefix.startsWith("/")) {
				return prefix + "/" + frameID;
			} else {
			    return "/" + prefix + "/" + frameID;
			}
		}  else {		    
			return "/" + frameID;			
		}
	}	
	
	/* Returns the tf prefix from the parameter list
	 * 
	 * TODO: does not work yet
	private static String getPrefixParam(NodeHandle nh) {
		String param; 
		if (!nh.hasParam("tf_prefix")) return ""; 
		
		try {
			return nh.getStringParam("tf_prefix", false);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return "";
	}
	*/	
	
	
	public static void main(String[] args) {
		TFMemory mem = TFMemory.getInstance();
		
		mem.setTransforms("[ { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/base_link' }, 'child_frame_id': '/base_bellow_link', 'transform': { 'translation': { 'x': -0.29, 'y': 0, 'z': 0.8 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/base_footprint' }, 'child_frame_id': '/base_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0.051 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/base_link' }, 'child_frame_id': '/base_laser_link', 'transform': { 'translation': { 'x': 0.275, 'y': 0, 'z': 0.252 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/sensor_mount_link' }, 'child_frame_id': '/double_stereo_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_plate_frame' }, 'child_frame_id': '/head_mount_link', 'transform': { 'translation': { 'x': -0.138, 'y': 0, 'z': 0.091 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_link' }, 'child_frame_id': '/head_mount_kinect_ir_link', 'transform': { 'translation': { 'x': -0.020959, 'y': 0.0140592, 'z': 0.152398 }, 'rotation': { 'x': -0.004228700956137955, 'y': 0.01751558140832542, 'z': 0.002824068643841697, 'w': 0.9998336597316809 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_kinect_ir_link' }, 'child_frame_id': '/head_mount_kinect_ir_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0.01, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_kinect_ir_link' }, 'child_frame_id': '/head_mount_kinect_rgb_link', 'transform': { 'translation': { 'x': -0.014501435534, 'y': -0.026453164382, 'z': -0.017792135969 }, 'rotation': { 'x': -6.965361194570848E-4, 'y': -0.01615112986346554, 'z': -8.531262056731336E-5, 'w': 0.9998693157424747 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_kinect_rgb_link' }, 'child_frame_id': '/head_mount_kinect_rgb_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_link' }, 'child_frame_id': '/head_mount_prosilica_link', 'transform': { 'translation': { 'x': -0.046457, 'y': 0.0125, 'z': 0.088921 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_mount_prosilica_link' }, 'child_frame_id': '/head_mount_prosilica_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_tilt_link' }, 'child_frame_id': '/head_plate_frame', 'transform': { 'translation': { 'x': 0.0232, 'y': 0, 'z': 0.0645 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/sensor_mount_link' }, 'child_frame_id': '/high_def_frame', 'transform': { 'translation': { 'x': 0.036168907185, 'y': -0.109387946422, 'z': 0.048769491003 }, 'rotation': { 'x': -0.002604276010305506, 'y': -0.001221411217240444, 'z': 0.003046509185294354, 'w': 0.9999912223029184 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/high_def_frame' }, 'child_frame_id': '/high_def_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/torso_lift_link' }, 'child_frame_id': '/imu_link', 'transform': { 'translation': { 'x': -0.02977, 'y': -0.1497, 'z': 0.164 }, 'rotation': { 'x': 0, 'y': 1, 'z': 0, 'w': -1.034115555773735E-13 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_forearm_roll_link' }, 'child_frame_id': '/l_forearm_cam_frame', 'transform': { 'translation': { 'x': 0.143848881808, 'y': 0.002655790759, 'z': 0.034734323537 }, 'rotation': { 'x': -0.6682498961054247, 'y': -0.2011510496384046, 'z': -0.2021043574082913, 'w': 0.6871201934894933 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_forearm_cam_frame' }, 'child_frame_id': '/l_forearm_cam_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_forearm_roll_link' }, 'child_frame_id': '/l_forearm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_gripper_palm_link' }, 'child_frame_id': '/l_gripper_led_frame', 'transform': { 'translation': { 'x': 0.0513, 'y': 0, 'z': 0.0244 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_gripper_palm_link' }, 'child_frame_id': '/l_gripper_motor_accelerometer_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_wrist_roll_link' }, 'child_frame_id': '/l_gripper_palm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_gripper_palm_link' }, 'child_frame_id': '/l_gripper_tool_frame', 'transform': { 'translation': { 'x': 0.18, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/torso_lift_link' }, 'child_frame_id': '/l_torso_lift_side_plate_link', 'transform': { 'translation': { 'x': 0.0535, 'y': 0.209285, 'z': 0.176625 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/l_upper_arm_roll_link' }, 'child_frame_id': '/l_upper_arm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/laser_tilt_mount_link' }, 'child_frame_id': '/laser_tilt_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0.027634486223 }, 'rotation': { 'x': 0, 'y': 0, 'z': -0.002977759453824769, 'w': 0.9999955664644895 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/double_stereo_link' }, 'child_frame_id': '/narrow_stereo_link', 'transform': { 'translation': { 'x': 0.035991178502, 'y': 0.059246998554, 'z': 0.045054750998 }, 'rotation': { 'x': 6.013579587989879E-4, 'y': -0.003661284031716345, 'z': 0.002667500797814703, 'w': 0.9999895588491603 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/narrow_stereo_link' }, 'child_frame_id': '/narrow_stereo_l_stereo_camera_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/narrow_stereo_l_stereo_camera_frame' }, 'child_frame_id': '/narrow_stereo_l_stereo_camera_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/narrow_stereo_link' }, 'child_frame_id': '/narrow_stereo_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/narrow_stereo_l_stereo_camera_frame' }, 'child_frame_id': '/narrow_stereo_r_stereo_camera_frame', 'transform': { 'translation': { 'x': 0, 'y': -0.09, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/narrow_stereo_r_stereo_camera_frame' }, 'child_frame_id': '/narrow_stereo_r_stereo_camera_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/projector_wg6802418_frame' }, 'child_frame_id': '/projector_wg6802418_child_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': -0.7071067811848163, 'z': 0, 'w': 0.7071067811882787 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_plate_frame' }, 'child_frame_id': '/projector_wg6802418_frame', 'transform': { 'translation': { 'x': 0, 'y': 0.11, 'z': 0.0546 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_forearm_roll_link' }, 'child_frame_id': '/r_forearm_cam_frame', 'transform': { 'translation': { 'x': 0.13761237131, 'y': -0.008475037622, 'z': 0.045627509593 }, 'rotation': { 'x': 0.6766397397764536, 'y': -0.1824717311971638, 'z': 0.1814822789956756, 'w': 0.689874562713903 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_forearm_cam_frame' }, 'child_frame_id': '/r_forearm_cam_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_forearm_roll_link' }, 'child_frame_id': '/r_forearm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_gripper_palm_link' }, 'child_frame_id': '/r_gripper_led_frame', 'transform': { 'translation': { 'x': 0.0513, 'y': 0, 'z': 0.0244 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_gripper_palm_link' }, 'child_frame_id': '/r_gripper_motor_accelerometer_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_wrist_roll_link' }, 'child_frame_id': '/r_gripper_palm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_gripper_palm_link' }, 'child_frame_id': '/r_gripper_tool_frame', 'transform': { 'translation': { 'x': 0.18, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/torso_lift_link' }, 'child_frame_id': '/r_torso_lift_side_plate_link', 'transform': { 'translation': { 'x': 0.0535, 'y': -0.209285, 'z': 0.176625 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/r_upper_arm_roll_link' }, 'child_frame_id': '/r_upper_arm_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/head_plate_frame' }, 'child_frame_id': '/sensor_mount_link', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/double_stereo_link' }, 'child_frame_id': '/wide_stereo_link', 'transform': { 'translation': { 'x': 0.039657926503, 'y': 0.029040003222, 'z': 0.044411499643 }, 'rotation': { 'x': 3.875942696743064E-4, 'y': -0.004943502678088352, 'z': 0.01228578189191559, 'w': 0.9999122317059923 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/wide_stereo_link' }, 'child_frame_id': '/wide_stereo_l_stereo_camera_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/wide_stereo_l_stereo_camera_frame' }, 'child_frame_id': '/wide_stereo_l_stereo_camera_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/wide_stereo_link' }, 'child_frame_id': '/wide_stereo_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/wide_stereo_l_stereo_camera_frame' }, 'child_frame_id': '/wide_stereo_r_stereo_camera_frame', 'transform': { 'translation': { 'x': 0, 'y': -0.09, 'z': 0 }, 'rotation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 } } }, { 'header': { 'seq': 0, 'stamp': { '$date': '1970-01-04T02:43:21.169Z' }, 'frame_id': '/wide_stereo_r_stereo_camera_frame' }, 'child_frame_id': '/wide_stereo_r_stereo_camera_optical_frame', 'transform': { 'translation': { 'x': 0, 'y': 0, 'z': 0 }, 'rotation': { 'x': -0.5, 'y': 0.4999999999975517, 'z': -0.5, 'w': 0.5000000000024483 } } } ]");
		
		int sec = ISO8601Date.parse("1970-01-04T02:43:21.169Z").getDate().getSeconds();
		StampedTransform tf = mem.lookupTransform("/base_link", "/base_bellow_link", new Time(sec));
		
		System.err.println(tf.toString());
	}
	
}
