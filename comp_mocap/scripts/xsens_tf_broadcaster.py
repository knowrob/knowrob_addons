#!/usr/bin/env python  
# Author: Benjamin Brieber

import roslib
#roslib.load_manifest('xsens_tf_broadcaster')
import rospy

import tf
import xsens_client
import numpy as np

from tf import transformations as tft

import signal
import sys

##This class transforms data from xsens to tf data                      
class xsens_tf_viewer:

  def __init__(self, ref_frame,tf_prefix='xsens_human', relative=True):
    self.br = tf.TransformBroadcaster()
    self.tr = tf.TransformerROS()
    self.ref_frame = ref_frame
    self.relative = relative
    self.relative = relative

  #called on incomming data
  def update(self, subject):
    if(self.relative):
      self.publish_relative_frames(subject)
    else:
      self.publish_global_frames(subject)

  ##used for global transormations of each segment
  def publish_global_frames(self, data):
    for segment in data.segments:
      if(segment.is_quaternion):
	    self.sendTransform(segment.position,
			segment.quat,
			rospy.Time.now(),
			xsens_client.get_segment_name(segment.id),
			xsens_client.get_segment_parent_name(1,self.ref_frame))
      else:	  
	    print "euler"
	    self.sendTransform(segment.position,
			tf.transformations.quaternion_from_euler(segment.euler),
			rospy.Time.now(),
			xsens_client.get_segment_name_by_index(segment.id),
			xsens_client.get_segment_parent_name_by_index(1,self.ref_frame))

  ##used for relative frames (Not working right now)
  def publish_relative_frames2(self, data):
    tmp_dict = {}
    for segment in data.segments:
      if(segment.is_quaternion):
        tmp_dict[segment.id] = (segment.position,segment.quat,self.tr.fromTranslationRotation(segment.position,segment.quat))
      else:	  
        tmp_dict[segment.id] = (segment.position,segment.euler)
  
    for idx in tmp_dict.keys():
      p_idx = xsens_client.get_segment_parent_id(idx)
      child_data = tmp_dict[idx]
      if(p_idx==-1):
        continue
      elif(p_idx==0):
        helper = tft.quaternion_about_axis(1,(-1,0,0))
        new_quat = tft.quaternion_multiply(tft.quaternion_inverse(helper),child_data[1])#if(segment.is_quaternion): TODO Handle Euler
        #self.sendTransform(child_data[0],
        self.sendTransform(child_data[0],
            child_data[1],
            #(1.0,0,0,0),#FIXME
            rospy.Time.now(),
            xsens_client.get_segment_name(idx),
            self.ref_frame)
      elif(p_idx>0):
        
        parent_data = tmp_dict[p_idx]
	new_quat = tft.quaternion_multiply(tft.quaternion_inverse(parent_data[1]),child_data[1])
	tmp_trans = (parent_data[0][0] - child_data[0][0],parent_data[0][1] - child_data[0][1],parent_data[0][2] - child_data[0][2])
        new_trans = qv_mult(parent_data[1],tmp_trans)
        self.sendTransform(
	  new_trans,
	  new_quat,
	  rospy.Time.now(),
	  xsens_client.get_segment_name(idx),
	  xsens_client.get_segment_name(p_idx))

      else:
        parent_data = tmp_dict[p_idx]
        
        this_data = np.multiply(tft.inverse_matrix(child_data[2]) , parent_data[2])
        self.sendTransform(
	  tft.translation_from_matrix(this_data),
	  tft.quaternion_from_matrix(this_data),
	  rospy.Time.now(),
	  xsens_client.get_segment_name(idx),
	  xsens_client.get_segment_name(p_idx))
	
    
  def publish_relative_frames(self, data):
    tmp_dict = {}
    for segment in data.segments:
      if(segment.is_quaternion):
        tmp_dict[segment.id] = (segment.position,segment.quat)
      else:	  
        tmp_dict[segment.id] = (segment.position,segment.euler)
    
    for idx in tmp_dict.keys():
      p_idx = xsens_client.get_segment_parent_id(idx)
      child_data = tmp_dict[idx]
      if(p_idx==-1):
        continue
      elif(p_idx==0):
        new_quat = tft.quaternion_multiply(child_data[1],tft.quaternion_inverse((1,1,1,1)))#if(segment.is_quaternion): TODO Handle Euler
        #self.sendTransform(child_data[0],
        self.sendTransform(child_data[0],
            child_data[1],
            #(1.0,0,0,0),#FIXME
            rospy.Time.now(),
            xsens_client.get_segment_name(idx),
            self.ref_frame)
      else:
        parent_data = tmp_dict[p_idx]
        parent_transform = tf.Transform(parent_data[1],parent_data[0])
        child_transform = tf.Transform(child_data[1],child_data[0])
        new_trans = parent_transform.inversetimes(child_transform)
        new_quat = tft.quaternion_multiply(child_data[1],tft.quaternion_inverse(parent_data[1]))
        tmp_pos = (child_data[0][0]-parent_data[0][0],child_data[0][1]-parent_data[0][1] ,child_data[0][2]-parent_data[0][2] )
        #tmp_pos = (child_data[0][0] - parent_data[0][0],child_data[0][1] - parent_data[0][1],child_data[0][2] - parent_data[0][2])
        #self.sendTransform(qv_mult(parent_data[1],tmp_pos),
        #self.sendTransform(tmp_pos,
    	#		new_quat,
          #child_data[1],
        self.sendTransform(new_trans.getOrigin(), new_trans.getRotation(),rospy.Time.now(),xsens_client.get_segment_name(idx),xsens_client.get_segment_name(p_idx))

  def sendTransform(self, pos, rot, t, name, frame):
    self.br.sendTransform(pos, rot, t, name, frame)
    
def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    q3 = tft.quaternion_multiply(q1, q2)
    return tft.quaternion_multiply(q3, tft.quaternion_conjugate(q1))[1:]
    #return tft.quaternion_multiply(q1, q2)[1:]

##Main starting point used for tf publishing
if __name__ == '__main__':
    rospy.init_node('xsens_tf_broadcaster')
    #tf_frame = rospy.get_param('~tf_frame')
    tf_frame = 'xsens'#TODO fix this
    client = xsens_client.xsens_client(ip='192.168.100.171',port=9763)
    client.connect()
    viewer = xsens_tf_viewer(tf_frame,relative=False)
    #viewer = xsens_tf_viewer(tf_frame)
    client.attach(viewer)
    while not rospy.is_shutdown():
      client.read_data()
    
    
    
