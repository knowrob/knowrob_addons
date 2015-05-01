#!/usr/bin/env python
# Author: Daniel Bessler

import roslib
import rospy
import tf
import sys
import argparse

# Handle command line arguments
parser = argparse.ArgumentParser(description='Read CSV file and publish TF messages.')
parser.add_argument('--root-frame', type=str, default='mocap', help='the root TF frame')
parser.add_argument('--tf-topic-out', type=str, default='tf', help='the published TF topic')
parser.add_argument('--tf-topic-in', type=str, default='tf_old', help='the subscribed TF topic')
args = parser.parse_args()

print("Publishing TF messages on topic: '" + args.tf_topic_out + "'")
print("Frame ID of published messages: '" + args.root_frame + "'")
print("Subscribing TF messages from topic '" + args.tf_topic_in + "'")

rospy.init_node("tf_restamper")
tfpublisher= rospy.Publisher(args.tf_topic_out, tf.msg.tfMessage)

hasStamp = False

def tfcallback(tfmessage):
    global hasStamp
    for transf in tfmessage.transforms:
        transf.header.stamp=rospy.Time.now() + rospy.Duration(0.5)
        if hasStamp==False:
            # TODO: HACK: str(..) also in csv publisher
            print("First timepoint: " + str(float(str(transf.header.stamp))/1000000000.0))
            hasStamp = True
        transf.header.frame_id = args.root_frame
    tfpublisher.publish(tfmessage)

tfproxy = rospy.Subscriber(args.tf_topic_in, tf.msg.tfMessage,tfcallback)
rospy.spin()

