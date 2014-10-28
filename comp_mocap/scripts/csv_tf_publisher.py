#!/usr/bin/env python
# Author: Daniel Bessler

import roslib
import rospy
import tf
import sys
import os
import csv
import time
import argparse
import copy
from geometry_msgs.msg import TransformStamped

class CsvTfBroadcaster:
    """
    A ROS node that broadcasts TF messages based on
    a CSV file where each row contains motion capturing position data.
    """
    
    def __init__(self, root_frame="mocap", tf_topic="tf"):
        rospy.init_node("csv_tf")
        self.tfpublisher = rospy.Publisher(tf_topic,tf.msg.tfMessage)
        self.pose_msg = TransformStamped()
        self.pose_msg.header.frame_id = root_frame
        self.pose_msg.child_frame_id = ''
        self.pose_msg.transform.translation.x = 0.0
        self.pose_msg.transform.translation.y = 0.0
        self.pose_msg.transform.translation.z = 0.0
        self.pose_msg.transform.rotation.x = 0.0
        self.pose_msg.transform.rotation.y = 0.0
        self.pose_msg.transform.rotation.z = 0.0
        self.pose_msg.transform.rotation.w = 1.0
        self.csvLayout = []
        self.loop = False
        self.line_range = [-1,-1]
    
    def __broadcastLine__(self, row, last_t):
        """
        Processing of a single row of a csv file.
        It is expected that the layout contains a timestamp.
        The timestamp of the row is returned and should be passed to
        the method call for the next row.
        """
        t0     = rospy.Time.now()
        this_t = 0.0
        i      = 0
        
        self.pose_msg.header.stamp = t0
        print("    stamp: " + str(t0))
        # Generate a message for each csv entry that represents a position
        messages = []
        for (n,c) in self.csvLayout:
            val = row[i:i+c]
            i += c
            if n=='time' or n=='t':
                # The real timestamp of this message
                this_t = float(val[0])
            elif c==3 and len(n)>0:
                self.pose_msg.child_frame_id = n
                self.pose_msg.transform.translation.x = float(val[0])/1000.0
                self.pose_msg.transform.translation.y = float(val[1])/1000.0
                self.pose_msg.transform.translation.z = float(val[2])/1000.0
                # TODO: Compute the orientation
                #self.pose_msg.transform.rotation.x = 0.0
                #self.pose_msg.transform.rotation.y = 0.0
                #self.pose_msg.transform.rotation.z = 0.0
                #self.pose_msg.transform.rotation.w = 1.0
                # Collect messages
                messages.append(copy.deepcopy(self.pose_msg))
        # Send all TF messages
        self.tfpublisher.publish(messages)
        t1=rospy.Time.now()
        
        # The time it actually took to generate the messages
        actual  = float(str(t1-t0))/1000000000.0 # HACK: str(..)
        # The time between last frame and this frame
        desired = this_t-last_t
        # Synchronize with timestamps from csv file
        if desired>actual:
            time.sleep(desired - actual)
        else:
            print("WARN! TF messages are not generated fast enough.")
        # return timestamp from csv row
        return this_t
    
    def __gettime__(self,row):
        """
        Returns timestamp associted to the specified CSV row.
        """
        i = 0
        for (n,c) in self.csvLayout:
            val = row[i:i+c]
            i += c
            if n=='time' or n=='t':
                return float(val[0])
        return 0.0

    def __readLayout__(self,row):
        """
        Obtain layout information from CSV row.
        """
        self.csvLayout = []
        i = 0
        while i < len(row):
            elem0 = row[i]
            count = 1
            name = elem0
            
            # handle 3D position data
            if elem0.endswith("X"):
                name = elem0.split("X")[0]
                if row[i+1] == name+"Y" and row[i+2] == name+"Z":
                    count += 2
                
            i += count
            self.csvLayout.append((name,count))

    def __broadcast__(self,csvFilePath):
        csvfile = open(csvFilePath, 'rb')
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        counter = 0
        last_t = 0.0
        # Read line by line
        for row in spamreader:
            # First line describes layout, skip it
            if counter==0:
                self.__readLayout__(row)
            elif counter>self.line_range[0]:
                print("Processing CSV row: " + str(counter))
                last_t = self.__broadcastLine__(row,last_t)
            else:
                last_t = self.__gettime__(row)
            counter += 1
            if self.line_range[1]>0 and counter>self.line_range[1]: break
            if rospy.is_shutdown(): break
        csvfile.close()

    def broadcast(self,csvFilePath):
        """
        Broadcast position data from csv into TF tree.
        Broadcasting is done line by line and synchronized in order
        to match the timestamps from the csv file.
        """
        self.__broadcast__(csvFilePath)
        if self.loop and not rospy.is_shutdown(): self.broadcast(csvFilePath)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read CSV file and publish TF messages.')
    parser.add_argument('--file', type=str, required=True, help='the path to the CSV file')
    parser.add_argument('--loop', type=bool, default=False, help='toggles if the playback should be repeated once finished')
    parser.add_argument('--line-range', type=int, nargs=2,
        default=[-1,-1], help='the range lines in the CSV file.')
    parser.add_argument('--root-frame', type=str, default='mocap', help='the root TF frame')
    parser.add_argument('--tf-topic', type=str, default='tf', help='the TF topic')
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print("ERROR! Not a file: " + cvFile)
        sys.exit(0)
    print("Publishing TF messages with frame id: " + args.root_frame)
    
    broadcaster = CsvTfBroadcaster(args.root_frame, args.tf_topic)
    broadcaster.loop = args.loop
    broadcaster.line_range = args.line_range
    broadcaster.broadcast(args.file)



