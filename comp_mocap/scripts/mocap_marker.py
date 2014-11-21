#!/usr/bin/env python
# Author: Daniel Bessler

import tf
import time
import numpy
import sys
import argparse
from math import sqrt

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class MocapMarkerPublisher:
    """
    Mapping of TF messages to marker messages
    for visualization purpose.
    """
    
    MARKER_NAMESPACE = "mocap_marker"
    
    def __init__(self):
        self.listener_rate = 10.0
        self.marker_color = (1.0,1.0,0.0,1.0)
        self.suffixes = [""]
        # The root frame (no parent frame) of the scene.
        self.tf_root_frame='/map'
        
        rospy.init_node('tf_mocap')
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
    
    def run(self):
        """ Mapping of TF messages to marker messages. Runs infinite. """
        rate = rospy.Rate(self.listener_rate)
        while not rospy.is_shutdown():
            self.__handleTfFrames__()
            rate.sleep()
    
    def __handleTfFrames__(self):
        """ Lookup current transform, create markers and publish them. """

        # Clear the marker array
        self.markerArray = MarkerArray()
        index = 0
        # Create cylinders between links
        for (child,connections,_) in self.getSkeleton():
            for suff in self.suffixes:
                for conn in connections:
                    try:
                        self.__addConnection__(self.tf_root_frame,child+str(suff),conn+str(suff),index)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                        #print("Lookup Failed! Reason: " + str(exc))
                        self.__deleteMarker__(child+str(suff),index)
                    index += 1
        # Create spheres at links
        for (child,_,rad) in self.getSkeleton():
            for suff in self.suffixes:
                try:
                    self.__addSphere__(self.tf_root_frame,child+str(suff),rad,index)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                    #print("Lookup Failed! Reason: " + str(exc))
                    self.__deleteMarker__(child+str(suff),index)
                index += 1
        # Publish markers
        self.publisher.publish(self.markerArray)
    
    def __createMarker__(self,parent,index):
        """ Create a marker. Namespace is set to 'mocap'. """
        marker = Marker()
        marker.ns = self.MARKER_NAMESPACE
        marker.id = index
        marker.header.frame_id = parent
        marker.action = marker.ADD
        marker.color.a = self.marker_color[3]
        marker.color.r = self.marker_color[0]
        marker.color.g = self.marker_color[1]
        marker.color.b = self.marker_color[2]
        return marker
    
    def __deleteMarker__(self,parent,index):
        m = self.__createMarker__(parent,index)
        m.action = m.DELETE
        self.markerArray.markers.append(m)
        
    def __computeOrientation__(self,u,v):
        """ Comoutes orientation between links. """
        a = numpy.cross(u,v)
        w = sqrt( (numpy.linalg.norm(u)**2) * (numpy.linalg.norm(v)**2) ) + u.dot(v)
        out = numpy.array((a[0], a[1], a[2], w))
        return out / numpy.linalg.norm(out)
    
    def __addConnection__(self,parent,child0,child1,index):
        """ Create a cylinder marker. """
        (trans0,rot0) = self.listener.lookupTransform(parent,child0,rospy.Time(0))
        (trans1,rot1) = self.listener.lookupTransform(parent,child1,rospy.Time(0))
        marker = self.__createMarker__(parent, index)
        marker.type = marker.CYLINDER
        
        # Compute center of the cylinder and height
        p0 = numpy.array(trans0)
        p1 = numpy.array(trans1)
        distance = numpy.linalg.norm(p0-p1)
        center = (p0+p1)*0.5
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = distance
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        
        # Compute orientation of vylinder based on adjacent links
        up = numpy.array((0,0,1))
        actual = p1-p0
        orient = self.__computeOrientation__(up, actual)
        marker.pose.orientation.x = orient[0]
        marker.pose.orientation.y = orient[1]
        marker.pose.orientation.z = orient[2]
        marker.pose.orientation.w = orient[3]
        
        self.markerArray.markers.append(marker)
    
    def __addSphere__(self,parent,child,rad,index):
        """ Create a sphere marker. """
        (trans,rot) = self.listener.lookupTransform(parent,child,rospy.Time(0))
        marker = self.__createMarker__(parent, index)
        marker.type = marker.SPHERE
        marker.scale.x = rad
        marker.scale.y = rad
        marker.scale.z = rad
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        self.markerArray.markers.append(marker)

class XsensMarkerPublisher(MocapMarkerPublisher):
    """
    Publishing of visualization markers for XSens data.
    """
    XSENS_SKELETON=[
        ('/Head',            ['/Neck'],                                0.2),
        ('/Neck',            [],                                       0.1),
        ('/RightShoulder',   ['/RightUpperArm'],                       0.1),
        ('/LeftShoulder',    ['/LeftUpperArm'],                        0.1),
        ('/RightUpperArm',   ['/RightForearm'],                        0.1),
        ('/LeftUpperArm',    ['/LeftForearm'],                         0.1),
        ('/RightForearm',    ['/RightHand'],                           0.1),
        ('/LeftForearm',     ['/LeftHand'],                            0.1),
        ('/RightHand',       [],                                       0.15),
        ('/LeftHand',        [],                                       0.15),
        ('/T8',              ['/RightShoulder','/LeftShoulder'],       0.1),
        ('/T12',             ['/T8'],                                  0.1),
        ('/L3',              ['/T12'],                                 0.1),
        ('/L5',              ['/L3'],                                  0.1),
        ('/Pelvis',          ['/RightUpperLeg','/LeftUpperLeg','/L5'], 0.2),
        ('/RightUpperLeg',   ['/RightLowerLeg'],                       0.1),
        ('/LeftUpperLeg',    ['/LeftLowerLeg'],                        0.1),
        ('/RightLowerLeg',   ['/RightFoot'],                           0.1),
        ('/LeftLowerLeg',    ['/LeftFoot'],                            0.1),
        ('/RightFoot',       ['/RightToe'],                            0.1),
        ('/LeftFoot',        ['/LeftToe'],                             0.1),
        ('/RightToe',        [],                                       0.1),
        ('/LeftToe',         [],                                       0.1)
    ]
    def getSkeleton(self):
        return self.XSENS_SKELETON

class OpenniMarkerPublisher(MocapMarkerPublisher):
    """
    Publishing of visualization markers for OPENNI data.
    """
    OPENNI_SKELETON=[
        ### Head
        ('/head',           ['/neck'], 0.2),
        ### Arm
        ('/neck',           ['/right_shoulder','/left_shoulder'], 0.1),
        ('/right_shoulder', ['/right_elbow'],                     0.1),
        ('/left_shoulder',  ['/left_elbow'],                      0.1),
        ('/right_elbow',    ['/right_hand'],                      0.1),
        ('/left_elbow',     ['/left_hand'],                       0.1),
        ('/right_hand',     [],                                   0.15),
        ('/left_hand',      [],                                   0.15),
        ### Leg
        ('/torso',          ['/right_hip','/left_hip','/neck'], 0.1),
        ('/right_hip',      ['/right_knee'],                    0.1),
        ('/left_hip',       ['/left_knee'],                     0.1),
        ('/right_knee',     ['/right_foot'],                    0.1),
        ('/left_knee',      ['/left_foot'],                     0.1),
        ('/right_foot',     [],                                 0.15),
        ('/left_foot',      [],                                 0.15)
    ]
    def getSkeleton(self):
        return self.OPENNI_SKELETON

class CsvMarkerPublisher(MocapMarkerPublisher):
    """
    Publishing of visualization markers for TUM kitchen data set.
    """
    CSV_SKELETON=[
        ('/BEC',  ['/OSR','/OSL','/ULW'], 0.2),
        ('/ULW',  ['/OLW'], 0.1),
        ('/OLW',  ['/UBW'], 0.1),
        ('/UBW',  ['/OBW'], 0.1),
        ('/OBW',  ['/UHW'], 0.1),
        ('/UHW',  ['/BRK'], 0.1),
        ### Head
        ('/KO',   ['/OHW'], 0.2),
        ('/OHW',  ['/BRK'], 0.1),
        ### Arm
        ('/BRK',  ['/SBR','/SBL', '/OHW'], 0.1),
        ('/SBR',  ['/OAR'],                0.1),
        ('/SBL',  ['/OAL'],                0.1),
        ('/OAR',  ['/UAR'],                0.1),
        ('/OAL',  ['/UAL'],                0.1),
        ('/UAR',  ['/HAR'],                0.1),
        ('/UAL',  ['/HAL'],                0.1),
        ('/HAR',  ['/FIR'],                0.1),
        ('/HAL',  ['/FIL'],                0.1),
        ('/FIR',  [],                      0.15),
        ('/FIL',  [],                      0.15),
        ### Leg
        ('/OSR', ['/USR'], 0.1),
        ('/OSL', ['/USL'], 0.1),
        ('/USR', ['/FUR'], 0.1),
        ('/USL', ['/FUL'], 0.1),
        ('/FUR', ['/FBR'], 0.1),
        ('/FUL', ['/FBL'], 0.1),
        ('/FBR', [],       0.1),
        ('/FBL', [],       0.1)
    ]
    def getSkeleton(self):
        return self.CSV_SKELETON

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=
        'Publish visualization marker messages for motion capturing TF messages.')
    parser.add_argument('--skeleton', type=str,
        choices=['xsens', 'openni', 'csv'],
        required=True, help='the skeleton identifier.')
    parser.add_argument('--color', type=float, nargs=4,
        default=[1.0,1.0,0.0,1.0], help='the marker color.')
    parser.add_argument('--root-frame', type=str,
        default='map', help='the root TF frame')
    parser.add_argument('--suffixes', type=str, nargs='+',
        default='', help='suffixes for TF frames')
    args = parser.parse_args()
    
    if args.skeleton=="xsens":
        pub = XsensMarkerPublisher()
    elif args.skeleton=="openni":
        pub = OpenniMarkerPublisher()
    elif args.skeleton=="csv":
        pub = CsvMarkerPublisher()
    else:
        print("Unknown argument: " + str(sys.argv[0]))
        sys.exit(0)
    
    pub.tf_root_frame = "/" + args.root_frame
    pub.marker_color = tuple(args.color)
    pub.suffixes = list(args.suffixes)
    pub.run()

