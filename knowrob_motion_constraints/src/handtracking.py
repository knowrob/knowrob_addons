#!/usr/bin/env python
'''
Created on Feb 19, 2013

@author: nyga
'''
import scipy.io as scipy
import os
import fnmatch
import numpy
from numpy.linalg import inv
import roslib
roslib.load_manifest('knowrob_motion_constraints')
import json_prolog
import rospy
import tf

from std_msgs.msg import Header
from std_msgs.msg import Time
#from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Pose
#from geometry_msgs.msg import Point
#from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from constraint_msgs.msg import Feature

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import time

class Object():
    '''
    Class representing a tracked object, which has a certain amount
    of itasc features (i.e., line, plane or point)
    '''
    def __init__(self, name, features=None):
        self.name = name
        if features is None:
            self.features = []
        else:
            self.features = features
        
    def iterFeaturePoses(self, rot_mat):
        for feature in self.features:
            
            pos = numpy.vstack((feature.pos,[1.0],))
            #print pos, type(pos)
            #print rot_mat, type(rot_mat)
            pos = (rot_mat * pos)[:3]
            #print rot_mat[0:3,0:3]
            #print inv(rot_mat[0:3,0:3].transpose())
            #print feature.dir
            dir = rot_mat[0:3,0:3].transpose() * feature.dir
            yield ITasCFeature(feature.name, feature.type, pos, dir)

class ITasCFeature():
    '''
    Represents an itasc feature and provides methods for transforming
    its coordinates relative to the parent object.
    '''
    LINE = 0
    PLANE = 1
    POINT = 2    
    NAMES = {0: 'line', 1: 'plane', 2: 'point'}
    
    def __init__(self, name, type, pos, dir):
        self.name = name
        self.type = type
        self.pos = pos
        self.dir = dir

concept_map = {'plate.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#DinnerPlate'",
               'spatula.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Spatula'",
               'pancake.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#Pancake'",
               'mixer.obj': "'http://ias.cs.tum.edu/kb/knowrob.owl#PancakeMaker'"}

if __name__ == '__main__':
    # extract frames
    scenario = 'scenario1'
    frames = map(lambda f: scipy.loadmat(os.path.join('..', 'data', scenario,f)), sorted(fnmatch.filter(os.listdir(os.path.join('..', 'data', scenario)), 'frame*.mat')))
    solutions = map(lambda f: scipy.loadmat(os.path.join('..', 'data', scenario,f)), sorted(fnmatch.filter(os.listdir(os.path.join('..', 'data', scenario)), 'solution*.mat')))   

    # extract object classes from data
    meshes = scipy.loadmat(os.path.join('..', 'data', scenario, 'mesh_filenames'))['filenames']
    classes = []
    for m in meshes:
        for p in m: pass
        print p[0]
        obj = p[0].split('\\')[-1]
        classes.append(obj)
    print 'object classes:'
    print classes
    
    # extract objects    
    objects2type = {}
    objects = []
    for inst, issue in zip(frames[0]['instance_ids'], frames[0]['issue_ids']):
        for inst_number in inst: inst_number -= 1
        for issue_number in issue: issue_number -= 1
        obj_name = '%s_%d' % (classes[issue_number], inst_number)
        objects2type[obj_name] = classes[issue_number]
        objects.append(obj_name)
    print 'objects:'
    print objects
    
    # start json-prolog         
    rospy.init_node('tracking2knowrob', anonymous=True)
    knowrob = json_prolog.Prolog()
 
    # ask knowrob for object features
    object_store = []
    for o in objects:
        o_type = objects2type[o]
        obj = Object(o)
        object_store.append(obj)
        if not o_type in concept_map: continue
        print 'Object: %s' % obj.name
        query = knowrob.query("object_feature(%s, ITasCFeatureT, Pos, Dir)" % concept_map[o_type])
        counters = [1] * 3
        for s in query.solutions():
            featureT = s['ITasCFeatureT']
            pos = numpy.matrix(s['Pos']).transpose()
            dir = numpy.matrix(s['Dir']).transpose()
            feat_name = '%s/%s.%d'%(obj.name,ITasCFeature.NAMES[featureT],counters[featureT])
            f = ITasCFeature(featureT, feat_name, pos, dir)
            obj.features.append(f)
            print '  ITasCFeature: %s, %s, pos:%s, dir:%s' % (feat_name, ITasCFeature.NAMES[featureT], str(pos), str(dir))
            counters[featureT] += 1
    
    # start ros node and broadcast all poses for each single frame
    pub = rospy.Publisher('handtracking', Feature)
    #rospy.init_node('handtracking')
    for frame_idx, frame in enumerate(frames):
        rot_mats = frame['mats']
        timestamp = Time(time.time())
        for i, (obj, rot_mat) in enumerate(zip(object_store, rot_mats)):
            if len(obj.features) == 0: continue
            #obj_pos = rot_mat[0:3,3]
            #obj_dir = rot_mat[0:3,0:3]
            for feature in obj.iterFeaturePoses(rot_mat):
                msg_frame = 'ref_frame_name'
                pos_vect = Vector3(float(feature.pos[0]), float(feature.pos[1]), float(feature.pos[2]))
                dir_vect = Vector3(float(feature.dir[0]), float(feature.dir[1]), float(feature.dir[2]))
                contactdir_vect = Vector3(0, 0, 0)
                feat_msg = Feature(msg_frame, feature.type, feature.name, pos_vect, dir_vect, contactdir_vect)
                print "publishing... "
                print feat_msg
                pub.publish(feat_msg)
                #header = Header(frame_idx, timestamp, feature.name)
                #pos_coord = map(lambda x: '%d'%x[0], feature.pos.tolist())
                #position = Point(pos_coord[0], pos_coord[1], pos_coord[2])
                #dir_coord = map(lambda x: '%d'%x[0], feature.dir.tolist())
                #print dir_coord
                #orientation = Quaternion(dir_coord[0], dir_coord[1], dir_coord[2], dir_coord[3])
                #pose = Pose(position, orientation)
        rospy.sleep(0.5)

#    fig = plt.figure()
#    ax = fig.gca(projection='3d')
#    x1=[]
#    y1=[]
#    z1=[]
#    x2=[]
#    y2=[]
#    z2=[]
#    for frame_idx, frame in enumerate(frames):
#        rot_mats = frame['mats']
#        
#        rot_mat1 = rot_mats[objects.index('spatula.obj_0')]
#        x1.append(rot_mat1[0,3])
#        y1.append(rot_mat1[1,3])
#        z1.append(rot_mat1[2,3])
#        
#        rot_mat2 = rot_mats[objects.index('pancake.obj_0')]
#        x2.append(rot_mat2[0,3])
#        y2.append(rot_mat2[1,3])
#        z2.append(rot_mat2[2,3])
#        
#    ax.plot(x1, y1, z1, color='red' )
#    ax.plot(x2, y2, z2, color='blue' )
#    
#    plt.show()
#        
        

    
