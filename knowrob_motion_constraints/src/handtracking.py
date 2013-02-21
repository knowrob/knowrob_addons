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
roslib.load_manifest('json_prolog')
import json_prolog
import rospy
import tf

from std_msgs.msg import Header
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

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
        self.features = features
        
    def iterFeaturePoses(self, rot_mat):
        for feature in self.features:
            pos = numpy.matrix(feature.pos.tolist()+[1]).transpose()
            pos = (rot_mat * pos)[:2]
            dir = inv(feature.dir[0:3,0:3])
            yield Feature(feature.name, feature.type, pos, dir)

class Feature():
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

concept_map = {'plate.obj': 'knowrob:DinnerPlate',
               'spatula.obj': 'knowrob:Spatula',
               'pancake.obj': 'knowrob:Pancake',
               'mixer.obj': 'knowrob:PancakeMaker'}

if __name__ == '__main__':
    # extract frames
    scenario = 'scenario1'
    frames = map(lambda f: scipy.loadmat(os.path.join('..', 'data', scenario,f)), sorted(fnmatch.filter(os.listdir(os.path.join('..', 'data', scenario)), 'frame*.mat')))
    solutions = map(lambda f: scipy.loadmat(os.path.join('..', 'data', scenario,f)), sorted(fnmatch.filter(os.listdir(os.path.join('..', 'data', scenario)), 'solution*.mat')))   

    # extract object classes from data
    meshes = scipy.loadmat(os.path.join(scenario, 'mesh_filenames'))['filenames']
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
        query = knowrob.query("features(ObjT, FeatureT, Pos, Dir).")
        counters = [1] * 3
        for s in query.solutions():
            featureT = s['FeatureT']
            pos = numpy.matrix(s['Pos']).transpose()
            dir = numpy.matrix(s['Dir']).transpose()
            feat_name = '%s/%s.%d'%(obj.name,counters[featureT])
            f = Feature(featureT, feat_name, pos, dir)
            print '  Feature: %s, %s, pos:%s, dir:%s' % (feat_name, Feature.NAMES[featureT], str(pos), str(dir))
            counters[featureT] += 1
    
    # start ros node and broadcast all poses for each single frame
    pub = rospy.Publisher('handtracking', PoseStamped)
    rospy.init_node('handtracking')
    for frame_idx, frame in enumerate(frames):
        rot_mats = frame['mats']
        timestamp = Time(time.time())
        for i, (obj, rot_mat) in enumerate(zip(object_store, rot_mats)):
            if len(obj.features) == 0: continue
            obj_pos = rot_mat[0:3,3]
            obj_dir = rot_mat[0:3,0:3]
            for feature in obj.iterFeaturePoses(obj_pos, obj_dir):
                header = Header(frame_idx, timestamp, feature.name)
                position = Pose(Point(feature.pos))
                orientation = Quaternion(feature.dir)
                pose = Pose(position, orientation)
                pub.publish(PoseStamped(header, pose))
        rospy.sleep(1.0)

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
        

    
