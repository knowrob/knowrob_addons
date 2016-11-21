#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    
    query_str = (
            "u_load,"
            "set_ep,"
            "grasp(Obj, [STs,ETs]),"
            "class(Obj, knowrob:'Bowl'),"
            "iri_xml_namespace(Obj, _, ObjShortName),"
            "actor_pose(ObjShortName, STs, Pose)"
        )
                        
    query = prolog.query(query_str)
    for solution in query.solutions():
        print 'Found solution. Pose[x,y,z,qw,qx,qy,qz] = %s' % (solution['Pose'])
    query.finish()
