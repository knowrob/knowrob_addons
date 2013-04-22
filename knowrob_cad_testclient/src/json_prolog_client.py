#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()

    print("Load and analyze CAD model: ")
    query = prolog.query("owl_parse('/home/tenorth/testPiece.owl', false, false, true); findall(Part, rdf_triple(knowrob:properPhysicalParts, knowrob:verticalPiece1, Part), Parts)")
    for solution in query.solutions():
        print 'OK '
    query.finish()

    print "\n\n\n\n\n"

    print("Read all cones, their poses and radii:")
    query = prolog.query("rdfs_individual_of(Cone, knowrob:'Cone'), rdf_triple(knowrob:radius, Cone, Rl), strip_literal_type(Rl, Radius), current_object_pose(Cone, Pose)")
    for solution in query.solutions():
        print 'Found cone:\n Instance = %s\n Radius = %s\n Pose = %s\n' % (solution['Cone'], solution['Radius'], solution['Pose'])
    query.finish()
