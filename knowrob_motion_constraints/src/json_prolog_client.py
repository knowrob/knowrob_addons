#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()

    print("\n\nRead motion phases:\n")
    query = prolog.query("plan_subevents(pancake_constr:'FlippingAPancake', P)")
    for solution in query.solutions():
        print 'Motion Phases = %s\n' % (solution['P'])
    query.finish()

    print "\n\n\n\n\n"

    print("Extracting constraints for first motion phase:\n")
    query = prolog.query("motion_constraint(pancake_constr:'BothSpatulasApproach', C), constraint_properties(C, Type, ToolFeature, WorldFeature, Weight, Lower, Upper, MinVel, MaxVel)")
    for solution in query.solutions():
        print 'Constraint: %s\n Function: %s\n ToolFeature: %s\n WorldFeature: %s\n Weight: %s\n Lower: %s\n Upper: %s\n MinVel: %s\n MaxVel: %s\n' % (solution['C'], solution['Type'], solution['ToolFeature'], solution['WorldFeature'], solution['Weight'], solution['Lower'], solution['Upper'], solution['MinVel'], solution['MaxVel'])
    query.finish()

    print "\n\n\n\n\n"

    print("Extracting feature properties for one of the features:\n")
    query = prolog.query("feature_properties(spatula:'LineFeature_z3rLFrlP', Type, Label, TfFrame, Position, Direction, ContactDirection)")
    for solution in query.solutions():
        print 'Feature:\n Type: %s\n Label: %s\n TfFrame: %s\n Position: %s\n Direction: %s\n ContactDirection: %s\n' % (solution['Type'], solution['Label'], solution['TfFrame'], solution['Position'], solution['Direction'], solution['ContactDirection'])
    query.finish()
