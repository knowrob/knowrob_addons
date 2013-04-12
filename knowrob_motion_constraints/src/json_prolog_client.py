#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()

    print("Read constraint templates:")
    query = prolog.query("plan_constraint_templates(pancake_constr:'FlippingAPancake', _Cs), member(C, _Cs), owl_direct_subclass_of(C, T), rdf_has(T, rdf:type, owl:'Class'),  constraint_properties(C, P, O)")
    for solution in query.solutions():
        print 'Found solution:\n ConstraintTemplate = %s\n Type = %s\n Property = %s\n Value = %s\n' % (solution['C'], solution['T'], solution['P'], solution['O'])
    query.finish()

    print "\n\n\n\n\n"

    print("Read the actual sets of constraints and their properties:")
    query = prolog.query("plan_constraints(pancake_constr:'FlippingAPancake', M, Cs), member(C, Cs), owl_direct_subclass_of(C, T), rdf_has(T, rdf:type, owl:'Class'),  constraint_properties(C, P, _Ol), strip_literal_type(_Ol, O)")
    for solution in query.solutions():
        print 'Found solution:\n MotionSegment = %s\n Constraint = %s\n ConstraintTemplate = %s\n Property = %s\n Value = %s\n' % (solution['M'], solution['C'], solution['T'], solution['P'], solution['O'])
    query.finish()
