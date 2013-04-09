#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    
    print("Read constraint templates:")
    query = prolog.query("plan_constraint_templates(pancake_constr:'FlippingAPancake', _Cs), member(C, _Cs), constraint_properties(C, P, O)")
    for solution in query.solutions():
        print 'Found solution:\n C = %s\n P = %s\n O = %s\n' % (solution['C'], solution['P'], solution['O'])
    query.finish()

    print "\n\n\n\n\n"
    
    print("Read the actual sets of constraints and their properties:")
    query = prolog.query("plan_constraints(pancake_constr:'FlippingAPancake', C), owl_direct_subclass_of(C, T), rdf_has(T, rdf:type, owl:'Class'),  constraint_properties(C, P, O).")
    for solution in query.solutions():
        print 'Found solution:\n C = %s\n P = %s\n O = %s\n' % (solution['C'], solution['P'], solution['O'])
    query.finish()
    