#!/usr/bin/env python
import yaml
import rospy
from json_prolog import json_prolog

class OwlToParamserver(object):
    def __init__(self):
        rospy.wait_for_service('/json_prolog/query')
        self.prolog = json_prolog.Prolog()

    def add_to_server(self, prolog_query, namespace):
        query = self.prolog.query(prolog_query)
        solution = query.solutions().next()
        for k, v in solution.items():
            rospy.set_param('{}/{}'.format(namespace, k), str(v))
            rospy.loginfo('added {}={} to paramserver'.format(k,v))
        query.finish()

    def load_yaml(self, path):
        with open(path, 'r') as stream:
            rospy.loginfo('Parameters loaded from {}'.format(path))
            yaml_dict = yaml.load(stream)
            for param in yaml_dict:
                param = param['param']
                query = param['query']
                ns = param['ns']
                self.add_to_server(query, ns)

if __name__ == '__main__':
    rospy.init_node('owl_to_paramserver')
    queries = rospy.get_param('~yaml_path', default='../../launch/prolog_queries.yaml')
    asdf = OwlToParamserver()
    asdf.load_yaml(queries)
