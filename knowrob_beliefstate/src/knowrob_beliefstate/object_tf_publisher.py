#!/usr/bin/env python
import rospy
import tf
from collections import defaultdict
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog

class ThorinObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''

    def add_transform(self, transform):
        self.transform = transform
        self.object_name = str(transform[1]).split('#')[1]

    def get_marker(self):
        marker = Marker()
        marker.header.frame_id = self.object_name
        marker.type = marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.id = 0
        marker.ns = 'muh'
        marker.pose.orientation.w = 1
        marker.mesh_resource = self.mesh_path
        return marker

class ObjectTfPublisher(object):
    def __init__(self):
        self.prolog = json_prolog.Prolog()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.objects = defaultdict(lambda: ThorinObject())

    def load_object_ids(self):
        query = self.prolog.query('get_known_object_ids(A)')
        solutions = [x for x in query.solutions()]
        if len(solutions) > 1:
            rospy.logwarn('get_known_object_ids returned more than one list')
        for object_id in solutions[0]['A']:
            self.objects[object_id] = ThorinObject()

        rospy.loginfo('Loaded object ids {}'.format(self.objects))
        query.finish()

    def get_object_transform(self, object_id):
        # q = "get_object_transform('{}', B)".format(object_id)
        # q = "get_object_mesh_path('{}', B)".format(object_id)
        # q = "get_associated_transform(_, _, _, TempRei, _, 'http://knowrob.org/kb/knowrob_paramserver.owl#hasTransform', Transform)."
        # q = "rdf_has(TempExt, paramserver:'endsAtTime', B)"
        # q = "rdf_has(TempRei, paramserver:'temporalExtent', TempExt)"
        # q = "rdf_has('{}', paramserver:'hasTransform', TempRei)".format(object_id)
        # q = "rdf_has('{}', paramserver:'hasTransform', TempRei)".format(object_id)
        # q = "rdf_has('{}', paramserver:'hasTransform', TempRei)," \
        #     "rdf_has(TempRei, paramserver:'temporalExtent', TempExt)".format(object_id)

        # print(q)
        # query = self.prolog.query(q)
        # solutions = [x for x in query.solutions()]
        # print(solutions)
        # for s in query.solutions():
        #     print(s)
        # query.finish()
        return ['map', object_id, (0, 0, 0), (0, 0, 0, 1)]

    def get_object_mash(self, object_id):
        q = "get_object_mesh_path('{}', B)".format(object_id)
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if len(solutions) > 1:
            rospy.logwarn('get_known_object_ids returned more than one list')
        self.objects[object_id].mesh_path = str(solutions[0]['B'])
        query.finish()
        # marker = Marker()
        # marker.header.frame_id = self.objects[object_id]['transform']
        # marker.type = marker.MESH_RESOURCE

    def load_object_transforms(self):
        for object_id in self.objects.keys():
            self.objects[object_id].add_transform(self.get_object_transform(object_id))

    def publish_object_frames(self):
        for object_id, thorin_object in self.objects.items():
            ref_frame, object_frame, translation, rotation = thorin_object.transform
            self.tf_broadcaster.sendTransform(translation,
                                              rotation,
                                              rospy.Time.now(),
                                              thorin_object.object_name,
                                              ref_frame)

    def load_object_shapes(self):
        for object_id in self.objects.keys():
            self.get_object_mash(object_id)

    def publish_object_markers(self):
        for object_id, v in self.objects.items():
            print('{} {}'.format(object_id, v.mesh_path))
            self.marker_publisher.publish(v.get_marker())

    def loop(self):
        self.load_object_ids()
        self.load_object_transforms()
        self.load_object_shapes()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_object_frames()
            self.publish_object_markers()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_tf_publisher')
    asdf = ObjectTfPublisher()
    asdf.loop()
