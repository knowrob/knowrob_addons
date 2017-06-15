#!/usr/bin/env python
import rospy
import tf
from collections import defaultdict
from knowrob_beliefstate.srv._DirtyObject import DirtyObject, DirtyObjectResponse
from std_msgs.msg._ColorRGBA import ColorRGBA
from visualization_msgs.msg._Marker import Marker
from json_prolog import json_prolog


class ThorinObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''
        self.color = ColorRGBA()

    def update_color(self, r, g, b, a):
        self.color = ColorRGBA()
        self.color.r = float(r)
        self.color.g = float(g)
        self.color.b = float(b)
        self.color.a = float(a)

    def update_transform(self, ref_frame, object_name, translation, rotation):
        self.transform = [ref_frame, object_name, translation, rotation]
        self.object_name = str(object_name)

    def get_marker(self):
        marker = Marker()
        marker.header.frame_id = self.object_name
        marker.type = marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.id = 1337
        marker.ns = self.get_short_name()
        marker.color = self.color
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.pose.orientation.w = 1
        marker.mesh_resource = self.mesh_path[:-4] + '.dae'
        return marker

    def get_short_name(self):
        return self.object_name.split('#')[1]


class ObjectTfPublisher(object):
    def __init__(self, tf_frequency):
        self.tf_frequency = tf_frequency
        self.prolog = json_prolog.Prolog()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.dirty_object_srv = rospy.Service('~dirty_object', DirtyObject, self.dirty_cb)
        self.objects = defaultdict(lambda: ThorinObject())

    def dirty_cb(self, srv_msg):
        self.load_object(srv_msg.object_id)
        self.publish_object_frames()
        rospy.loginfo('updated {}'.format(srv_msg.object_id))
        r = DirtyObjectResponse()
        r.success = True
        return r

    def prolog_query(self, q):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if len(solutions) > 1:
            rospy.logwarn('{} returned more than one result'.format(q))
        elif len(solutions) == 0:
            rospy.logwarn('{} returned nothing'.format(q))
        query.finish()
        return solutions

    def load_objects(self):
        self.load_object_ids()
        for object_id in self.objects.keys():
            self.load_object(object_id)

    def load_object(self, object_id):
        self.load_object_transform(object_id)
        self.load_object_mesh(object_id)
        self.load_object_color(object_id)

    def load_object_ids(self):
        q = 'get_known_object_ids(A)'
        solutions = self.prolog_query(q)
        for object_id in solutions[0]['A']:
            self.objects[object_id] = ThorinObject()
        rospy.loginfo('Loaded object ids {}'.format(self.objects.keys()))

    def load_object_color(self, object_id):
        q = "get_object_color('{}', A)".format(object_id)
        solutions = self.prolog_query(q)
        self.objects[object_id].update_color(*solutions[0]['A'])

    def load_object_transform(self, object_id):
        # q = "get_object_transform('{}', A)".format(object_id)
        # solutions = self.prolog_query(q)
        # self.objects[object_id].update_transform('left_gripper_tool_frame', object_id, (0, 0, 0), (0, 0, 0, 1))
        self.objects[object_id].update_transform('map', object_id, (0, 0, 0), (0, 0, 0, 1))

    def load_object_mesh(self, object_id):
        q = "get_object_mesh_path('{}', A)".format(object_id)
        solutions = self.prolog_query(q)
        self.objects[object_id].mesh_path = str(solutions[0]['A'])

    def publish_object_markers(self):
        for object_id, v in self.objects.items():
            self.marker_publisher.publish(v.get_marker())

    def publish_object_frames(self):
        for object_id, thorin_object in self.objects.items():
            ref_frame, object_frame, translation, rotation = thorin_object.transform
            self.tf_broadcaster.sendTransform(translation,
                                              rotation,
                                              rospy.Time.now(),
                                              thorin_object.object_name,
                                              ref_frame)

    def loop(self):
        self.load_objects()
        rate = rospy.Rate(self.tf_frequency)
        while not rospy.is_shutdown():
            self.publish_object_frames()
            self.publish_object_markers()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_tf_publisher')
    asdf = ObjectTfPublisher(1)
    asdf.loop()
