#!/usr/bin/env python
import rospy
import tf
from collections import defaultdict
from knowrob_beliefstate.srv._DirtyObject import DirtyObject, DirtyObjectResponse
from std_msgs.msg._ColorRGBA import ColorRGBA
from std_srvs.srv._Trigger import Trigger, TriggerResponse
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
        ref_frame = str(ref_frame)
        object_name = str(object_name)
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
        # return self.object_name.split('#')[1]
        return self.object_name


class ObjectTfPublisher(object):
    def __init__(self, tf_frequency):
        self.tf_frequency = tf_frequency
        self.prolog = json_prolog.Prolog()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.dirty_object_srv = rospy.Service('~mark_dirty_object', DirtyObject, self.dirty_cb)
        self.test_srv = rospy.Service('~asdf', Trigger, self.test_srv_cb)
        self.objects = defaultdict(lambda: ThorinObject())

    def test_srv_cb(self, srv_msg):
        objectid = 'http://www.knowrob.org/kb/knowrob_beliefstate#TestWheel'
        object_type = 'http://knowrob.org/kb/knowrob_assembly.owl#BasicMechanicalPart'
        new_transform = ['map', objectid, [0.5, 0, 0.8], [0, 0, 0, 1]]
        # sol = self.prolog_query("owl_individual_of('{}', B)".format(objectid, object_type))
        # sol = self.prolog_query("owl_individual_of('{}', '{}')".format(objectid, object_type))
        q = "replace_object_transforms('{}', {})".format(objectid, new_transform)
        # sol = self.prolog_query("owl_individual_of('{}', '{}'), nth0(1, '{}', '{}')".format(objectid, object_type, new_transform,objectid))
        # sol = self.prolog_query("nth0(1, {}, '{}')".format(new_transform,objectid))
        # sol = self.prolog_query("owl_individual_of('{}', '{}'), "
        #                         "nth0(1, {}, '{}'), "
        #                         "replace_object_transforms('{}', {})".format(objectid, object_type, new_transform, objectid, objectid, new_transform))
        # sol = self.prolog_query("assert_object_at_location('{}', '{}', {})".format(object_type, objectid, new_transform))
        print('queue::: {}'.format(q))
        sol = self.prolog_query(q)
        for s in sol:
            print(s)
        print('---------------------------------------------------')
        # query = self.prolog.query(q)
        # solutions = [x for x in query.solutions()]
        # if len(solutions) > 1:
        #     rospy.logwarn('{} returned more than one result'.format(q))
        # elif len(solutions) == 0:
        #     rospy.logwarn('{} returned nothing'.format(q))
        # query.finish()
        return TriggerResponse()

    def dirty_cb(self, srv_msg):
        print(srv_msg)
        r = DirtyObjectResponse()
        r.error_code = r.SUCCESS
        for object_id in srv_msg.object_ids:
            if not self.load_object(object_id):
                r.error_code = r.UNKNOWN_OBJECT
            else:
                rospy.loginfo("object '{}' updated".format(object_id))
            self.publish_object_frames()
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
        if object_id not in self.objects.keys():
            self.load_objects()
        if object_id in self.objects.keys():
            self.load_object_transform(object_id)
            self.load_object_mesh(object_id)
            self.load_object_color(object_id)
            return True
        return False

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
        q = "get_object_transform('{}', A)".format(object_id)
        # q = "temporal_extent_active(A)"
        # q = "rdf_has(ObjectId, paramserver:'hasTransform', TempRei), temporal_extent_active(TempRei),rdf_has(TempRei, assembly:'hasReferencePart', Ref),atom_string(Ref, RefStr),'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol' == 'http://knowrob.org/kb/knowrob_paramserver.owl#MapFrameSymbol'"
        #q = "rdf_has(ObjectId, paramserver:'hasTransform', TempRei), temporal_extent_active(TempRei),rdf_has(TempRei, assembly:'hasReferencePart', Ref)"
        solutions = self.prolog_query(q)
        print(solutions[0]['A'])
        # self.objects[object_id].update_transform('left_gripper_tool_frame', object_id, (0, 0, 0), (0, 0, 0, 1))
        # self.objects[object_id].update_transform('map', object_id, (0, 0, 0), (0, 0, 0, 1))
        self.objects[object_id].update_transform(*solutions[0]['A'])

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
