#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import threading
import yaml
import json
import random
import re

import rospy
import actionlib
import rospkg
# import tf
import tf2_ros
from tf2_geometry_msgs import PointStamped

from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)

from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mind_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse
# from geometry_msgs.msg import PointStamped

class NaoBehaviors(NaoqiNode):
    NODE_NAME = "nao_render_gesture"

    def __init__(self):
        NaoqiNode.__init__(self, self.NODE_NAME)

        self.behavior = None
        self.gesture = ''
        self.lock = threading.RLock()

        self.behaviorProxy = self.get_proxy("ALBehaviorManager")
        self.motionProxy = self.get_proxy("ALTracker")

        self.tf_buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buf)

        if self.behaviorProxy == None:
            quit(1)

        try:
            motion_file = rospy.get_param('~motion_file')
        except KeyError:
            rospy.logerr('Set parammeter ~motion_file')
            quit(1)

        # motion_file = os.path.join(rospkg.RosPack().get_path('nao_v5'), 'config', 'motions.yaml')
        stream = file(motion_file, 'r')
        self.motion_list = yaml.load(stream)

        self.getInstalledGesturesService = rospy.Service(
            "get_installed_gestures",
            GetInstalledGestures,
            self.getInstalledGestures
        )

        # Prepare and start actionlib server
        self.actionlibServer = actionlib.SimpleActionServer(
            "render_gesture",
            RenderItemAction,
            self.render_gesture,
            False
        )

        self.actionlibServer.register_preempt_callback(self.stop_gesture)
        self.actionlibServer.start()

        rospy.loginfo(self.NODE_NAME + " intialized...")
        rospy.spin()

    def getInstalledGestures(self, request):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse(result)

    def render_gesture(self, goal):
        rospy.logdebug("Execution of behavior: '{}' requested".format(goal.data))

        result = RenderItemResult()
        feedback = RenderItemFeedback()
        succeed = True

        gesture_type, gesture_data = goal.data.split('=')
        if gesture_type == 'gesture':
            (cmd, item_name) = gesture_data.split(':')
            if cmd == 'tag':
                match = re.search(r'\[(.+?)\]', item_name)
                rendering_gesture = ''
                if match:
                    item_name = item_name.replace(match.group(0), '')
                    emotion = match.group(1)

                    try:
                        rendering_gesture = self.motion_list[item_name][emotion][random.randrange(0, len(self.motion_list[item_name]) - 1)]
                    except (KeyError, TypeError):
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]

                else:
                    try:
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                    except KeyError:
                        rendering_gesture = self.motion_list['neutral'][random.randint(0, len(self.motion_list[item_name]) - 1)]

                with self.lock:
                    if self.actionlibServer.is_preempt_requested():
                        self.actionlibServer.set_preempted()
                        rospy.logdebug("Gesture execution preempted before it started")
                        return

                    self.gesture = rendering_gesture
                    taskID = self.behaviorProxy.post.runBehavior(self.gesture)

                rospy.logdebug("Waiting for behavior execution to complete")
                while self.behaviorProxy.isRunning(taskID) and not rospy.is_shutdown():
                    rospy.sleep(0.2)

                with self.lock:
                    self.gesture = None
                    succeed = True

            elif cmd == 'play':
                found_motion = False
                for k, v in self.motion_list.items():
                    if item_name in v:
                        found_motion = True

                if not found_motion:
                    error_msg = "Gesture '{}' not installed".format(item_name)
                    self.actionlibServer.set_aborted(text = error_msg)
                    rospy.logerr(error_msg)
                    return
                else:
                    self.gesture = item_name
                    taskID = self.behaviorProxy.post.runBehavior(self.gesture)

                    rospy.logdebug("Waiting for behavior execution to complete")
                    while self.behaviorProxy.isRunning(taskID) and not rospy.is_shutdown():
                        rospy.sleep(0.2)

                    with self.lock:
                        self.gesture = None
                        succeed = True

        elif gesture_type == 'pointing':
            parse_data = json.loads(gesture_data)
            rospy.loginfo('\033[94m[%s]\033[0m rendering pointing to xyz:%s, frame_id [%s]...'%(rospy.get_name(),
                parse_data['xyz'], parse_data['frame_id']))

            target = PointStamped()
            target.header.frame_id = parse_data['frame_id']
            target.point.x = parse_data['xyz'][0]
            target.point.y = parse_data['xyz'][1]
            target.point.z = parse_data['xyz'][2]

            try:
                point_transformed = self.tf_buf.transform(target, 'torso')
                self.motionProxy.pointAt("RArm", [point_transformed.point.x, point_transformed.point.y, point_transformed.point.z], 0, 0.3)
                rospy.sleep(1)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn('The transform information can not find with /RShoulder and target point...')
                succeed = False

        else:
            rospy.sleep(1.0)
            succeed = False

        if succeed:
            result.result = True
            self.actionlibServer.set_succeeded(result)
            rospy.loginfo('\033[95m%s\033[0m rendering completed...' % rospy.get_name())

    def stop_gesture(self):
        with self.lock:
            if self.gesture and self.actionlibServer.is_active():
                self.behaviorProxy.stopBehavior(self.gesture)


if __name__ == '__main__':
    node = NaoBehaviors()