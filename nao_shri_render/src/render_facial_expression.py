#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib
import random
import naoqi
import time
import operator
from threading import Thread

from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALModule, ALBroker, ALProxy)
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback


class NaoFace(NaoqiNode):
    NODE_NAME = "nao_render_expression"

    # Eye Color for emotions ->  r  |  g  |  b
    eye_color = {}
    eye_color['neutral'] = int(int(255) << 16 | int(255) << 8 | int(255))
    eye_color['happiness'] = int(int(44) << 16 | int(176) << 8 | int(28))
    eye_color['surprise'] = int(int(254) << 16 | int(233) << 8 | int(150))
    eye_color['anger'] = int(int(249) << 16 | int(0) << 8 | int(7))
    eye_color['sadness'] = int(int(30) << 16 | int(95) << 8 | int(174))
    eye_color['disgust'] = int(int(120) << 16 | int(41) << 8 | int(174))
    eye_color['fear'] = int(int(218) << 16 | int(85) << 8 | int(168))
    eye_color['sleepiness'] = int(int(254) << 16 | int(233) << 8 | int(150))

    def __init__(self):
        NaoqiNode.__init__(self, self.NODE_NAME)

        self.proxy = self.get_proxy("ALLeds")
        random.seed(rospy.Time.now().to_nsec())
        self.current_eye_color = self.eye_color['neutral']
        self.last_eye_color = self.current_eye_color

        self.server = actionlib.SimpleActionServer('render_facial_expression', RenderItemAction, self.execute_callback, False)
        self.server.start()

        self.t1 = Thread(target=self.handle_blink_thread)
        self.t1.start()

        rospy.loginfo("nao_render_expression initialized")
        rospy.spin()

    def handle_blink_thread(self):
        while not rospy.is_shutdown():
            rDuration = 0.02
            self.proxy.post.fadeRGB("FaceLed0", 0x000000, rDuration)
            self.proxy.post.fadeRGB("FaceLed1", 0x000000, rDuration)
            self.proxy.post.fadeRGB("FaceLed2", self.current_eye_color, rDuration)
            self.proxy.post.fadeRGB("FaceLed3", 0x000000, rDuration)
            self.proxy.post.fadeRGB("FaceLed4", 0x000000, rDuration)
            self.proxy.post.fadeRGB("FaceLed5", 0x000000, rDuration)
            self.proxy.post.fadeRGB("FaceLed6", self.current_eye_color, rDuration)
            self.proxy.fadeRGB("FaceLed7", 0x000000, rDuration)
            time.sleep(0.1)
            self.proxy.fadeRGB("FaceLeds", self.current_eye_color, rDuration)

            time.sleep(3)

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()

        success = True
        self.last_eye_color = self.current_eye_color
        try:
            self.current_eye_color = self.eye_color[goal.data]
            self.proxy.fadeRGB("FaceLeds", self.current_eye_color, 0.02)
        except KeyError:
            rospy.logwarn('render %s expression is not defined...'%goal.data)
            pass

        rospy.sleep(0.2)

        if success:
            result.result = True
            self.server.set_succeeded(result)
            rospy.loginfo('\033[95m%s\033[0m rendering completed...' % rospy.get_name())


if __name__ == '__main__':
    ROSNaoFaceModule = NaoFace()