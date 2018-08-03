#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import time

import naoqi
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALModule, ALBroker, ALProxy)
from optparse import OptionParser

from std_msgs.msg import Bool

class IdleMotion:
    def __init__(self):
        rospy.init_node('nao_behavior_idle_motion', anonymous=False)

        parser = OptionParser()
        parser.add_option("--pip", dest="pip", default="",
                          help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
        parser.add_option("--pport", dest="pport", default=0,
                          help="IP/hostname of broker. Default is automatic port.", metavar="PORT")

        (options, args) = parser.parse_args()
        self.ip = options.pip
        self.port = int(options.pport)

        self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
        self.postureProxy = ALProxy("ALRobotPosture", self.ip, self.port)

        self.motionProxy.wakeUp()
        self.postureProxy.goToPosture("Stand", 1.0)

        rospy.Subscriber('idle_motion/set_enabled', Bool, self.handle_idle_status)
        rospy.loginfo('[%s] is ready.' % rospy.get_name())

        rospy.loginfo('[%s] is starting the idle motion.' % rospy.get_name())
        self.motionProxy.setBreathEnabled('Body', True)
        self.motionProxy.setBreathEnabled('Head', False)
        self.motionProxy.setBreathConfig([['Bpm', 12], ['Amplitude', 0.8]])
        rospy.spin()


    def shutdown(self):
        self.motionProxy.setBreathEnabled('Body', False)
        self.motionProxy.setBreathEnabled('Head', False)
        time.sleep(1)
        self.motionProxy.rest()
        print self.motionProxy.getSummary()


    def handle_idle_status(self, msg):
        if msg.data:
            self.motionProxy.setBreathEnabled('Body', True)
            self.motionProxy.setBreathEnabled('Head', False)
            self.motionProxy.setBreathConfig([['Bpm', 12], ['Amplitude', 0.8]])
        else:
            self.motionProxy.setBreathEnabled('Body', False)
            self.motionProxy.setBreathEnabled('Head', False)


if __name__ == '__main__':
    m = IdleMotion()
    m.shutdown()