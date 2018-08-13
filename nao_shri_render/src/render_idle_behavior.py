#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import time
import math

import naoqi
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALModule, ALBroker, ALProxy)
from optparse import OptionParser
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

from std_msgs.msg import Bool
from mind_msgs.msg import SetIdleMotion

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
        self.postureProxy.goToPosture("Stand", 0.5)

        # TBD
        self.enable_leaning_forward = False
        rospy.Subscriber('set_enable_leaning_forward', Bool, self.handle_enable_leaning_forward)

        rospy.Subscriber('set_enable_idle_motion', SetIdleMotion, self.handle_idle_status)
        self.pub_joint_cmd = rospy.Publisher(
            'joint_angles', JointAnglesWithSpeed, queue_size=100)

        rospy.loginfo('[%s] is ready.' % rospy.get_name())
        rospy.loginfo('[%s] is starting the idle motion.' % rospy.get_name())

        self.motionProxy.setBreathEnabled('Body', True)
        self.motionProxy.setBreathEnabled('Head', False)        
        self.motionProxy.setBreathConfig([['Bpm', 12], ['Amplitude', 0.8]])

        rospy.spin()

    def handle_enable_leaning_forward(self, msg):
        self.enable_leaning_forward = msg.data
        if msg.data:
            self.motionProxy.setBreathEnabled('Legs', False)

            cmd_msg = JointAnglesWithSpeed()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.joint_names.append('LHipPitch')
            cmd_msg.joint_angles.append(-9.0 * math.pi / 180.0)
            cmd_msg.joint_names.append('RHipPitch')
            cmd_msg.joint_angles.append(-9.0 * math.pi / 180.0)
            cmd_msg.speed = 0.06
            cmd_msg.relative = 0

            self.pub_joint_cmd.publish(cmd_msg)
        else:
            cmd_msg = JointAnglesWithSpeed()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.joint_names.append('LHipPitch')
            cmd_msg.joint_angles.append(0.0 * math.pi / 180.0)
            cmd_msg.joint_names.append('RHipPitch')
            cmd_msg.joint_angles.append(0.0 * math.pi / 180.0)
            cmd_msg.speed = 0.06
            cmd_msg.relative = 0

            self.motionProxy.setBreathEnabled('Legs', True)

    def shutdown(self):
        self.motionProxy.setBreathEnabled('Body', False)
        self.motionProxy.setBreathEnabled('Head', False)
        self.motionProxy.setBreathEnabled('Legs', False)

        self.postureProxy.goToPosture("Stand", 0.2)
    
        self.motionProxy.rest()
        print self.motionProxy.getSummary()


    def handle_idle_status(self, msg):
        if msg.enabled:
            self.motionProxy.setBreathEnabled('Body', True)
            self.motionProxy.setBreathEnabled('Head', False)

            if self.enable_leaning_forward:
                self.motionProxy.setBreathEnabled('Legs', False)
            self.motionProxy.setBreathConfig([['Bpm', 12], ['Amplitude', 0.8]])

            if self.enable_leaning_forward:
                rospy.sleep(0.6)

                cmd_msg = JointAnglesWithSpeed()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.joint_names.append('LHipPitch')
                cmd_msg.joint_angles.append(-9.0 * math.pi / 180.0)
                cmd_msg.joint_names.append('RHipPitch')
                cmd_msg.joint_angles.append(-9.0 * math.pi / 180.0)
                cmd_msg.speed = 0.1
                cmd_msg.relative = 0

                self.pub_joint_cmd.publish(cmd_msg)
                rospy.sleep(0.2)

        else:
            self.postureProxy.goToPosture("Stand", 0.2)

            self.motionProxy.setBreathEnabled('Body', False)
            self.motionProxy.setBreathEnabled('Head', False)

            if self.enable_leaning_forward:
                self.motionProxy.setBreathEnabled('Legs', False)            


if __name__ == '__main__':
    m = IdleMotion()
    m.shutdown()