#!/usr/bin/env python
#-*- encoding: utf8 -*-

import time
import sys
import math
import argparse
from optparse import OptionParser

import rospy
from naoqi import ALProxy, ALBroker, ALModule
from geometry_msgs.msg import PointStamped

memory = None

class LoudSoundLocalization(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)

        self.pub_soundlocalization_cmd = rospy.Publisher('loud_sound_detect', PointStamped, queue_size=1)

        sensitivity = rospy.get_param('~sensitivity', 0.02)

        self.soundLoc = ALProxy("ALSoundLocalization")
        self.soundLoc.setParameter("Sensitivity", sensitivity)  # Set a sound sensitivity
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("ALSoundLocalization/SoundLocated", "loudSoundDetector", "onLoudSoundDetected")

    def onLoudSoundDetected(self, name, value):
        azimuth = 0.0
        elevation = 0.0
        confidence = 0.0
        evergy = 0.0

        if(value and isinstance(value, list)):
            soundInfo = value[1]

            azimuth = soundInfo[0]  # 방위각
            elevation = soundInfo[1]  # 고도
            confidence = soundInfo[2]
            energy = soundInfo[3]

        target_point = PointStamped()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = 'Head'
        target_point.point.x = 2.0 * math.cos(azimuth)
        target_point.point.y = 2.0 * math.sin(azimuth)
        target_point.point.z = 0.0

        self.pub_soundlocalization_cmd.publish(target_point)

    def shutdown(self):
        memory.unsubscribeToEvent("ALSoundLocalization/SoundLocated", "loudSoundDetector")

def main(ip, port):
    myBroker = ALBroker("myBroker",
                        "0.0.0.0",   # listen to anyone
                        0,           # find a free port and use it
                        ip,         # parent broker IP
                        port)       # parent broker port

    global loudSoundDetector
    loudSoundDetector = LoudSoundLocalization("loudSoundDetector")
    rospy.spin()

    loudSoundDetector.shutdown()
    myBroker.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('nao_sound_localization', anonymous=False)

    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default="",
                      help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
    parser.add_option("--pport", dest="pport", default=0,
                      help="IP/hostname of broker. Default is automatic port.", metavar="PORT")

    (options, args) = parser.parse_args()
    main(options.pip, int(options.pport))