#!/usr/bin/env python

import rospy
import naoqi
import threading
from std_msgs.msg import Bool
from naoqi_bridge_msgs.msg import HandTouch
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import ( ALModule, ALBroker, ALProxy )


class NaoHandTactile(ALModule):
    "Sends callbacks for tactile touch, bumper press and foot contact to ROS"
    def __init__(self, moduleName):
        # get connection from command line:
        from optparse import OptionParser

        parser = OptionParser()
        parser.add_option("--ip", dest="ip", default="",
                          help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
        parser.add_option("--port", dest="port", default=0,
                          help="IP/hostname of broker. Default is automatic port.", metavar="PORT")
        parser.add_option("--pip", dest="pip", default="127.0.0.1",
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_option("--pport", dest="pport", default=9559,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        (options, args) = parser.parse_args()
        self.ip = options.ip
        self.port = int(options.port)
        self.pip = options.pip
        self.pport = int(options.pport)
        self.moduleName = moduleName

        self.lock = threading.RLock()

        self.init_almodule()
        rospy.init_node('hand_touch_detector')

        # init. messages:
        self.handTouchPub = rospy.Publisher("hand_touch", HandTouch, queue_size=10)
        self.subscribe()

        rospy.loginfo("nao_tactile initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)

        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)


    def shutdown(self):
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)


    def subscribe(self):
        self.memProxy.subscribeToEvent("HandLeftBackTouched", self.moduleName, "onLeftTactileChanged")
        self.memProxy.subscribeToEvent("HandLeftLeftTouched", self.moduleName, "onLeftTactileChanged")
        self.memProxy.subscribeToEvent("HandLeftRightTouched", self.moduleName, "onLeftTactileChanged")

        self.memProxy.subscribeToEvent("HandRightBackTouched", self.moduleName, "onRightTactileChanged")
        self.memProxy.subscribeToEvent("HandRightLeftTouched", self.moduleName, "onRightTactileChanged")
        self.memProxy.subscribeToEvent("HandRightRightTouched", self.moduleName, "onRightTactileChanged")


    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("HandLeftBackTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("HandLeftLeftTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("HandLeftRightTouched", self.moduleName)

        self.memProxy.unsubscribeToEvent("HandRightBackTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("HandRightLeftTouched", self.moduleName)
        self.memProxy.unsubscribeToEvent("HandRightRightTouched", self.moduleName)



    def onLeftTactileChanged(self, strVarName, value, strMessage):
        "Called when tactile touch status changes in ALMemory"
        msg = HandTouch()

        with self.lock:
            if strVarName == "HandLeftBackTouched":
                msg.hand = HandTouch.LEFT_BACK
            elif strVarName == "HandLeftLeftTouched":
                msg.hand = HandTouch.LEFT_LEFT
            elif strVarName == "HandLeftRightTouched":
                msg.hand = HandTouch.LEFT_RIGHT

            msg.state = int(value);
            self.handTouchPub.publish(msg)
            rospy.logdebug("Hand touched: name=%s, value=%d, message=%s.", strVarName, value, strMessage);

    def onRightTactileChanged(self, strVarName, value, strMessage):
        "Called when tactile touch status changes in ALMemory"
        msg = HandTouch()

        with self.lock:
            if strVarName == "HandRightBackTouched":
                msg.hand = HandTouch.RIGHT_BACK
            elif strVarName == "HandRightLeftTouched":
                msg.hand = HandTouch.RIGHT_LEFT
            elif strVarName == "HandRightRightTouched":
                msg.hand = HandTouch.RIGHT_RIGHT

            msg.state = int(value);
            rospy.logdebug("Hand touched: name=%s, value=%d, message=%s.", strVarName, value, strMessage);
            self.handTouchPub.publish(msg)

if __name__ == '__main__':
    ROSNaoHandTactileModule = NaoHandTactile("ROSNaoHandTactileModule")
    rospy.spin()

    rospy.loginfo("Stopping hand_touch_detector ...")
    ROSNaoHandTactileModule.shutdown();
    rospy.loginfo("hand_touch_detector stopped.")
    exit(0)