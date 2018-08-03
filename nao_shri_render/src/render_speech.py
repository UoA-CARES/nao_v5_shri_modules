#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib

from dynamic_reconfigure.server import Server as ReConfServer
import dynamic_reconfigure.client
from naoqi_driver_py.cfg import NaoqiSpeechConfig as NodeConfig
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import ALBroker, ALProxy, ALModule

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from naoqi_bridge_msgs.msg import SpeechWithFeedbackGoal,SpeechWithFeedbackResult, SpeechWithFeedbackFeedback, SpeechWithFeedbackAction
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback


class Constants:
    NODE_NAME = "nao_render_speech"
    TEXT_STARTED_EVENT = "ALTextToSpeech/TextStarted"
    TEXT_DONE_EVENT = "ALTextToSpeech/TextDone"


class NaoSpeech(ALModule, NaoqiNode):
    def __init__(self, moduleName):
        NaoqiNode.__init__(self, Constants.NODE_NAME)

        self.moduleName = moduleName
        self.ip = ""
        self.port = 0
        self.init_almodule()

        self.speech_with_feedback_flag = False # Used for speech with feedback mode only
        self.conf = None
        self.srw = None

        self.subscribe()

        self.reconf_server = ReConfServer(NodeConfig, self.reconfigure)
        self.reconf_client = dynamic_reconfigure.client.Client(rospy.get_name())

        self.sub = rospy.Subscriber("speech", String, self.say)

        self.speechWithFeedbackServer = actionlib.SimpleActionServer("render_speech", RenderItemAction, self.executeSpeechWithFeedbackAction, auto_start=False)
        self.speechWithFeedbackServer.start()

    def init_almodule(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker(
                "%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError, e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)

        self.memProxy = ALProxy("ALMemory", self.pip, self.pport)
        if self.memProxy is None:
            rospy.logerr(
                "Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)

        self.tts = self.get_proxy("ALTextToSpeech")
        if self.tts is None:
            rospy.logerr(
                "Could not get a proxy to ALTextToSpeech on %s:%d", self.pip, self.pport)
            exit(1)

        self.audio = self.get_proxy("ALAudioDevice")
        if self.audio is None:
            rospy.logwarn(
                "Proxy to ALAudioDevice not available, using dummy device (normal in simulation; volume controls disabled)")
            self.audio = DummyAudioDevice()

    def subscribe(self):
        self.memProxy.subscribeToEvent(Constants.TEXT_DONE_EVENT, self.moduleName, "onTextDone")
        self.memProxy.subscribeToEvent(Constants.TEXT_STARTED_EVENT, self.moduleName, "onTextStarted")

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent(Constants.TEXT_DONE_EVENT, self.moduleName)
        self.memProxy.unsubscribeToEvent(Constants.TEXT_STARTED_EVENT, self.moduleName)

    def onTextStarted(self, strVarName, value, strMessage):
        if value == 0 or self.speech_with_feedback_flag == False:
            return

        # fb = SpeechActionFeedback()
        # self.speechWithFeedbackServer.publish_feedback(fb)

    def onTextDone(self, strVarName, value, strMessage):
        if value == 0 or self.speech_with_feedback_flag == False:
            return

        self.speech_with_feedback_flag = False

    def executeSpeechWithFeedbackAction(self, goal):
        self.speech_with_feedback_flag = True
        success = True
        result = RenderItemResult()

        saystr = goal.data
        if goal.data == '':
            rospy.sleep(0.5)
            result.result = True
            self.speechWithFeedbackServer.set_succeeded(result)
            return
        else:
            self.internalSay(saystr)

        counter = 0
        while self.speech_with_feedback_flag == True and counter < 1200:
            if self.speechWithFeedbackServer.is_preempt_requested():
                self.tts.stopAll()
                self.speechWithFeedbackServer.set_preempted()
                return
            rospy.sleep(0.1)
            counter += 1

        if success:
            rospy.sleep(0.2)
            result.result = True
            self.speechWithFeedbackServer.set_succeeded(result)

    def reconfigure(self, request, level):
        newConf = {}

        newConf["voice"] = request["voice"]
        newConf["language"] = request["language"]
        newConf["volume"] = request["volume"]
        newConf["vocabulary"] = request["vocabulary"]
        newConf["audio_expression"] = request["audio_expression"]
        newConf["visual_expression"] = request["visual_expression"]
        newConf["word_spotting"] = request["word_spotting"]

        if not newConf["voice"]:
            newConf["voice"] = self.tts.getVoice()
        elif newConf["voice"] not in self.tts.getAvailableVoices():
            rospy.logwarn(
                "Unknown voice '{}'. Using current voice instead".format(
                    newConf["voice"]))
            rospy.loginfo("Voices available: {}".format(
                self.tts.getAvailableVoices()))
            newConf["voice"] = self.tts.getVoice()

        if not newConf["language"]:
            newConf["language"] = self.tts.getLanguage()
        elif newConf["language"] not in self.tts.getAvailableLanguages():
            newConf["language"] = self.tts.getLanguage()
            rospy.logwarn(
                "Unknown language '{}'. Using current language instead".format(
                    newConf["language"]))
            rospy.loginfo("Languages available: {}".format(
                self.tts.getAvailableLanguages()))

        if not self.conf and not rospy.has_param("~volume"):
            newConf["volume"] = self.audio.getOutputVolume()

        if self.srw and self.conf and (
                newConf["language"] != self.conf["language"] or
                newConf["audio_expression"] != self.conf["audio_expression"] or
                newConf["visual_expression"] != self.conf["visual_expression"]):
            need_to_restart_speech = True
        else:
            need_to_restart_speech = False

        self.conf = newConf
        if need_to_restart_speech:
            self.stop()
            self.start()

        return self.conf

    def say(self, request):
        self.internalSay(request.data)

    def internalSay(self, sentence):
        current_voice = self.tts.getVoice()
        current_language = self.tts.getLanguage()
        current_volume = self.audio.getOutputVolume()
        current_gain = self.tts.getVolume()
        target_gain = 1.0


        if self.conf["voice"] != current_voice:
            self.tts.setVoice(self.conf["voice"])

        if self.conf["language"] != current_language:
            self.tts.setLanguage(self.conf["language"])

        if self.conf["volume"] != current_volume:
            self.audio.setOutputVolume(self.conf["volume"])

        if target_gain != current_gain:
            self.tts.setVolume(target_gain)

        self.tts.post.say(sentence)


        if self.conf["voice"] != current_voice:
            self.tts.setVoice(current_voice)

        if self.conf["language"] != current_language:
            self.tts.setLanguage(current_language)

        if self.conf["volume"] != current_volume:
            self.audio.setOutputVolume(current_volume)

        if target_gain != current_gain:
            self.tts.setVolume(current_gain)

    def shutdown(self):
        self.unsubscribe()


if __name__ == '__main__':
    ROSNaoSpeechModule = NaoSpeech("ROSNaoSpeechModule")
    rospy.loginfo("ROSNaoSpeechModule running...")
    rospy.spin()

    rospy.loginfo("Stopping ROSNaoSpeechModule ...")
    if ROSNaoSpeechModule.srw:
        ROSNaoSpeechModule.srw.stop()

    ROSNaoSpeechModule.shutdown()
    rospy.loginfo("ROSNaoSpeechModule stopped.")
    exit(0)