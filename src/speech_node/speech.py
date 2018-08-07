#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
#This node listens to a service call and a topic for text to speech
#requests. These will be processed by the festival or the philips tts module.
#'''

import rospy
import actionlib

from std_msgs.msg import String
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from text_to_speech.srv import Speak, SpeakRequest  # , Play, PlayRequest
import sys

reload(sys)
sys.setdefaultencoding('utf8')


class bcolors:
    OKBLUE = '\033[94m'
    ENDC = '\033[0m'


class TTS(object):
    def __init__(self):
        self.samples_path = rospy.get_param("~samples_path", "~/MEGA/media/audio/soundboard")

        # topics
        self.sub_speak = rospy.Subscriber("~input", String, self.speak)
        # self.pub_speak = rospy.Publisher("/talk_request", Voice, queue_size=10)

        # services
        self.srv_speak = rospy.Service('~speak', Speak, self.speak_srv)

        # clients
        self.speech_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        self.speech_client.wait_for_server()

    def do_tts(self, req):
        rospy.loginfo('TTS: Toyota TTS, through bridge node. "' + bcolors.OKBLUE + req.sentence + bcolors.ENDC + '"')

        goal = TalkRequestGoal()
        out = Voice()
        out.interrupting = False
        out.queueing = True
        out.language = 1
        out.sentence = req.sentence
        goal.data = out

        self.speech_client.send_goal(goal)

        if req.blocking_call:
            self.speech_client.wait_for_result()

    def speak(self, sentence_msg):
        req = SpeakRequest()
        req.sentence = sentence_msg.data
        # req.character = self.character
        # req.language = self.language
        # req.voice = self.voice
        # req.emotion = self.emotion
        req.blocking_call = True

        self.do_tts(req)

    def speak_srv(self, req):
        self.do_tts(req)
        return ""


if __name__ == "__main__":
    rospy.init_node('text_to_speech')
    tts = TTS()

    rospy.spin()
