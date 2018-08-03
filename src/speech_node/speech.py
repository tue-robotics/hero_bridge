#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
#This node listens to a service call and a topic for text to speech
#requests. These will be processed by the festival or the philips tts module.
#'''

import rospy

from std_msgs.msg import String
from tmc_msgs.msg import Voice
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
        self.pub_speak = rospy.Publisher("/talk_request", Voice, queue_size=10)

        # services
        self.srv_speak = rospy.Service('~speak', Speak, self.speak_srv)

        # clients
        # self.client_play = rospy.ServiceProxy('play', Play)

    def do_tts(self, req):
        rospy.loginfo('TTS: Toyota TTS, through bridge node. "' + bcolors.OKBLUE + req.sentence + bcolors.ENDC + '"')

        # Check if an audio file for this sentence already exists
        # for extension in ["wav", "mp3", "oga"]:
        #     potential_filename = os.path.join(os.path.expanduser(self.samples_path),
        #                                       req.sentence.lower() + "." + extension)
        #     rospy.logdebug("Checking for file on path: " + potential_filename)
        #     if os.path.isfile(potential_filename):
        #         rospy.logdebug("Found file!")
        #         play_req = PlayRequest()
        #         play_req.audio_data = open(potential_filename, "rb").read()
        #         play_req.audio_type = extension
        #         play_req.blocking_call = req.blocking_call
        #         play_req.pitch = 0
        #         resp = self.client_play(play_req)
        #         rospy.logdebug("Response: " + resp.error_msg)
        #         return ""

        # No audio sample existed, continuing with TTS
        out = Voice()
        out.interrupting = False
        out.queueing = True
        out.language = 1
        out.sentence = req.sentence

        self.pub_speak.publish(out)

    def speak(self, sentence_msg):
        req = SpeakRequest()
        req.sentence = sentence_msg.data
        # req.character = self.character
        # req.language = self.language
        # req.voice = self.voice
        # req.emotion = self.emotion
        # req.blocking_call = False

        self.do_tts(req)

    def speak_srv(self, req):
        self.do_tts(req)
        return ""


if __name__ == "__main__":
    rospy.init_node('text_to_speech')
    tts = TTS()

    rospy.spin()
