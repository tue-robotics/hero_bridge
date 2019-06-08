#!/usr/bin/env python
# -*- coding: utf-8 -*-

#'''
#This node listens to a service call and a topic for text to speech
#requests. These will be processed by the festival or the philips tts module.
#'''

import rospy
import actionlib
import collections

from std_srvs.srv import Empty
from std_msgs.msg import String
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from text_to_speech.srv import Speak, SpeakRequest


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class TTS(object):
    def __init__(self, rate):
        self.samples_path = rospy.get_param("~samples_path", "~/MEGA/media/audio/soundboard")
        self.rate = rate

        # buffer and goal state
        self.buffer = collections.deque()
        self.block_queue = False

        # topics
        self.sub_speak = rospy.Subscriber("~input", String, self.speak)
        # self.pub_speak = rospy.Publisher("/talk_request", Voice, queue_size=10)

        # services
        self.srv_speak = rospy.Service('~speak', Speak, self.speak_srv)
        self.srv_clear_buffer = rospy.Service('~clear_buffer', Empty, self.clear_buffer_srv)

        # clients
        self.speech_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        self.speech_client.wait_for_server()


    def buffer_requests(self, req):
        # rospy.loginfo('TTS: Toyota TTS, through bridge node. "' + bcolors.OKBLUE + req.sentence + bcolors.ENDC + '"')

        if req.blocking_call:
            self.block_queue = True

        # Extend the buffer queue on the right
        self.buffer.append(req)
        rospy.loginfo("Buffer size [{}]: Added the TTS request to the queue.".format(len(self.buffer)))

        # Blocking if there the queue is blocking
        while not rospy.is_shutdown() and self.block_queue:
            rospy.loginfo("Speech is currently blocking")
            rospy.Rate(self.rate).sleep()

        # if req.blocking_call:
        #     self.speech_client.wait_for_result()

    def speak(self, sentence_msg):
        req = SpeakRequest()
        req.sentence = sentence_msg.data
        # req.character = self.character
        # req.language = self.language
        # req.voice = self.voice
        # req.emotion = self.emotion
        # req.blocking_call = False  #False by default

        self.buffer_requests(req)

    def speak_srv(self, req):
        self.buffer_requests(req)
        return ""

    def clear_buffer_srv(self):
        self.buffer.clear()
        return []

    def spin(self):
        while not rospy.is_shutdown():

            # rospy.loginfo(self.speech_client.simple_state)

            # If the buffer is non-empty and the robot is not currently talking, send a new TTS request
            if self.buffer and not self.speech_client.simple_state == actionlib.SimpleGoalState.ACTIVE:
                send_req = self.buffer[0]

                # Bridge between the req object and simple_action_client goal object
                goal = TalkRequestGoal()
                out = Voice()
                out.interrupting = False
                out.queueing = True
                out.language = 1
                out.sentence = send_req.sentence
                goal.data = out

                # Send the left-most queue entry
                self.speech_client.send_goal(goal)

                # Pop left-most queue entry
                if self.buffer:
                    self.buffer.popleft()
                rospy.loginfo("Buffer size [{}]: Send TTS request and removed from queue.".format(len(self.buffer)))

                # If the last call is blocking then wait before setting block_queue to false, i.e. blocking until done.
                # Might need to check code below for better alternatives to while-loop.
                if send_req.blocking_call:
                    while not rospy.is_shutdown() and not self.buffer and \
                            self.speech_client.simple_state == actionlib.SimpleGoalState.ACTIVE:
                        rospy.Rate(self.rate).sleep()

                # Check if there is (still) a blocking call in the queue or if the send goal is blocking, if so the
                # queue is blocking
                self.block_queue = any(requests.blocking_call for requests in self.buffer)

            rospy.Rate(self.rate).sleep()


if __name__ == "__main__":
    rospy.init_node('text_to_speech')

    tts = TTS(rospy.get_param('~rate', 10))
    # rospy.loginfo("kom ik hier uberhaupt?")
    tts.spin()

    # rospy.spin()
