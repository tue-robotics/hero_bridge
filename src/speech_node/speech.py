#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This node listens to a service call and a topic for text to speech
requests. These will be processed by the festival or the philips tts module.
"""

import rospy
import actionlib
import collections

from std_srvs.srv import Empty, EmptyResponse
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
    """
    Bridge from TTS service calls and topic messages to Toyota TTS, introducing a buffer to handle all requests in
    receiving order
    """
    def __init__(self, rate):
        """
        Constructor

        :param rate: ROS parameter for spin-rate
        """

        self.samples_path = rospy.get_param("~samples_path", "~/MEGA/media/audio/soundboard")
        self._rate = rate

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
        """
        Handle requests: put them in buffer and wait if there is a blocking TTS call in the buffer
        :param req: server TTS request
        :type req: SpeakRequest
        """

        if req.blocking_call:
            self.block_queue = True

        # Extend the buffer queue on the right
        self.buffer.append(req)
        rospy.logdebug("Buffer size [{}]: Added the TTS request to the queue.".format(len(self.buffer)))

        # Wait with returning function call if there is a blocking element in the queue
        while not rospy.is_shutdown() and self.block_queue:
            rospy.Rate(self.rate).sleep()

    def speak(self, sentence_msg):
        """
        Receiving subscribed messages over the ~input topic
        :param sentence_msg: topic TTS message
        :type sentence_msg: String
        """

        # Change speak topic message to same type as service call
        req = SpeakRequest()
        req.sentence = sentence_msg.data

        # req.blocking_call = False  # False by default

        self.buffer_requests(req)

    def speak_srv(self, req):
        """
        Receiving service calls over the ~speak service
        :param req: server TTS request
        :type req: SpeakRequest
        """

        self.buffer_requests(req)
        return ""

    def clear_buffer_srv(self, empty):
        """
        Clearing the buffer on service call and setting the (now empty) queue to non-blocking to avoid dead-lock
        :param empty: empty service call
        """

        self.buffer.clear()
        self.block_queue = False
        return EmptyResponse

    def spin(self):
        """
        ROS spin-like function, should be ran after initializing the node. Sends new TTS goals to the Toyota TTS if
        the buffer queue is non-empty and the robot isn't already talking and adjusts the buffer as necessary
        """

        while not rospy.is_shutdown():

            # Print the talking state of the robot (PENDING = 0, TALKING = 1, DONE = 2) to the ROS debugging level
            rospy.logdebug(self.speech_client.simple_state)

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

                # Send the left-most queue entry to Toyota TTS over the simple_action_client
                self.speech_client.send_goal(goal)

                # Pop left-most queue entry
                if self.buffer:
                    self.buffer.popleft()
                rospy.logdebug("Buffer size [{}]: Send TTS request and removed from queue.".format(len(self.buffer)))

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
    tts.spin()
