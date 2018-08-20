#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hmi import AbstractHMIServer, HMIResult

from grammar_parser.cfgparser import CFGParser

from tmc_rosjulius_msgs.msg import RecognitionResult

from std_srvs.srv import Empty

class HMIServerJuliusClient(AbstractHMIServer):

    def __init__(self):
        """
        JuliusHMIServer that exposes the HMI ROS Server and holds a socket client that talks to the julius
        speech recognition server.
        """
        super(HMIServerJuliusClient, self).__init__(rospy.get_name())

        self.start_listen_client = rospy.ServiceProxy('/hsrb/voice/start_recognition', Empty)
        self.stop_listen_client = rospy.ServiceProxy('/hsrb/voice/stop_recognition', Empty)

        self.restart_server = rospy.Service('~restart_node', Empty, self.restart_node)


    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        grammar_parser = CFGParser.fromstring(grammar)
        grammar_parser.verify(target)

        self.start_listen_client.wait_for_service()
        self.start_listen_client.call()

        timeout = 10
        score=0.0
        while score < 0.8:       
            # Now wait for result
            try:
                result = rospy.wait_for_message("/hsrb/voice/text", RecognitionResult, timeout)
            except rospy.ROSException:
                self.stop_listen_client.call()
                return None
            score = result.scores[0]

        self.stop_listen_client.call()

        sentence = result.sentences[0]

        rospy.loginfo("Julius Client received sentence %s", sentence)

        if result is None:
            return None

        semantics = grammar_parser.parse(target, sentence)

        return HMIResult(sentence, semantics)


    def restart_node(self, data):
        rospy.loginfo('sending the restart command to windows')
        return {}

if __name__ == "__main__":
    rospy.init_node('hmi_server_julius_client')
    hmi_server_julius_client = HMIServerJuliusClient()

    rospy.spin()

