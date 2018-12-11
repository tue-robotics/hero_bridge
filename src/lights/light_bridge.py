#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys

from amigo_msgs.msg import RGBLightCommand
from std_msgs.msg import ColorRGBA

reload(sys)
sys.setdefaultencoding('utf8')


class Shine(object):
    def __init__(self):

        # topics
        self.sub_lights = rospy.Subscriber('/hero/rgb_lights_manager/user_set_rgb_lights', RGBLightCommand, self.shine)
        self.pub_lights = rospy.Publisher('/hsrb/command_status_led_rgb', ColorRGBA, queue_size=2)

    def shine(self, command):
        color = command.color
        self.pub_lights.publish(color)


if __name__ == "__main__":
    rospy.init_node('lights_bridge')
    shine = Shine()

    rospy.spin()
