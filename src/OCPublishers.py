#! /usr/bin/env python2.7
"""
All the Publisher used for publishing messages to opencog.
Wrapping the settings as a publisher class.
"""


import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import visibility_msg

class VisionPublisher:

    tickrate=0.1

    def __init__(self,ploader):
        
        self.event=ploader.requires('Event')
        self.visibility=ploader.requires('Visibility')
        self.timers=ploader.requires('Timers')
        self.timers.reg_event_timer(self.tickrate,self.sendvisibility)
        self.Publisher=rospy.Publisher('visibility_data', visibility_msg)

    def sendvisibility(self):
        
        message=visibility_msg()
        message.visibleblocks=self.visibility.get()
        self.Publisher.publish(message)

publisherlist=[VisionPublisher]
