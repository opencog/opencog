#! /usr/bin/env python2.7
"""
All the Subscriber used for publishing messages to opencog.
Wrapping the settings as a subscriber class, including the callback function.
"""


import roslib; roslib.load_manifest('minecraft_bot')
import rospy

from minecraft_bot.msg import controller_msg


class TestSubscriber:

    def __init__(self,client):
        
        self.msg_name="controller_data"
        self.msg_class=controller_msg
        self.queue_size=1

        self.event=client.requires('Event')
    
        rospy.Subscriber(self.msg_name,self.msg_class,self.callback,self.queue_size)

    def callback(self, __,data):
        
        command = data
        command_out = ""
        print("received value: " + str(command))
        if command == 1:
            command_out = "cmd_animation"
            print("sent command to spock: " + command_out)
            self.event.emit(command_out)

subscriberlist=[TestSubscriber]
