#! /usr/bin/env python2.7
"""
To initialize the ROS nodes and subscribers/publishers.
All the subscribers/publishers settings should be placed in the OCSubscribers/Publishers python files.
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
import OCSubscribers
import OCPublishers

class ROSPlugin:


    def __init__(self,ploader,settings):
        rospy.init_node("mcproxy")
        for subscriber in OCSubscribers.subscriberlist:
            subscriber(ploader)
        for publisher in OCPublishers.publisherlist:
            publisher(ploader)
