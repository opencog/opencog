#!/usr/bin/env python
import rospy
from oc_chat_ros.msg import ChatMessage
from netcat import netcat

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.utterance)
    netcat("localhost",17040,"(chat \""+data.utterance+"\")")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('stt_to_oc', anonymous=True)

    rospy.Subscriber("/robot/speech", ChatMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

