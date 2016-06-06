#!/usr/bin/env python
#
# STT to OpenCog bridge.  Subscribe to ROS chat messages coming
# from the Speech-to-Text (STT) subsystem, and forward them to
# the OpenCog chatbot.
#
import rospy
from chat_api.msg import ChatMessage
from netcat import netcat

def stt_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.utterance)

    # port 17020 should be the same as that in ../scripts/opencog.conf
    netcat("localhost", 17020, "(chat \"" + data.utterance + "\")")

if __name__ == '__main__':
    rospy.init_node('stt_to_oc', anonymous=True)

    rospy.Subscriber("/robot/speech", ChatMessage, stt_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    listener()

