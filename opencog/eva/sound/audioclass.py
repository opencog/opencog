#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float32
from netcat import netcat
from audio_stream.msg import audiodata

CWD = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(CWD, '..', 'face_track'))

'''
    This class subscribes to topic audio_sensors and based on the different
    range of values of the Decibel classifies the input sound into Quiet,
    Normal Conversation, Loud conversation,critical sound and when there is
    a transfer from one range to another based on the value difference
    considers it as sudden change.And according to the type of sound, various
    gestures and expressions are sent to the cogserver.
'''

class AudioStrength:
    def __init__(self):
        self.hostname = "localhost"
        self.port = 17020
        rospy.Subscriber("audio_sensors", audiodata, self.GetAudioClass)

    def AudioEnergy(self, value):
        # A StateLink is used because evaluation of psi-rules should
        # only depend on the most recent value.
        deci = '(StateLink decibel-value (NumberNode "' + str(value) + '"))\n'
        netcat(self.hostname, self.port, deci)
        print deci

    def GetAudioClass(self, data):
        self.Decibel = data.Decibel
        print "sudden sound change value {}".format(data.suddenchange)

        loud = '(StateLink \"Sudden sound change value\" (NumberNode ' + \
             str(data.suddenchange) + '))\n'
        netcat(self.hostname, self.port, loud)

        self.AudioEnergy(self.Decibel)
        return self.Decibel

if __name__ == '__main__':
    try:
        rospy.init_node('AudioClass', anonymous=True)
        AudioStrength()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
