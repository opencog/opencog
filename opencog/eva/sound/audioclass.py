#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float32
from netcat import netcat

# XXX FIXME -- where the heck is audio_stream.msg defined ?????
from audio_stream.msg import audiodata

# CWD = os.path.dirname(os.path.abspath(__file__))
# sys.path.insert(0, os.path.join(CWD, '..', 'face_track'))


'''
    This implements a ROS node that subscribes to the `audio_sensors`
    topic, and passes the audio power data to the cogserver. This is
    used by OpenCog to react to loud sounds, sudden changes, and
    general background noise levels.

    An enhancement would be a a neural net that responded to clapping,
    cheering, or other common sound events, identified them, labelled
    them, and passed them on into the atomspace.
'''

class AudioStrength:
    def __init__(self):
        self.hostname = "localhost"
        self.port = 17020
        rospy.Subscriber("audio_sensors", audiodata, self.GetAudioClass)

    def AudioEnergy(self, value):
        # A StateLink is used because evaluation of psi-rules should
        # only depend on the most recent value.
        deci = '(StateLink (AnchorNode "Decibel value") (NumberNode "' + str(value) + '"))\n'
        netcat(self.hostname, self.port, deci)
        print deci

    def GetAudioClass(self, data):
        self.Decibel = data.Decibel
        print "Sudden sound change {}".format(data.suddenchange)

        loud = '(StateLink (AnchorNode \"Sudden sound change value\") (NumberNode ' + \
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
