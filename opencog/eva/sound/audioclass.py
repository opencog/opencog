#!/usr/bin/env python
import os
import sys
import rospy

CWD = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(CWD, '..', 'face_track'))
from netcat import netcat
from audio_stream.msg import audiodata

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
        rospy.Subscriber("audio_sensors", audiodata, self.audio_sensors_cb)

    def audio_sensors_cb(self, msg):
        if msg.SuddenChange:
            print "sudden change"
            l = "(cog-evaluate! (EvaluationLink (DefinedPredicateNode \"Show expression\")" + \
            "(ListLink (ConceptNode \"surprised\") (NumberNode 2) (NumberNode 0.5))))\n"
            netcat(self.hostname, self.port, l + "\n")
        else:
            self.AudioEnergy(msg.Decibel)

    def AudioEnergy(self, value):
        # A StateLink is used b/c evaluation of psi-rules should only depend on
        # the most value.
        deci = "(StateLink (AnchorNode \"Decibel value\")" + \
                    "(ListLink (NumberNode {})))\n".format(value)

        netcat(self.hostname, self.port, deci + "\n")

        # TODO: Convert each if/elif clause into a psi-rule.
        if value < 35:
            print "very low sound", value

            x = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "(ListLink (ConceptNode \"happy\") (NumberNode 3) (NumberNode 0.5))))\n"
            netcat(self.hostname, self.port, x + "\n")

        elif value < 65:
            print "Normal conversation:", value

            y = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "(ListLink (ConceptNode \"amused\") (NumberNode 3) (NumberNode 0.5))))\n"
            netcat(self.hostname, self.port, y + "\n")

            m = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
        "(ListLink (ConceptNode \"nod-1\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"
            netcat(self.hostname, self.port, m + "\n")

        elif value < 90:
            print "high sound:", value

            z = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "(ListLink (ConceptNode \"irritated\") (NumberNode 3) (NumberNode 0.5))))\n"
            netcat(self.hostname, self.port, z + "\n")

            n = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
        "(ListLink (ConceptNode \"think-browsDown.001\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"
            netcat(self.hostname, self.port, n + "\n")

        else:
            print 'critical:', value
            t = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "(ListLink (ConceptNode \"afraid\") (NumberNode 3) (NumberNode 0.5))))\n"
            netcat(self.hostname, self.port, t + "\n")

if __name__ == '__main__':
    try:
        rospy.init_node('AudioClass', anonymous=True)
        AudioStrength()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
