#!/usr/bin/env python
#
# audio_strength.py -- ROS node to send audio to the Cogserver.
#
import rospy
from std_msgs.msg import Float32
from netcat import netcat

# XXX FIXME -- where the heck is audiosysneeds.msg defined ?????
from audiosysneeds.msg import audiodata


'''
  This implements a ROS node that subscribes to the
  `/opencog/AudioFeature` and `/opencog/suddenchange` topics, and
  passes the messages there to the cogserver.

  XXX FIXME -- this currently does some pre-processing, which should
  not be done here!  Specifically, it converts ranges of decibel
  volumes into Quiet, Normal Conversation, Loud conversation, atoms.
  This should be handled by audio processing rules in the atomspace.
'''

class AudioStrength:
  Decibel = None

  def __init__(self):
    self.hostname = "localhost"
    self.port = 17020
    rospy.Subscriber("/opencog/AudioFeature", audiodata, self.GetAudioClass)
    rospy.Subscriber("/opencog/suddenchange", Float32, self.GetSuddenClass)

  def AudioEnergy(self, value):
    # A StateLink is used b/c evaluation of psi-rules should only depend on
    # the most value.
    deci = "(StateLink (AnchorNode \"Decibel value\")" + \
                "(ListLink (NumberNode {})))\n".format(value)

    netcat(self.hostname, self.port, deci + "\n")

    # XXX FIXME: Convert to a psi-rule.
    if value < 35:
        print "Quiet sound", value
        x = "(cog-evaluate! " + \
        "(EvaluationLink (DefinedPredicateNode \"Show expression\")" + \
        "   (ListLink (ConceptNode \"happy\") " + \
        "       (NumberNode 3) (NumberNode 0.5))))\n\n"

        netcat(self.hostname, self.port, x)
        return 'Quiet Whisper'

    elif value < 65:
        print "Normal conversation:", value
        y = "(cog-evaluate! " + \
        "(EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "  (ListLink (ConceptNode \"amused\") " + \
        "     (NumberNode 3) (NumberNode 0.5))))\n"

        y += "(cog-evaluate! " + \
        "(EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
        "   (ListLink (ConceptNode \"nod-1\") " + \
        "      (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"

        netcat(self.hostname, self.port, y)
        return 'Normal Conversation'

    elif value < 90:
        print "Loud sound:", value
        z = "(cog-evaluate! " + \
        "(EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "   (ListLink (ConceptNode \"irritated\") " + \
        "      (NumberNode 3) (NumberNode 0.5))))\n"

        z += "(cog-evaluate! " + \
        "(EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
        "   (ListLink (ConceptNode \"think-browsDown.001\") " + \
        "       (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n\n"

        netcat(self.hostname, self.port, z)
        return 'Loud: Shouted Conversation'

    else:
        print 'Deafening sound:', value
        t = "(cog-evaluate! " + \
        "(EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
        "   (ListLink (ConceptNode \"afraid\") " + \
        "       (NumberNode 3) (NumberNode 0.5))))\n\n"

        netcat(self.hostname, self.port, t)
        return 'Loud: Critical'

  # Just record the loudness, but don't do anything.
  # XXX FIXME -- we should be sending this data (30 Hz, I presume)
  # to the time-server, and handling it just like the face data.
  def GetAudioClass(self, data):
    self.Decibel = data.Decibel
    return self.Decibel

  def GetSuddenClass(self, msg):
    change = msg.data
    print "sudden sound change value {}".format(msg.data)
    loud = "(StateLink (AnchorNode \"Sudden sound change value\")" + \
             "(ListLink(NumberNode {})))\n\n".format(msg.data)

    netcat(self.hostname, self.port, loud)
    if int(change) >= 10:
        loud = "(cog-evaluate! " + \
           "(EvaluationLink (DefinedPredicateNode \"Show expression\")" + \
           "   (ListLink (ConceptNode \"surprised\") " + \
           "       (NumberNode 2) (NumberNode 0.5))))\n\n"
        netcat(self.hostname, self.port, loud)
    else:
        self.AudioEnergy(self.Decibel)

if __name__ == '__main__':
    try:
        rospy.init_node('AudioClass', anonymous = True)
        AudioStrength()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
