#!/usr/bin/env python
#
# audio_strength.py -- ROS node to send audio to the Cogserver.
#
from collections import deque
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
  d = 0
  Decibel = None

  def __init__(self):
    self.hostname = "localhost"
    self.port = 17020
    self.loop = 0
    self.d = deque()
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

  def GetAudioClass(self, data):
    # Implement a short FIFO, for recognizing sudden sound changes.
    # The FIFO simply records the last two sound values.
    # XXX This is not used anywhere ...
    # XXX anyway, sudden-change should be done inside of opencog,
    # in the time-server, and not here. FIXME.
    try:
        self.Decibel = data.Decibel
        if self.loop > 2:
            self.d.popleft()

        self.d.append(self.Decibel)
        self.loop += 1

    except ArithmeticError as e:
        print(e)
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
