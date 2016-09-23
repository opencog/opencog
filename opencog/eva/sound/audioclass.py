#!/usr/bin/env python
from collections import deque
import rospy
import math
import time
import sys
from std_msgs.msg import Float32
from netcat import netcat
from audiosysneeds.msg import audiodata


'''
   This class subscribes to /opencog/AudioFeature and /opencog/suddenchange
    topics, and based on the different range of values of the Decibel classifies
     the input sound into Quiet, Normal Conversation, Loud conversation,critical
      sound and when there is a transfer from one range to another based on the 
      value difference considers it as sudden change.And according to the type of sound, various gestures and expressions are sent to the cogserver.
   
'''

class AudioStrength:
  d = 0
  Decibel = None

  def __init__(self):
    
    self.hostname = "localhost"
    self.port = 17020
    self.loop = 0
    rospy.Subscriber("/opencog/AudioFeature", audiodata, self.GetAudioClass)
    rospy.Subscriber("/opencog/suddenchange", Float32, self.GetSuddenClass)

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

        return 'Quiet Whisper'

    elif value < 65:
        print "Normal conversation:", value

        y = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"amused\") (NumberNode 3) (NumberNode 0.5))))\n"

        netcat(self.hostname, self.port, y + "\n")

        m = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
       "(ListLink (ConceptNode \"nod-1\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"

        netcat(self.hostname, self.port, m + "\n")

        return 'Normal Conversation'

    elif value < 90:
        print "high sound:", value
        z = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"irritated\") (NumberNode 3) (NumberNode 0.5))))\n"
        netcat(self.hostname, self.port, z + "\n")

        n = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
       "(ListLink (ConceptNode \"think-browsDown.001\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"

        netcat(self.hostname, self.port, n + "\n")
        return 'Loud: Shouted Conversation'
    else:
        print 'critical:', value
        t = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"afraid\") (NumberNode 3) (NumberNode 0.5))))\n"
        netcat(self.hostname, self.port, t + "\n")
        return 'Loud: Critical'

  def GetAudioClass(self, data):
    try:
        
        #self.Decibel = data.data
        self.Decibel = data.Decibel
        if self.loop <=2:
            d.append(self.Decibel)

            self.loop += 1
        else:
            d.popleft();d.append(self.Decibel)
            self.loop += 1

    except ArithmeticError as e:
        print(e)
    return self.Decibel

  def GetSuddenClass(self, msg):
    change = msg.data
    print "sudden sound change value {}".format(msg.data)
    loud = "(StateLink (AnchorNode \"Sudden sound change value\")" + \
             "(ListLink(NumberNode {})))\n".format(msg.data)
    
    netcat(self.hostname, self.port, loud + "\n")
    
    if int(change) >= 10:
        l = "(cog-evaluate! (EvaluationLink (DefinedPredicateNode \"Show expression\")" + \
           "(ListLink (ConceptNode \"surprised\") (NumberNode 2) (NumberNode 0.5))))\n"
                  
        netcat(self.hostname, self.port, l + "\n")
    else:
        self.AudioEnergy(self.Decibel)

if __name__ == '__main__':
    global d
    d =deque()

    try:
        rospy.init_node('AudioClass', anonymous=True)
        AudioStrength()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
