#!/usr/bin/env python
from collections import deque
import rospy
import math
import numpy as np
import time
import sys
import struct
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from std_msgs.msg import Float32
from netcat import netcat
from audio_stream.msg import audiodata


'''
   This class subscribes to /opencog/AudioFeature and /opencog/suddenchange
    topics, and based on the different range of values of the Decibel classifies
     the input sound into Quiet, Normal Conversation, Loud conversation,critical
      sound and when there is a transfer from one range to another based on the 
      value difference considers it as sudden change.And according to the type of
       sound, various gestures and expressions are sent to the cogserver.
   
'''

class AudioStrength:
  d = 0
  Decibel = None
  value = 0

  def __init__(self):
    
    self.hostname = "localhost"
    self.port = 17020
    self.loop = 0
    rospy.Subscriber("/opencog/AudioFeature", audiodata, self.GetAudioClass)
 
  def AudioEnergy(self, value):
    # A StateLink is used b/c evaluation of psi-rules should only depend on
    # the most value.
    
    deci = '(StateLink decibel-value (NumberNode "' + \
		       str(value) + '"))'
    
    netcat(self.hostname, self.port, deci + "\n")
    print deci
    
  def GetAudioClass(self, data):
    try:
        
        self.Decibel = data.Decibel
        if self.loop <=2:
            d.append(self.Decibel)

            self.loop += 1
        else:
            d.popleft();d.append(self.Decibel)
            self.loop += 1
        change = data.suddenchange
        print "sudden sound change value {}".format(data.suddenchange)
    	
        loud = '(StateLink Sudden sound change value (NumberNode "' + \
		       str(change) + '"))'

    	netcat(self.hostname, self.port, loud + "\n")

    	self.AudioEnergy(self.Decibel)


    except ArithmeticError as e:
        print(e)
    return self.Decibel

if __name__ == '__main__':
    d =deque()
    try:
        rospy.init_node('AudioClass', anonymous=True)
        AudioStrength()

        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
