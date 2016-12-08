##Simple sound system for OpenCog
------------

  This package is built with the intention of addressing the following requirements.

1. Sudden Change Detection
  To make the robot detect significant audio changes. e.g some one yelling at the robot  all of the sudden.
2. Sound Classification
  Classifying each chunk of sound as quiet, normal conversation, high sound and critical sound.

##Requirements and Setup
------------
 - Middle ware     - ROS/indigo
 - Dev. Language   - Python v2.7/rospy
 - Audio Package   - PyAudiov0.2.9

##Building
------------ 

 cd your_work_space &&  source devel/setup.bash

##Running
------------
  - add the directory of netcat into PYTHONPATH in .bashrc file
  - cd hansonrobotics/HEAD && ./scripts/dev.sh

##Topics 
1. /opencog/AudioFeature
  It contains Decibel and Frequency of a given chunk as Float
2. /opencog/suddenchange
  It holds 0 for No Sudden Change Events and 10 for sudden change

##Employed Techniques and Algorithms
------------
- [Root Mean Square (RMS)](http://www.gaussianwaves.com/2015/07/significance-of-rms-root-mean-square-value/)
- [Frequency Detection] (https://gist.github.com/endolith/255291) 


