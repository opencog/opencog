#! /usr/bin/env python

"""
quick test to send position and look messages to server

"""



import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import position_msg
#from minecraft_bot.msg import map_block_msg


def testLook(x, y, z, pitch, yaw):
    
    msg = position_msg()

    msg.x = x
    msg.y = y
    msg.z = z
    msg.pitch = pitch
    msg.yaw = yaw

    pub_looktest.publish(msg)


pub_looktest = rospy.Publisher('look_test', position_msg, queue_size = 1)

if __name__ == "__main__":
    
    rospy.init_node('look_tester')
    pitch = 0.
    yaw = 0.
    
    x = -14.7
    y = 13
    z = -46.02

    while not rospy.is_shutdown():
        testLook(x, y, z, pitch, yaw)
        yaw = yaw + 5
        #z = z + 1
        rospy.sleep(3.)

