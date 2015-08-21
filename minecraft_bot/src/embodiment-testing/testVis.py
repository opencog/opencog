#!/usr/bin/env python

"""
created by Bradley Sheneman
requests a set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import visible_blocks_srv
from minecraft_bot.msg import map_block_msg, map_block_multi_msg, position_msg

def handleVisBlocks(data):

    print data


def sendCoords():

    msg = position_msg()
    msg.x = -29.2
    msg.y = 14
    msg.z = -41
    msg.pitch = 20.3
    msg.yaw = -92

    coord_pub.publish(msg)
    rospy.loginfo(msg)



def testVisibleBlocks():

    #rospy.wait_for_service('get_visible_blocks')

    rospy.init_node('test_vis_blocks_node')    
    print "test visibility node initialized"

    while not rospy.is_shutdown():
        sendCoords()
        rospy.sleep(5.)

    #rospy.spin()
    

coord_pub = rospy.Publisher('camera_position_data', position_msg, queue_size = 1)
rospy.Subscriber('camera_vis_data', map_block_multi_msg, handleVisBlocks)

if __name__ == "__main__":

        testVisibleBlocks()
        #blocks = testVisibleBlocksClient(-29.2, 14, -41, 20.3, -92)
        #print blocks

