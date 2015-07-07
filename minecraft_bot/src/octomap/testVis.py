#!/usr/bin/env python

"""
created by Bradley Sheneman
requests a set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import visible_blocks_srv
from minecraft_bot.msg import map_block_msg


def testVisibleBlocksClient(x, y, z, pitch, yaw):

    rospy.wait_for_service('get_visible_blocks')

    try:
        getVisBlocksFromSrv = rospy.ServiceProxy('get_visible_blocks', visible_blocks_srv)
        vb_response = getVisBlocksFromSrv(x, y, z, pitch, yaw)
        return vb_response.visible_blocks
    except rospy.ServiceException, e:
        print "service call failed: %s"%e



if __name__ == "__main__":

        blocks = testVisibleBlocksClient(-29.2, 14, -39, 20.3, -92)
        print blocks

