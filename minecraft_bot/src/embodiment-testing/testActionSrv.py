#! /usr/bin/env python
"""
created by Bradley Sheneman
small script to test the action server
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import rel_look_srv, rel_move_srv
#from minecraft_bot.msg import map_block_msg


def testRelativeLookClient(yaw, pitch):

    rospy.wait_for_service('set_relative_look')

    try:
        setRelativeLook = rospy.ServiceProxy('set_relative_look', rel_look_srv)
        response = setRelativeLook(yaw, pitch)
        return response.state
    except rospy.ServiceException, e:
        print "service call failed: %s"%e


if __name__ == "__main__":
        
    while not rospy.is_shutdown():

        look_positions = [
                (-20,0),
                (20,0),
                (0,30),
                (0,-30)]

        for pos in look_positions:
            response = testRelativeLookClient(pos[0], pos[1])
            print "service responded with: %s"%response
            rospy.sleep(3.)

