#! /usr/bin/env python
"""
created by Bradley Sheneman
small script to test the action server
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import look_srv, rel_move_srv, abs_move_srv
#from minecraft_bot.msg import map_block_msg


def testRelativeLookClient(pitch, yaw):

    rospy.wait_for_service('set_relative_look')

    try:
        setRelativeLook = rospy.ServiceProxy('set_relative_look', look_srv)
        response = setRelativeLook(pitch, yaw)
        return response.state
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

def testRelativeMoveClient(yaw, dist, jump):

    rospy.wait_for_service('set_relative_move')

    try:
        setRelativeMove = rospy.ServiceProxy('set_relative_move', rel_move_srv)
        response = setRelativeMove(yaw, dist, jump)
        return response.state
    except rospy.ServiceException, e:
        print "service call failed: %s"%e


def testLookClient(pitch, yaw):

    rospy.wait_for_service('set_look')

    try:
        setLook = rospy.ServiceProxy('set_look', look_srv)
        response = setLook(pitch, yaw)
        return response.state
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

def testMoveClient(x, z, jump):

    rospy.wait_for_service('set_move')

    try:
        setMove = rospy.ServiceProxy('set_move', abs_move_srv)
        response = setMove(x, z, jump)
        return response.state
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

def testMove():

    while not rospy.is_shutdown():
        move_steps = [
            (-14, -33, True),
            (-18, -35, True),
            (-18, -30, True),
            (-14, -38, True)]
        
        for step in move_steps:
            response = testMoveClient(*step)
            print "service move respond with %s"%response
            rospy.sleep(10.)

def testRelativeMove():

    while not rospy.is_shutdown():
        move_steps = [
            (0,3,True),
            (90,3,False),
            (90,3,True),
            (0,3,False)]
        for step in move_steps:
            response = testRelativeMoveClient(*step)
            print "service rel_move respond with %s"%response
            rospy.sleep(3.)

def testRelativeLook():
        
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


def resetLook():
    
    resp = testLookClient(0.,0.)
    if resp:
        print "reset bot head pose"
    else:
        print "could not reset bot head pose"
    

if __name__ == "__main__":

    resetLook()
    testMove()
    #testRelativeLook()
