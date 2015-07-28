#!/usr/bin/env python

"""
Created by Bradley Sheneman
action server. receives requests for simple actions and returns success or failure
another node should be responsible for building plans, and send the deconstructed plan
to this server one action at a time

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
#from minecraft_bot.srv import visible_blocks_srv, get_block_multi_srv
#from minecraft_bot.msg import map_block_msg, vec3_msg

from minecraft_bot.msg import position_msg, movement_msg
from minecraft_bot.srv import rel_look_srv, rel_move_srv
import visibility as vis

#from spock.utils import pl_announce, BoundBuffer
#from spock.mcmap import smpmap, mapdata
#from spock.mcp import mcdata

class ClientMover():

    def __init__(self):

        self.x = None
        self.y = None
        self.z = None
        self.pitch = None
        self.yaw = None
        
        rospy.Subscriber('client_pos_data', position_msg, self.handleUpdatePosition)

    

    def handleUpdatePosition(self, data):
        
        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.pitch = data.pitch
        self.yaw = data.yaw
        
        #print "current pos: %f, %f, %f"%(self.x, self.y, self.z)


def inRange(first, second):
    
    if first > second*0.95 and first < second*1.05:
        return True
    else:
        return False



def handleRelativeLook(req):
    
    yaw = req.yaw
    pitch = req.pitch

    timer = 0.
    while timer < 3:
        abs_pitch = clientpos.pitch
        abs_yaw = clientpos.yaw

        # 95% interval of actual desired value
        if inRange(abs_yaw, yaw) and inRange(abs_pitch, pitch):
            return True

        desired_pitch = abs_pitch - pitch
        desired_yaw = abs_yaw + yaw

        if desired_pitch < -90:
            desired_pitch = -90
        elif desired_pitch > 90:
            desired_pitch = 90

        if desired_yaw > 180:
            while desired_yaw > 180:
                desired_yaw -= 360
        elif desired_yaw < -180:
            while desired_yaw < -180:
                desired_yaw += 360
        
        msg = movement_msg()
        msg.x = 0
        msg.y = 0
        msg.z = 0
        msg.pitch = desired_pitch
        msg.yaw = desired_yaw
        msg.jump = False
        msg.speed = 1

        print "current pitch: %f, current yaw: %f"%(abs_pitch, abs_yaw)
        print "desired pitch: %f, desired yaw: %f"%(desired_pitch, desired_yaw)
        pub_move.publish(msg)
        
        timer += 0.2
        rospy.sleep(0.2)

    
    # if we have not reached correct position in 3 seconds
    return False

# jump is either True or False
def handleRelativeMove(yaw, dist, jump):
    
    abs_yaw = clientpos.yaw
    abs_x = clientpos.x
    abs_y = clientpos.y
    abs_z = clientpos.z

    desired_yaw = abs_yaw + yaw

    if desired_yaw > 180:
        while desired_yaw > 180:
            desired_yaw -= 360
    elif desired_yaw < -180:
        while desired_yaw < -180:
            desired_yaw += 360
    
    # calculate change in x, y, z desired using pitch of zero (no movement in y dir)
    # this will work the same for jumps, but dist will be small
    dx, dy, dz = vis.calcRayStep(0, desired_yaw, dist)
   
    desired_x = abs_x + dx
    desired_y = abs_x + dy
    desired_z = abs_x + dz

    msg = movement_msg()
    msg.x = desired_x
    msg.y = desired_y
    msg.z = desired_z
    msg.pitch = 0
    msg.yaw = desired_yaw
    msg.jump = jump
    msg.speed = 1

    print msg
    pub_move.publish(msg)



def actionServer():

    #vis.initBlockMats()
    
    #client = ClientMover()

    rospy.init_node('action_server')
    
    # look handles changes in head position
    # move handles changes in body location and jump commands
    look_srv = rospy.Service('set_relative_look', rel_look_srv, handleRelativeLook)
    move_srv = rospy.Service('set_relative_move', rel_move_srv, handleRelativeMove)

    print("action server initialized")
    
    rospy.spin()


clientpos = ClientMover()
pub_move = rospy.Publisher('movement_data', movement_msg, queue_size = 10)

if __name__ == "__main__":

	actionServer()
