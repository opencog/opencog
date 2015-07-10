#!/usr/bin/env python

"""
created by Bradley Sheneman
calculates set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import get_block_multi_srv
from minecraft_bot.msg import map_block_msg, vec3_msg

from math import sin, cos, radians, pi, floor
import numpy as np
from sys import argv
import time

# radius of vision
MAX_DIST = 10

# step size
D_DIST 	 = 1.
D_PITCH  = 15.
D_YAW 	 = 15.

# angles cover range theta - R_THETA to theta + R_THETA
R_PITCH = 60
R_YAW   = 60

block_mats = {}

def initBlockMats():

    # this is not a comprehensive list, but includes most common solid blocks
    # probably a better way to do this
    blocks = [i for i in range(43+1)]
    solids = []
    for i in range(1,5+1,1):
            solids.append(i)
    solids.append(7)
    for i in range(12,17+1,1):
            solids.append(i)
    solids.append(19)
    for i in range(21,25+1,1):
            solids.append(i)
    solids.append(35)
    for i in range(41,43+1,1):
            solids.append(i)
    

    for blockid in blocks:
        block_mats[blockid] = False
    
    for blockid in solids:
        block_mats[blockid] = True


def isSolid(blockid):
    
    #print blockid
    if block_mats[blockid] == True:
            return True
    
    return False

   
def calcRayStep(pitch, yaw, dist):
    
    pt = radians(pitch)
    yw = radians(yaw)

    if pt < pi/2 and pt > -pi/2:
        dx = -(dist*cos(pt))*sin(yw)
        dz = (dist*cos(pt))*cos(yw)
    else:
        dx = (dist*cos(pt))*sin(yw)
        dz = -(dist*cos(pt))*cos(yw)
 
    dy = -dist*sin(pt)
    #dz = (dist*cos(pt))*cos(yw)
    
    if pitch == 20 and yaw == -127:
        print dx, dy, dz
    
    return dx, dy, dz


def createVec3Msg(coords, step, num_steps):
    msg = vec3_msg()
    
    #print "%f, %f, %f"%(coords[0], coords[1], coords[2])

    msg.x = floor(coords[0] + step[0]*num_steps)
    msg.y = floor(coords[1] + step[1]*num_steps)
    msg.z = floor(coords[2] + step[2]*num_steps)

    #print "x %f, y %f, z %f"%(msg.x, msg.y, msg.z)

    return msg


def getCoordinatesInRange(x, y, z, pitch, yaw):
    
    #all_coords = []
    #pitch = int(pitch)
    #yaw = int(yaw)
    
    pit_range = np.arange(pitch - R_PITCH, pitch + R_PITCH + D_PITCH, D_PITCH)
    yaw_range = np.arange(yaw - R_YAW, yaw + R_YAW + D_YAW, D_YAW)
    num_steps = np.arange(0, int(MAX_DIST/D_DIST) + D_DIST)
    
    #print pit_range
    #print yaw_range
    #print num_steps
    
    # ROS messages only support 1-D arrays...
    ray_steps = [calcRayStep(pt, yw, D_DIST) for pt in pit_range for yw in yaw_range]
    
    #print ray_steps

    #params = [(pt, yw, D_DIST) for pt in pit_range for yw in yaw_range]

    #count = 0
    #for item in params:
    #    count+=1
    #    print count
    #    print item

    #coords = [(x+step[0]*num, y+step[1]*num, z+step[2]*num) for step in ray_steps for num in num_steps]
    
    all_coords = [createVec3Msg((x,y,z), step, num) for step in ray_steps for num in num_steps]
    return all_coords


def getVisibleBlocks(blocks):
    
    #start = time.time()
    vis_blocks = {}
 
    p_jump = int((2*R_PITCH)/D_PITCH) + 1
    y_jump = int((2*R_YAW)/D_YAW) + 1
    d_jump = int((MAX_DIST)/D_DIST) + 1
   
    #print p_jump
    #print y_jump
    #print d_jump
    
    #print len(blocks)

    blocks3D = np.reshape(np.array(blocks), (p_jump, y_jump, d_jump))
    
    #count = 0
    #for ylist in blocks3D:
    #    for dlist in ylist:
    #        print ''
    #        count+=1
    #        print count
    #        for item in dlist:
    #            print "(%d, %d, %d)"%(item.x, item.y, item.z)

    for y_list in blocks3D:
        for d_list in y_list:
            #print ""
            for block in d_list:
                
                xyz = (block.x, block.y, block.z)

                #print xyz
                bid = block.blockid

                if (bid == 0):
                    #print "found air, continuing..."
                    continue
                
                elif xyz not in vis_blocks:
                    #print "bid: %d"%bid
                    #print "new block. adding to list"
                    vis_blocks[xyz] = block
                    
                    if isSolid(bid):
                        #print "block is solid, breaking out"
                        break

                
                elif isSolid(vis_blocks[xyz].blockid):
                    #print "bid: %d"%bid
                    #print "found block: %d already at these coordinates"%vis_blocks[xyz].blockid
                    break
    
    vis_blocks_list = vis_blocks.values()

    #end = time.time()
    print "total: %f"%(end-start)
    
    return vis_blocks_list


"""
# client waits for block data on request
def getBlockData(coords):

    rospy.wait_for_service('get_block_data')

    try:
        x = coords[0]
        y = coords[1]
        z = coords[2]

        getBlockFromSrv = rospy.ServiceProxy('get_block_data', get_block_srv)
        response = getBlockFromSrv(x, y, z)
        return response.block
    
    except rospy.ServiceException, e:
        print "service call failed: %s"%e
"""


