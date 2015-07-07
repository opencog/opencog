#!/usr/bin/env python

"""
created by Bradley Sheneman
calculates set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import visible_blocks_srv, get_block_srv
from minecraft_bot.msg import map_block_msg

from math import sin, cos, radians, pi, floor
import numpy as np
from sys import argv
#import time

# radius of vision
MAX_DIST = 6

# step size
D_DIST 	 = 1
D_PITCH  = 15
D_YAW 	 = 15

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


def calcRayStep(coords, pitch, yaw):
    
    newx = coords[0] - D_DIST*sin(radians(yaw))
    newy = coords[1] - D_DIST*sin(radians(pitch))
    newz = coords[2] + D_DIST*cos(radians(yaw))

    return (newx, newy, newz)


def getVisibleBlocks(x, y, z, pitch, yaw):
    
    vis_blocks = {}
    
    
    cur_coords = (x, y, z)
    pitch = int(pitch)
    yaw = int(yaw)
    
    for cpitch in np.arange(pitch - R_PITCH, pitch + R_PITCH + 1, D_PITCH):
        for cyaw in np.arange(yaw - R_YAW, yaw + R_YAW + 1, D_YAW):
            for cdist in np.arange(0, MAX_DIST + 1, D_DIST):
                
                #print ("cx: %f, cy: %f, cz: %f")%(cx, cy, cz)
                #print ("cdist: %d, cyaw: %d, cpitch: %d")%(cdist, cyaw, cpitch)
                
                cur_coords = calcRayStep(cur_coords, cpitch, cyaw)
                int_coords = tuple(np.floor(cur_coords))
                
                block = getBlockData(int_coords)
                bid = block.blockid

                if (bid == 0):
                    #print "found air, continuing..."
                    continue
                
                elif int_coords not in vis_blocks:
                    #print "bid: %d"%bid
                    #print "new block. adding to list"
                    vis_blocks[int_coords] = block
                    
                    if isSolid(bid):
                        #print "block is solid, breaking out"
                        break
                
                elif isSolid(vis_blocks[int_coords].blockid):
                    #print "bid: %d"%bid
                    #print "found block: %d already at these coordinates"%vis_blocks[int_coords].blockid
                    break
                
            cur_coords = (x, y, z)


    blocks = vis_blocks.items()

    return blocks



# this client waits for block data on request
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



# visibility service handler to call local function
def handleGetVisibleBlocks(req):

    return getVisibleBlocks(req.x, req.y, req.z, req.pitch, req.yaw)
    

def visibleBlocksServer():

    initBlockMats()
    
    rospy.init_node('visibility_server')
    
    vb_srv = rospy.Service('get_visible_blocks', visible_blocks_srv, handleGetVisibleBlocks)

    print("visibility server initialized")
    print("usage: call getVisibleBlocks() with args: (x, y, z, pitch, yaw)")
    
    rospy.spin()



if __name__ == "__main__":

	visibleBlocksServer()
