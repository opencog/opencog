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



MAX_DIST = 6
D_DIST 	 = 1
D_PITCH  = 30
D_YAW 	 = 30

block_mats = {}

def initBlockMats():

    # this is not a comprehensive list, but includes most common solid blocks
    # probably a better way to do this
    blocks = [i for i in range(43)]
    solids = []
    for i in range(1,5,1):
            solids.append(i)
    solids.append(7)
    for i in range(12,17,1):
            solids.append(i)
    solids.append(19)
    for i in range(21,25,1):
            solids.append(i)
    solids.append(35)
    for i in range(41,43,1):
            solids.append(i)
    

    for blockid in blocks:
        block_mats[blockid] = False
    
    for blockid in solids:
        block_mats[blockid] = True


def isSolid(block):
    
    print block.blockid
    if block_mats[block.blockid] == True:
            return True
    
    return False


def calcRayStep(x, y, z, pitch, yaw):
    
    newx = x - D_DIST*sin(radians(yaw))
    newy = y - D_DIST*sin(radians(pitch))
    newz = z + D_DIST*cos(radians(yaw))

    return newx, newy, newz


def getVisibleBlocks(x, y, z, pitch, yaw):
    
    vis_blocks = {}
    
    cx = x
    cy = y
    cz = z
    cpitch = pitch - 90
    cyaw = yaw - 90
    cdist = 0
    
    #print "pitch: %f, yaw: %f"%(pitch, yaw)
    while cpitch <= pitch + 90:
        while cyaw <= yaw + 90:
            while cdist <= MAX_DIST:
                
                #print ("cx: %f, cy: %f, cz: %f")%(cx, cy, cz)
                #print ("cdist: %f, cyaw: %f, cpitch: %f")%(cdist, cyaw, cpitch)
                cx, cy, cz = calcRayStep(cx, cy, cz, cpitch, cyaw)
                cdist += D_DIST

                block = getBlockData(floor(cx), floor(cy), floor(cz))
                if (floor(cx), floor(cy), floor(cz)) not in vis_blocks:
                    if isSolid(block):
                        #print "solid block not already in list"
                        vis_blocks[(floor(cx), floor(cy), floor(cz))] = block
                        break
                else:
                    #print "block found in list"
                    break
                
            cdist = 0
            cx = x
            cy = y
            cz = z
            cyaw += D_YAW
        
        cyaw = yaw - 90
        cpitch += D_PITCH

    #print "list of visible solid blocks:"
    vis_blocks = [vis_blocks[key] for key in vis_blocks]
    #print vis_blocks
    
    return vis_blocks


# this client waits for block data on request
def getBlockData(x, y, z):

    rospy.wait_for_service('get_block_data')

    try:
        getBlockFromSrv = rospy.ServiceProxy('get_block_data', get_block_srv)
        response = getBlockFromSrv(x, y, z)
        #print response.block
        return response.block
    except rospy.ServiceException, e:
        print "service call failed: %s"%e



# handle to call local function
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
