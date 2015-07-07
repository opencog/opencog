#!/usr/bin/env python

"""
created by Bradley Sheneman
calculates set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import visible_blocks_srv, get_block_srv, get_block_multi_srv
from minecraft_bot.msg import map_block_msg, vec3_msg

from math import sin, cos, radians, pi, floor
import numpy as np
from sys import argv
import time

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


def calcRayStep(x, y, z, pitch, yaw, dist):
    
    msg = vec3_msg()

    msg.x = int(x - dist*sin(radians(yaw)))
    msg.y = int(y - dist*sin(radians(pitch)))
    msg.z = int(z + dist*cos(radians(yaw)))

    return msg


def getVisibleBlocks(x, y, z, pitch, yaw):
    
    start = time.time()
    vis_coords = []
    vis_blocks = {}
    
    cur_coords = (x, y, z)
    pitch = int(pitch)
    yaw = int(yaw)
    
    p_range = xrange(pitch - R_PITCH, pitch + R_PITCH + 1, D_PITCH)
    y_range = xrange(yaw - R_YAW, yaw + R_YAW + 1, D_YAW)
    d_range = xrange(0, MAX_DIST + 1, D_DIST)

    # ROS messages only support 1-D arrays...
    p_jump = len(p_range)
    y_jump = len(y_range)
    d_jump = len(d_range)

    all_coords = [calcRayStep(x, y, z, pit, yaw, dis) for pit in p_range for yaw in y_range for dis in d_range]

    #params = np.array([(pit, yaw, dis) for pit in p_range for yaw in y_range for dis in d_range],
    #        dtype=('i4,i4,i4'))

    #print np.shape(params)
    #print params
    #reshaped_params = np.reshape(params, (p_jump, y_jump, d_jump))
    #print np.shape(params)
    #print reshaped_params

    #print "pj: %d, yj: %d, dj: %d, total length: %d"%(p_jump, y_jump, d_jump, len(params))
    #print np.reshape(np.array(params), (p_jump, y_jump, d_jump))

    
    all_blocks = getBlockMulti(all_coords)
    #print all_blocks
    blocks_1d = np.array(all_blocks)
    #print blocks_1d
    #print np.shape(blocks_1d)
    
    blocks_3d = np.reshape(blocks_1d, (p_jump, y_jump, d_jump))
    #print blocks_3d
    #print np.shape(blocks_3d)
    #print blocks_3d[1][1]
    
    for y_list in blocks_3d:
        for d_list in y_list:
            for block in d_list:
                coords = (block.x, block.y, block.z)
                
                #print block
                #block = all_blocks[pos]
                bid = block.blockid

                #print "block id: %d"%bid
                #print coords

                if (bid == 0):
                    #print "found air, continuing..."
                    continue
                
                elif coords not in vis_blocks:
                    #print "bid: %d"%bid
                    #print "new block. adding to list"
                    vis_blocks[coords] = block
                    
                    if isSolid(bid):
                        #print "block is solid, breaking out"
                        break

                
                elif isSolid(vis_blocks[coords].blockid):
                    #print "bid: %d"%bid
                    #print "found block: %d already at these coordinates"%vis_blocks[coords].blockid
                    break
    
    #print vis_blocks[(-28,12,-40)]
    vis_blocks_list = vis_blocks.values()

    end = time.time()
    print "total: %f"%(end-start)
    
    #print vis_blocks_list
    return vis_blocks_list



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


# client waits for grouped blocks (dict) on request
def getBlockMulti(coords):

    rospy.wait_for_service('get_block_multi')

    try:
        getBlocksFromSrv = rospy.ServiceProxy('get_block_multi', get_block_multi_srv)
        response = getBlocksFromSrv(coords)
        return response.blocks
    
    except rospy.ServiceException, e:
        print "service call failed: %s"%e




# visibility service handler to call local function
def handleGetVisibleBlocks(req):

    return {'visible_blocks': getVisibleBlocks(req.x, req.y, req.z, req.pitch, req.yaw)}
    

def visibleBlocksServer():

    initBlockMats()
    
    rospy.init_node('visibility_server')
    
    vb_srv = rospy.Service('get_visible_blocks', visible_blocks_srv, handleGetVisibleBlocks)

    print("visibility server initialized")
    print("usage: call getVisibleBlocks() with args: (x, y, z, pitch, yaw)")
    
    rospy.spin()



if __name__ == "__main__":

	visibleBlocksServer()
