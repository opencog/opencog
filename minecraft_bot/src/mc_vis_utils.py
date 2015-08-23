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

def init_block_mats():

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


def is_solid(blockid):
    
    #print blockid
    if block_mats[blockid] == True:
            return True
    
    return False

   
def calc_ray_step(pitch, yaw, dist):
    
    pt = radians(pitch)
    yw = radians(yaw)

    if pt < pi/2 and pt > -pi/2:
        dx = -(dist*cos(pt))*sin(yw)
        dz = (dist*cos(pt))*cos(yw)
    else:
        dx = (dist*cos(pt))*sin(yw)
        dz = -(dist*cos(pt))*cos(yw)
 
    dy = -dist*sin(pt)
    
    return dx, dy, dz


def create_vec3_msg(coords, step, num_steps):
    msg = vec3_msg()
    

    msg.x = floor(coords[0] + step[0]*num_steps)
    msg.y = floor(coords[1] + step[1]*num_steps)
    msg.z = floor(coords[2] + step[2]*num_steps)

    #print "x: %d, y: %d, z: %d"%(msg.x, msg.y, msg.z)

    return msg


def get_coordinates_in_range(x, y, z, pitch, yaw):
    
    pit_range = np.arange(pitch - R_PITCH, pitch + R_PITCH + D_PITCH, D_PITCH)
    yaw_range = np.arange(yaw - R_YAW, yaw + R_YAW + D_YAW, D_YAW)
    num_steps = np.arange(0, int(MAX_DIST/D_DIST) + D_DIST)
    
    # ROS messages only support 1-D arrays...
    ray_steps = [calc_ray_step(pt, yw, D_DIST) for pt in pit_range for yw in yaw_range]
    
    all_coords = [create_vec3_msg((x,y,z), step, num) for step in ray_steps for num in num_steps]
    return all_coords


def get_visible_blocks(blocks):
    
    #start = time.time()
    vis_blocks = {}
 
    p_jump = int((2*R_PITCH)/D_PITCH) + 1
    y_jump = int((2*R_YAW)/D_YAW) + 1
    d_jump = int((MAX_DIST)/D_DIST) + 1

    blocks3D = np.reshape(np.array(blocks), (p_jump, y_jump, d_jump))
    
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
                    
                    if is_solid(bid):
                        #print "block is solid, breaking out"
                        break

                
                elif is_solid(vis_blocks[xyz].blockid):
                    #print "bid: %d"%bid
                    #print "found block: %d already at these coordinates"%vis_blocks[xyz].blockid
                    break
    
    vis_blocks_list = vis_blocks.values()

    #end = time.time()
    #print "total: %f"%(end-start)
    
    return vis_blocks_list


