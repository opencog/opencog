#!/usr/bin/env python

"""
created by Bradley Sheneman
calculates set of visible blocks for a givent client position pitch and yaw (head)
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import visible_blocks_msg

from math import sin, cos, radians, pi
import numpy as np
from sys import argv



MAX_DIST = 10
D_DIST 	 = 1
D_PITCH  = 20
D_YAW 	 = 20

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

	if block_mats[block.blockid]:
		return True
	
	return False


def calcRayStep(x, y, z, pitch, yaw):
	
	newx = x - D_DIST*sin(radians(yaw))
	newy = y - D_DIST*sin(radians(pitch))
	newz = z + D_DIST*cos(radians(yaw))

	return newx, newy, newz


def getVisibleBlocks(x, y, z, pitch, yaw):
	
	vis_blocks = {}

	cpitch = pitch - 90
	cyaw = yaw - 90
	cdist = 0
	
	while cpitch <= pitch +90:
		while cyaw <= yaw + 90:
			while cdist <= MAX_DIST:
				cx, cy, cz = calcRayStep(cx, cy, cz, cpitch, cyaw)
				cdist += D_DIST
				cpitch += D_PITCH
				cyaw += D_YAW

				block = getBlock(cx, cy, cz)
				if (cx, cy, cz) not in vis_blocks:
					if isSolid(block):
						vis_blocks[(cx, cy, cz)] = block
						break
				else:
					break


def visibleBlocksServer():

	initBlockMats()
	
	rospy.init_node('visibility_server')
	vb_srv = rospy.Service('get_visible_blocks', visible_blocks_msg, getVisibleBlocks)

	print("visibility server initialized")
	print("call getVisibleBlocks() with args: (x, y, z, pitch, yaw)")
	
	rospy.spin()



if __name__ == "__main__":

	visibleBlocksServer()
