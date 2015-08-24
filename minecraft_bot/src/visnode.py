#!/usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
import mc_vis_utils as vis
from minecraft_bot.srv import visible_blocks_srv, get_block_multi_srv
from minecraft_bot.msg import map_block_msg, position_msg, map_block_multi_msg


def get_block_multi(coords):

    rospy.wait_for_service('get_block_multi')

    try:
        getBlocksFromSrv = rospy.ServiceProxy('get_block_multi', get_block_multi_srv)
        response = getBlocksFromSrv(coords)
        return response.blocks
    
    except rospy.ServiceException, e:
        print "service call failed: %s"%e


def handle_get_visible_blocks(req):
    
    coords      = vis.get_coordinates_in_range(req.x, req.y, req.z, req.pitch, req.yaw)
    blocks      = get_block_multi(coords)
    vis_blocks  = vis.get_visible_blocks(blocks)

    block_pub.publish(vis_blocks)
    

def visible_blocks_node():

    vis.init_block_mats()
    
    rospy.init_node('visibility_node')
    rospy.Subscriber('camera_position_data', position_msg, handle_get_visible_blocks)
    
    print("visibility node initialized")
    
    rospy.spin()




block_pub = rospy.Publisher('camera_vis_data', map_block_multi_msg, queue_size = 100)

if __name__ == "__main__":

    visible_blocks_node()

