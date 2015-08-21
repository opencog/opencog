#!/usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
import visibility as vis
from minecraft_bot.srv import visible_blocks_srv, get_block_multi_srv
from minecraft_bot.msg import map_block_msg, position_msg, map_block_multi_msg



# client waits for array of blockdata (VarInt) on request
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
    
    coords      = vis.getCoordinatesInRange(req.x, req.y, req.z, req.pitch, req.yaw)
    blocks      = getBlockMulti(coords)
    vis_blocks  = vis.getVisibleBlocks(blocks)

    block_pub.publish(vis_blocks)
    #return {'visible_blocks': vis_blocks}
    



def visibleBlocksServer():

    vis.initBlockMats()
    
    rospy.init_node('visibility_server')
    
    vb_srv = rospy.Service('get_visible_blocks', visible_blocks_srv, handleGetVisibleBlocks)

    print("visibility server initialized")
    print("usage: call getVisibleBlocks() with args: (x, y, z, pitch, yaw)")
    
    rospy.spin()


def visibleBlocksNode():

    vis.initBlockMats()
    
    rospy.init_node('visibility_node')
    rospy.Subscriber('camera_position_data', position_msg, handleGetVisibleBlocks)
    
    print("visibility node initialized")
    
    rospy.spin()




block_pub = rospy.Publisher('camera_vis_data', map_block_multi_msg, queue_size = 100)

if __name__ == "__main__":

    #visibleBlocksServer()
    visibleBlocksNode()

