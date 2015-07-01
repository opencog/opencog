#!/usr/bin/python

"""
Created by Bradley Sheneman

receives map data and channels it into a single stream of blocks. Modified from smpmap.py in SpockBot
Chunks are packed in X, Z, Y order:

The array walks down X, every 16 elements you enter a new Z-level eg:

[0] - [15] are X = 0-15, Z = 0, Y = 0
[16] - [31] are X = 0-15, Z = 1, Y = 0

...
and so on

Every 256 elements you enter a new Y-level eg:

[0]-[255] are X = 0-15, Z = 0-15, Y = 0
[256]-[511] are X = 0-15, Z = 0-15, Y = 1

...
and so on

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import chunk_data_msg, chunk_bulk_msg, chunk_meta_msg, block_data_msg, map_block_msg

import spock.mcmap.smpmap as smpmap

def unwrapChunk(data, column):
    
    x = data.chunk_x*16
    z = data.chunk_z*16

    for y_iter in range(16):
        
        y = y_iter*16

        sendBlockData(data, column, x, y, z)
                


# note: a block is not necessarily a minecraft block
# e.g. when dealing with chunks, a block is a 16x16 cube of blocks
# therefore, if column exists, coords will be evaluated for a chunk column
# and for a unit block otherwise
def sendBlockData(data, x, y, z, column=None):
    
    msg = octmap_block_msg()
    
    msg.x = x
    msg.y = y
    msg.z = z
   
    # in other words, if this is a single block
    if column is None:
        msg.level = 1

        # in the case of a single block, data is a byte containing both id and meta
        msg.data = data.block_data
    
    # otherwise this is a larger block
    else:
        msg.level = 16
        
        # for a cube, data will be an array of 16*16*16 blocks
        msg.data = column.get_cube(x, y, z)

    
    blockpub.publish(msg)
    # if (msg.x > -80 and msg.x < -1
    #        and msg.y > 0 and msg.y < 12
    #        and msg.z > -96 and msg.z < -19):
    #    print (msg.x, msg.y, msg.z)
    
    # msg.blocklight = light
    # msg.skylight = sky
    
    # if block_id != 0:
    #    blockpub.publish(msg)
    #    print msg



def handleChunkBulk(data):

    skylight = data.sky_light
    bbuff = utils.BoundBuffer(data.data)
    
    for meta in data.metadata:
        # Read chunk metadata
        
        mask = meta.primary_bitmap
        
        # Unpack the chunk column data
        col = ChunkColumn()
        col.unpack(bbuff, mask, skylight)
        
        # unroll and send the block data
        unwrapChunk(meta, col)



def handleChunkData(data):
    
    continuous = data.continuous
    bbuff = utils.BoundBuffer(data.data)
    
    if self.dimension == DIMENSION_OVERWOLD:
        skylight = True
    else:
        skylight = False
    key = (x_chunk, z_chunk)
    
    col = ChunkColumn()
    col.unpack(bbuff, mask, skylight, continuous)
    
    unwrapChunk(data, col)
    
    

def handleBlockData(data):
    
    if data == None:
        data = (block_id<<4)|(meta&0x0F)
    
    msg = map_block_msg()
    msg.x = x + chunk_x
    msg.y = y
    msg.z = z + chunk_z
    
    msg.blocklight = 0
    msg.skylight = 0

    msg.blockid = data.blockid 
    
    # in other words, if not air...
    if msg.blockid != 0:
        blockpub.publish(msg)




if __name__ == "__main__":
    rospy.init_node('map_data_node')

    #rospy.Subscriber('chunk_data', chunk_data_msg, handleChunkData)
    rospy.Subscriber('chunk_bulk', chunk_bulk_msg, handleChunkBulk)
    #rospy.Subscriber('block_data', block_data_msg, handleBlockData)

    blockpub = rospy.Publisher('block', map_block_msg, queue_size = 100000)
    
    rospy.spin()

