#!/usr/bin/env python

"""
Created by Bradley Sheneman

Provides a client/server api for Minecraft world data
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import chunk_data_msg, chunk_bulk_msg, chunk_meta_msg, block_data_msg, map_block_msg


from spock.utils import pl_announce
from spock.mcmap import smpmap, mapdata
from spock.mcp import mcdata

DIMENSION_NETHER   = -0x01
DIMENSION_OVERWOLD =  0x00
DIMENSION_END      =  0x01



def handle_add_two_ints(req):
        print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
            return AddTwoIntsResponse(req.a + req.b)

        def add_two_ints_server():
                rospy.init_node('add_two_ints_server')
                    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
                        print "Ready to add two ints."
                            rospy.spin()

                            if __name__ == "__main__":
                                    add_two_ints_server()



# modified from class 'Dimension' in smpmap.py from spock
# some functions added, and changed to handle ROS messages
# will serve as a dynamic (and fast) storage for the current rendered world
class MinecraftMap(object):
    
    def __init__(self, dimension):
        
        self.dimension = dimension
        self.columns = {}        
   

    def handleUnpackBulk(self, data):
        
        skylight = data.sky_light
        bbuff = utils.BoundBuffer(data.data)
        
        for meta in data.metadata:
            chunk_x = meta.chunk_x
            chunk_z = meta.chunk_z
            mask = meta.primary_bitmap
            
            key = (chunk_x, chunk_z)
            
            if key not in self.columns:
                self.columns[key] = ChunkColumn()
            
            self.columns[key].unpack(bbuff, mask, skylight)


    def handleUnpackChunk(self, data):
        
        chunk_x = data.chunk_x
        chunk_z = data.chunk_z
        mask = data.primary_bitmap
        continuous = data.continuous
        bbuff = utils.BoundBuffer(data.data)
        
        if self.dimension == DIMENSION_OVERWOLD:
            skylight = True
        else:
            skylight = False
        
        key = (x_chunk, z_chunk)
        
        if key not in self.columns:
            self.columns[key] = ChunkColumn()
        
        self.columns[key].unpack(bbuff, mask, skylight, continuous)
    
    
    def handleUnpackBlock(self, data):
        
        #becomes (chunk number, offset in chunk)
        x, rx = divmod(data.x, 16)
        y, ry = divmod(data.y, 16)
        z, rz = divmod(data.z, 16)

        if y > 0x0F:
            return
        
        if (x,z) in self.columns:
            column = self.columns[(x,z)]
        else:
            column = ChunkColumn()
            self.columns[(x,z)] = column
        
        chunk = column.chunks[y]
        
        if chunk == None:
            chunk = Chunk()
            column.chunks[y] = chunk
        
        chunk.block_data.set(rx, ry, rz, data.data)


    # note, returns block ID and meta in the same byte (data) for consistency
    def getBlock(self, x, y, z):
        
        x, y, z = int(x), int(y), int(z)
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)
        
        if (x, z) not in self.columns or y > 0x0F:
            return 0
        
        column = self.columns[(x,z)]
        chunk = column.chunks[y]
        
        if chunk == None:
            return 0
        
        data = chunk.block_data.get(rx,ry,rz)
        return data


    def getLight(self, x, y, z):
        
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)
        
        if (x, z) not in self.columns or y > 0x0F:
            return 0, 0
        
        column = self.columns[(x,z)]
        chunk = column.chunks[y]
        
        if chunk == None:
            return 0, 0
        
        return chunk.light_block.get(rx,ry,rz), chunk.light_sky.get(rx,ry,rz)
    
    
    # y is just a dummy variable, for consistency of the API
    def getBiome(self, x, y, z):
        
        x, rx = divmod(x, 16)
        z, rz = divmod(z, 16)
        
        if (x,z) not in self.columns:
            return 0
        
        return self.columns[(x,z)].biome.get(rx, rz)


    def setLight(self, x, y, z, light_block = None, light_sky = None):
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)

        if y > 0x0F:
            return
        
        if (x,z) in self.columns:
            column = self.columns[(x, z)]
        else:
            column = ChunkColumn()
            self.columns[(x, z)] = column
        
        chunk = column.chunks[y]
        
        if chunk == None:
            chunk = Chunk()
            column.chunks[y] = chunk

        if light_block != None:
            chunk.light_block.set(rx, ry, rz, light_block&0xF)
        
        if light_sky != None:
            chunk.light_sky.set(rx, ry, rz, light_sky&0xF)


    def setBiome(self, x, z, data):
        
        x, rx = divmod(x, 16)
        z, rz = divmod(z, 16)

        if (x,z) in self.columns:
            column = self.columns[(x,z)]
        else:
            column = ChunkColumn()
            self.columns[(x,z)] = column

        return column.biome.set(rx, rz, data)



if __name__ == "__main__":
    rospy.init_node('minecraft_map_node')

    rospy.Subscriber('chunk_data', chunk_data_msg, handleChunkData)
    rospy.Subscriber('chunk_bulk', chunk_bulk_msg, handleChunkBulk)
    rospy.Subscriber('block_data', block_data_msg, handleBlockData)

    blockpub = rospy.Publisher('block', map_block_msg, queue_size = 100000)
    
    rospy.spin()

