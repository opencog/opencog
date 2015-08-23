#!/usr/bin/env python

"""
Created by Bradley Sheneman

Provides a client/server api for Minecraft world data
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import chunk_data_msg, chunk_bulk_msg, chunk_meta_msg, block_data_msg, map_block_msg
from minecraft_bot.srv import get_block_srv, get_block_multi_srv

from spock.utils import pl_announce, BoundBuffer
from spock.mcmap import smpmap, mapdata
from spock.mcp import mcdata

import time

DIMENSION_NETHER    = -0x01
DIMENSION_OVERWORLD =  0x00
DIMENSION_END       =  0x01


# modified from class 'Dimension' in smpmap.py from spock
# some functions added, and changed to handle ROS messages
# will serve as a dynamic (and fast) storage for the current rendered world
class MinecraftMap(object):
    
    def __init__(self, dimension):
        
        self.dimension = dimension
        self.columns = {}        
   

    def handle_unpack_bulk(self, data):
        
        #print "unpacking bulk"        
        skylight = data.sky_light
        bbuff = BoundBuffer(data.data)
        
        #print "light: %s: buffer:"%skylight
        #print bbuff

        for meta in data.metadata:
            chunk_x = meta.chunk_x
            chunk_z = meta.chunk_z
            mask = meta.primary_bitmap
            
            #print "unpacking chunk meta x: %d, z: %d, mask: %d"%(chunk_x, chunk_z, mask)

            key = (chunk_x, chunk_z)
            
            if key not in self.columns:
                self.columns[key] = smpmap.ChunkColumn()
            
            self.columns[key].unpack(bbuff, mask, skylight)



    def handle_unpack_chunk(self, data):
        
        chunk_x = data.chunk_x
        chunk_z = data.chunk_z
        mask = data.primary_bitmap
        continuous = data.continuous
        bbuff = BoundBuffer(data.data)
        
        if self.dimension == DIMENSION_OVERWORLD:
            skylight = True
        else:
            skylight = False
        
        key = (chunk_x, chunk_z)
        
        if key not in self.columns:
            self.columns[key] = smpmap.ChunkColumn()
        
        self.columns[key].unpack(bbuff, mask, skylight, continuous)

        #print "unpacking chunk full x: %d, z: %d, mask: %d, cont: %s"%(chunk_x, chunk_z, mask, continuous)
        #print "light: %d, buffer:"%skylight
        #print bbuff
    
    def handle_unpack_block(self, data):
        
        #becomes (chunk number, offset in chunk)
        x, rx = divmod(data.x, 16)
        y, ry = divmod(data.y, 16)
        z, rz = divmod(data.z, 16)

        if y > 0x0F:
            return
        
        if (x,z) in self.columns:
            column = self.columns[(x,z)]
        else:
            column = smpmap.ChunkColumn()
            self.columns[(x,z)] = column
        
        chunk = column.chunks[y]
        
        if chunk == None:
            chunk = smpmap.Chunk()
            column.chunks[y] = chunk
        
        chunk.block_data.set(rx, ry, rz, data.data)

        #print "unpacking block x: %d, y: %d, z: %d, data: %d"%(data.x, data.y, data.z, data.data)


    # note, returns block ID and meta in the same byte (data) for consistency
    def get_block(self, x, y, z):
        
        x, y, z = int(x), int(y), int(z)
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)
        
        if (x, z) not in self.columns or y > 0x0F:
            return 0, 0
        
        column = self.columns[(x,z)]
        chunk = column.chunks[y]
        
        if chunk == None:
            return 0, 0
        
        data = chunk.block_data.get(rx, ry, rz)

        return data >> 4, data&0x0F


    def get_light(self, x, y, z):
        
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
    def get_biome(self, x, y, z):
        
        x, rx = divmod(x, 16)
        z, rz = divmod(z, 16)
        
        if (x,z) not in self.columns:
            return 0
        
        return self.columns[(x,z)].biome.get(rx, rz)


    def set_light(self, x, y, z, light_block = None, light_sky = None):
        
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)

        if y > 0x0F:
            return
        
        if (x,z) in self.columns:
            column = self.columns[(x, z)]
        else:
            column = smpmap.ChunkColumn()
            self.columns[(x, z)] = column
        
        chunk = column.chunks[y]
        
        if chunk == None:
            chunk = smpmap.Chunk()
            column.chunks[y] = chunk

        if light_block != None:
            chunk.light_block.set(rx, ry, rz, light_block&0xF)
        
        if light_sky != None:
            chunk.light_sky.set(rx, ry, rz, light_sky&0xF)


    def set_biome(self, x, z, data):
        
        x, rx = divmod(x, 16)
        z, rz = divmod(z, 16)

        if (x,z) in self.columns:
            column = self.columns[(x,z)]
        else:
            column = smpmap.ChunkColumn()
            self.columns[(x,z)] = column

        return column.biome.set(rx, rz, data)


# the service
def get_block(req):

    #start = time.time()

    msg = map_block_msg()
    msg.x = req.x
    msg.y = req.y
    msg.z = req.z
    
    msg.blockid, msg.metadata = world.get_block(msg.x, msg.y, msg.z)
    #print msg

    #end = time.time()
    #print"call to getBlock(): %f"%(end-start)
    return msg

def get_block_multi(req):

    blocks = list()
    for req_msg in req.coords:
        msg = map_block_msg()
        msg.x = req_msg.x
        msg.y = req_msg.y
        msg.z = req_msg.z
        msg.blockid, msg.metadata = world.get_block(msg.x, msg.y, msg.z)
        #print msg
        blocks.append(msg)

    #print type(blocks)
    return {'blocks': blocks}


world = MinecraftMap(DIMENSION_OVERWORLD)

if __name__ == "__main__":

    rospy.init_node('minecraft_map_server')

    rospy.Subscriber('chunk_data', chunk_data_msg, world.handle_unpack_chunk)
    rospy.Subscriber('chunk_bulk', chunk_bulk_msg, world.handle_unpack_bulk)
    rospy.Subscriber('block_data', block_data_msg, world.handle_unpack_block)

    srv_block = rospy.Service('get_block_data', get_block_srv, get_block)
    srv_block_multi = rospy.Service('get_block_multi', get_block_multi_srv, get_block_multi)
    #srv_light = rospy.Service('get_light_data', light_data_msg, world.getLight)
    #srv_biome = rospy.Service('get_biome_data', biome_data_msg, world.getBiome)

    rospy.spin()















