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

import array
import struct
from spock import utils

DIMENSION_NETHER   = -0x01
DIMENSION_OVERWOLD =  0x00
DIMENSION_END      =  0x01



# a bit of a misnomer... a chunk in spock is actually 1/16th of a minecraft chunk
# a minecraft chunk is a 'chunk column' in spock, or 16 spock chunks arranged vertically

class ChunkData:
    length = 16*16*16
    ty = 'B'
    data = None

    def fill(self):
        
        if not self.data:
            self.data = array.array(self.ty, [0]*self.length)
    

    def unpack(self, buff):
        
        self.data = array.array(self.ty, buff.read(self.length))
    
    
    def pack(self):
        
        self.fill()
        return self.data.tobytes()

    
    def get(self, x, y, z):
        
        self.fill()
        return self.data[x + ((y * 16) + z) * 16]

    
    def set(self, x, y, z, data):
        
        self.fill()
        self.data[x + ((y * 16) + z) * 16] = data



class BiomeData(ChunkData):
    
    """ A 16x16 array stored in each ChunkColumn. """
    length = 16*16
    data = None
    
    def get(self, x, z):
        self.fill()
        return self.data[x + z * 16]

    def set(self, x, z, d):
        self.fill()
        self.data[x + z * 16] = d

class ChunkDataShort(ChunkData):
    """ A 16x16x16 array for storing block IDs/Metadata. """
    length = 16*16*16*2
    ty = 'H'

class ChunkDataNibble(ChunkData):
    """ A 16x16x8 array for storing metadata, light or add. Each array element
    contains two 4-bit elements. """
    length = 16*16*8

    def get(self, x, y, z):
        self.fill()
        x, r = divmod(x, 2)
        i = x + ((y * 16) + z) * 16
        if r:
            return self.data[i] & 0x0F
        else:
            return self.data[i] >> 4

    def set(self, x, y, z, data):
        self.fill()
        x, r = divmod(x, 2)
        i = x + ((y * 16) + z) * 16
        if r:
            self.data[i] = (self.data[i] & 0xF0) | (data & 0x0F)
        else:
            self.data[i] = (self.data[i] & 0x0F) | ((data & 0x0F) << 4)

class Chunk:
    def __init__(self):
        self.block_data = ChunkDataShort()
        self.light_block = ChunkDataNibble()
        self.light_sky = ChunkDataNibble()


class ChunkColumn:
    
    def __init__(self):
        
        self.chunks = [None]*16
        self.biome = BiomeData()
    
    
    def unpack(self, buff, mask, skylight=True, continuous=True):
        
        #In the protocol, each section is packed sequentially (i.e. attributes
        #pertaining to the same chunk are *not* grouped)
        self.unpack_block_data(buff, mask)
        self.unpack_light_block(buff, mask)
        
        if skylight:
            self.unpack_light_sky(buff, mask)
        
        if continuous:
            self.biome.unpack(buff)

    
    def unpack_block_data(self, buff, mask):
        
        for i in range(16):
            if mask&(1<<i):
                if self.chunks[i] == None:
                    self.chunks[i] = Chunk()
                
                self.chunks[i].block_data.unpack(buff)

    
    def unpack_light_block(self, buff, mask):
        
        for i in range(16):
            if mask&(1<<i):
                if self.chunks[i] == None:
                    self.chunks[i] = Chunk()
                self.chunks[i].light_block.unpack(buff)

    
    def unpack_light_sky(self, buff, mask):
        
        for i in range(16):
            if mask&(1<<i):
                if self.chunks[i] == None:
                    self.chunks[i] = Chunk()
                self.chunks[i].light_sky.unpack(buff)
    
    
    # modified to include light and sky data, since there
    # doesn't seem to be a function to retrieve it already
    def get_block(self, x, y, z):
        
        x, y, z = int(x), int(y), int(z)
        x, rx = divmod(x, 16)
        y, ry = divmod(y, 16)
        z, rz = divmod(z, 16)
        
        chunk = self.chunks[y]
        
        if chunk == None:
            #print (0, 0)
            return 0, 0, -1, -1
        
        data = chunk.block_data.get(rx,ry,rz)
        
        # light is -1 if no light data. 0 implies lowest light level (total darkness)
        light =-1
        sky =-1
        """
        if bl:
            light = chunk.light_block.get(rx, ry, rz)
        else:
            light = -1

        if sl:
            sky = chunk.light_sky.get(rx, ry, rz)
        else:
            sky = -1
        """

        #print (light, sky)
        return data>>4, data&0x0F, light, sky



def unwrapChunk(data, column):
    
    for x in range(16):
        for z in range(16):
            for y in range(256):

                sendBlockData(data, column, x, y, z)
                

         
def sendBlockData(data, column, x, y, z):
    
    chunk_x = data.chunk_x
    chunk_z = data.chunk_z
    
    msg = map_block_msg()
    msg.x = x + chunk_x
    msg.y = y
    msg.z = z + chunk_z

    block_id, block_meta, light, sky = column.get_block(msg.x, msg.y, msg.z)
    if (msg.x,msg.y,msg.z) == (-39,13,-45):
        print "in send block data:it's gold right? %s"%(block_id)
    if block_id==14:
        print 'in send block data:it is gold, pos %s %s %s'%(msg.x,msg.y,msg.z)
    msg.blockid = block_id
    msg.meta = block_meta
    msg.blocklight = light
    msg.skylight = sky
    rostime = rospy.get_rostime()
    msg.timestamp = rostime.secs*(10e9)+rostime.nsecs
    
    # in other words, if not air...
    if block_id != 0:
        blockpub.publish(msg)
#        print msg



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
    msg.x = data.x
    msg.y = data.y
    msg.z = data.z
    if (msg.x,msg.y,msg.z) == (-39,13,-45):
        print "in single block change:it's gold right? %s"%(data.block_id)
    if block_id==14:
        print 'in single block change: it is gold, pos %s %s %s'%(msg.x,msg.y,msg.z)

    # need to change this. find a way to get light, sky, and meta
    # data from the original block change message (if possible)
    # otherwise, will have to look at light information from previous
    # block at specified location
    msg.blocklight = 0
    msg.skylight = 0

    msg.blockid = data.blockid 
    rostime = rospy.get_rostime()
    msg.timestamp = rostime.secs*(10e9)+rostime.nsecs
    # in other words, if not air...
    if msg.blockid != 0:
        blockpub.publish(msg)

        """

        def set_block(self, x, y, z, block_id = None, meta = None, data = None):
		x, rx = divmod(x, 16)
		y, ry = divmod(y, 16)
		z, rz = divmod(z, 16)

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

		if data == None:
			data = (block_id<<4)|(meta&0x0F)
		chunk.block_data.set(rx, ry, rz, data)


	def set_light(self, x, y, z, light_block = None, light_sky = None):
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
	
        def set_biome(self, x, z, data):
		x, rx = divmod(x, 16)
		z, rz = divmod(z, 16)

		if (x,z) in self.columns:
			column = self.columns[(x,z)]
		else:
			column = ChunkColumn()
			self.columns[(x,z)] = column

		return column.biome.set(rx, rz, data)
        """

if __name__ == "__main__":
    rospy.init_node('map_data_node')

    rospy.Subscriber('chunk_data', chunk_data_msg, handleChunkData)
    rospy.Subscriber('chunk_bulk', chunk_bulk_msg, handleChunkBulk)
    rospy.Subscriber('block_data', block_data_msg, handleBlockData)


    blockpub = rospy.Publisher('block', map_block_msg, queue_size = 100000)    
    rospy.spin()

