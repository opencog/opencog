"""
calculating what blocks player can see.
For now it's all testing, so we don't really calculate bot's vision. just return all the blocks(20*20*20)surrounding bot's position.
"""

from copy import copy
from math import sin,cos,pi,radians,floor
import numpy as np
from spock.utils import pl_announce,Vec3
from spock.mcp import mcdata

"""

#The default settings in Minecraft game
DefaultRadius=16
DefaultFOV=radians(70.0)

#Some helper functions and class
def getvisiblerange(fov,yaw,pitch):
    return {'yaw' : np.linspace(yaw-fov/2,yaw+fov/2,num=32), 'pitch': np.linspace(yaw-fov/2,pitch+fov/2,num=32)}

def polartoCartesian(r,yaw,pitch):
    return Vec3(-cos(pitch)*sin(yaw)*r,-sin(pitch)*r,cos(pitch)*cos(yaw)*r)

"""

PlayerHeight=1.62

class Block:
    
    def __init__(self,x,y,z,btype,meta):
        self.x=x
        self.y=y
        self.z=z
        self.btype=btype
        self.meta=meta



class VisibilityCore:
    def __init__(self,world,clinfo,settings):
        self.world=world
        self.clinfo=clinfo
        #self.radius = settings.get('radius',DefaultRadius)
        #self.fov = settings.get('fov',DefaultFOV)
        self.visibleblocks=[]
    
    def update(self):

        """
        To update the visibility, calculate the visible range of player,
        slice the range to get all the direction player can look along,
        then for each direction, starting from playerpos, step along the direction,
        if it meets no block, keep stepping;
        if it meets a block, that block is visible,stop stepping and update the block if the block has been in the visibleblock list.

        So far we're still testing the pipeline, so we comment out the algorithm, just returning the 20*20*20 blocks surrounding the player.
        

        playerpos=Vec3(self.clinfo.position.x,self.clinfo.position.y+PlayerHeight,self.clinfo.position.z)
        visiblerange=getvisiblerange(self.fov,radians(self.clinfo.position.yaw),radians(self.clinfo.position.pitch))
        self.visibleblocks=[]

        for yaw in visiblerange['yaw']:
            for pitch in visiblerange['pitch']:
                
                pos=copy(playerpos)
                step=0.2
                rayvec=polartoCartesian(step,yaw,pitch)
                blockdata=(0,0)
                distance=0

                while distance < self.radius:
                    
                    pos.add_vector(vec=rayvec)
                    distance+=step
                    blockdata=self.world.get_block(floor(pos.x),floor(pos.y),floor(pos.z))
                    if blockdata!=(0,0):
                        block=Block(floor(pos.x),floor(pos.y),floor(pos.z),blockdata[0],blockdata[1])
                        if block not in self.visibleblocks:
                            self.visibleblocks.append(block)
                        break                    
        """
        
        playerpos=Vec3(int(floor(self.clinfo.position.x)),int(floor(self.clinfo.position.y+PlayerHeight)),int(floor(self.clinfo.position.z)))
        self.visibleblocks=[]

        for x in range(playerpos.x-10,playerpos.x+10):
            for y in range(playerpos.y-10,playerpos.y+10):
                for z in range(playerpos.z-10,playerpos.z+10):
                    blockdata=self.world.get_block(x,y,z)
                    if blockdata!=(0,0):
                        block=Block(x,y,z,blockdata[0],blockdata[1])
                        self.visibleblocks.append(block)

    def getBlocks(self):
        return self.visibleblocks
    

@pl_announce('Visibility')
class VisibilityPlugin:


    def __init__(self,ploader,settings):
        
        self.event = ploader.requires('Event')
        self.world = ploader.requires('World')
        self.clinfo = ploader.requires('ClientInfo')
        self.timers = ploader.requires('Timers')
        self.tickrate=0.2
        self.visibility = VisibilityCore(self.world,self.clinfo,settings)
        ploader.reg_event_handler('PLAY_STATE', self.startUpdate)
        ploader.provides('Visibility',self.visibility)

    def startUpdate(self,packet,data):
        self.timers.reg_event_timer(self.tickrate,self.updateVisibleBlocks)

    def updateVisibleBlocks(self):    
        self.visibility.update()
        data=self.visibility.getBlocks()
        self.event.emit('ros_send_visible_blocks',data)


                
        


