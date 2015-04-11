"""
calculating what blocks player can see.
"""

from copy import copy
from math import sin,cos,pi,radians,floor
import numpy as np
from spock.utils import pl_announce,Vec3
from spock.mcp import mcdata

#The default settings in Minecraft game
DefaultRadius=16
DefaultFOV=radians(70.0)
PlayerHeight=1.62

#Some helper functions and class
def getvisiblerange(fov,yaw,pitch):
    return {'yaw' : np.linspace(yaw-fov/2,yaw+fov/2,num=32), 'pitch': np.linspace(yaw-fov/2,pitch+fov/2,num=32)}

def polartoCartesian(r,yaw,pitch):
    return Vec3(-cos(pitch)*sin(yaw)*r,-sin(pitch)*r,cos(pitch)*cos(yaw)*r)

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
        self.radius = settings.get('radius',DefaultRadius)
        self.fov = settings.get('fov',DefaultFOV)
        self.visibleblocks=[]
    
    def update(self):

        """
        To update the visibility, calculate the visible range of player,
        slice the range to get all the direction player can look along,
        then for each direction, starting from playerpos, step along the direction,
        if it meets no block, keep stepping;
        if it meets a block, that block is visible,stop stepping and update the block if the block has been in the visibleblock list.
        """

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

    def get(self):
        return self.visibleblocks
    

@pl_announce('Visibility')
class VisibilityPlugin:

    tickrate=0.1
    def __init__(self,ploader,settings):

        self.event = ploader.requires('Event')
        self.world = ploader.requires('World')
        self.clinfo = ploader.requires('ClientInfo')
        self.timers = ploader.requires('Timers')
        self.visibility = VisibilityCore(self.world,self.clinfo,settings)
        ploader.reg_event_handler('PLAY_STATE', self.start_update)
        ploader.provides('Visibility',self.visibility)

    def start_update(self,packet,data):
        self.timers.reg_event_timer(self.tickrate,self.update_visibility)

    def update_visibility(self):    
        self.visibility.update()


                
        


