"""

created by Bradley Sheneman

composes actions and sends them to SendAction.py

Hybrid Spock plugin and ROS node that receives movement commands from ROS and pieces together
a frame-by-frame action.



"""
import math
import Queue
import logging
logger = logging.getLogger('spock')

from spockbot.plugins.base import pl_announce
from spockbot import mcdata

# receives movement commands from ROS and sends update of current
# position of client every 'client_tick'
class NewMovementCore:
    
    # holds a queue of action commands. when a position update
    # is sent by the minecraft server, we do not add to queue
    def __init__(self):

        self.targets = Queue.Queue()
        

    # 'add a unit motion to queue'
    def addTargetMotion(self, data):
        
        target = {}

        target['jump'] = bool(data.jump)
        target['pitch'] = data.pitch
        target['yaw'] = data.yaw
        
        target['x'] = float(data.x)
        target['y'] = float(data.y)
        target['z'] = float(data.z)
        
        self.targets.put(target)


    def resetMotion(self):
        
        self.targets.clear()

               

@pl_announce('NewMovement')
class NewMovementPlugin:
    
    def __init__(self, ploader, settings):
        
        self.net = ploader.requires('Net')
        self.clinfo = ploader.requires('ClientInfo')
        self.event = ploader.requires('Event')
        
        self.core = NewMovementCore()
        
        ploader.reg_event_handler('ros_moveto', self.handleRosRequest)
        ploader.reg_event_handler('client_tick', self.handleClientTick)
        ploader.reg_event_handler('action_tick', self.handleActionTick)
        ploader.reg_event_handler('cl_position_update', self.handlePositionUpdate)
        
        ploader.provides('NewMovement', self.core)    
    
    
    def handleRosRequest(self, name, data):
        
        #print "receiving request for movement"
       self.core.addTargetMotion(data)


    # on every client tick:
    # 1) sends packet to server
    # 2) sends update to ROS
    def handleClientTick(self, name, data):
        
        data_dict = self.clinfo.position.get_dict()
        self.net.push_packet('PLAY>Player Position and Look', data_dict)
        self.event.emit('ros_position_update', data_dict)
        #print "client tick"

 
    # called whenever the Minecraft server sends a position update
    # or when a position change is requested by the external client
    # currently the client info plugin in Spock handles updating internal state...
    # it would be better if all changes to state happen in one place.
    def handlePositionUpdate(self, name, data):
        
        #print "position update"
        self.net.push_packet('PLAY>Player Position and Look', data.get_dict())
   
    
    # on every action tick:
    # 1) attempt next movement in queue
    def handleActionTick(self, name, data):
        
        #print "action tick"
        self.doMovement()


    def doMovement(self):
        
        try:
            #print "trying to get queue item"
            pos = self.core.targets.get(False)
           
        except Queue.Empty:
            pass
            #print "no requested movements"
        else:
            #print "Moving to x:%f, y:%f, z:%f, pitch:%f, yaw:%f\n"%(
            #        pos['x'],
            #        pos['y'],
            #        pos['z'],
            #        pos['pitch'],
            #        pos['yaw'])
	    
            p = self.clinfo.position
	    p.x = pos['x']
	    p.y = pos['y']
	    p.z = pos['z']
	    p.yaw = pos['yaw']
	    p.pitch = pos['pitch']

	    self.event.emit('cl_position_update', self.clinfo.position)
