"""

created by Bradley Sheneman

composes actions and sends them to SendAction.py

Hybrid Spock plugin and ROS node that receives movement commands from ROS and pieces together
a frame-by-frame action. Has knowledge of what high level actions like "jump" and "sprint"
are composed of at the low level



"""
from spock.utils import pl_announce
from spock.mcp import mcdata
from spock.utils import Vec3
# from spockextras.plugins.actionutils import Vec5

import math


import logging
logger = logging.getLogger('spock')


# receives movement commands from ROS. currently does not have error checking, but will eventually
# return value of: success, failure, in-progress to the planner node

class NewMovementCore:
    def __init__(self):

        self.target = None
    
    
    def setTarget(self, data):

        # jump might cause weird behavior. bot will continue to 'jump' until it reaches its target
        # it is supposed to be used as a separate action from moving, but the x, y, z are still there
        # to allow movement while jumping. requested distance should be no more than it takes to
        # perform a single jump.
        
        self.jump = bool(data.jump)
        self.speed = data.speed
        
        x = float(data.x)
        y = float(data.y)
        z = float(data.z)
        
        self.target = Vec3(x=x, y=y, z=z)
                
        # print ("target pos: %2f, %2f, %2f") %(
        #        self.target.x,
        #        self.target.y,
        #        self.target.z)



@pl_announce('NewMovement')
class NewMovementPlugin:
    
    def __init__(self, ploader, settings):
        
        self.net = ploader.requires('Net')
        self.clinfo = ploader.requires('ClientInfo')
        self.phys = ploader.requires('NewPhysics')
        
        self.mvc = NewMovementCore()
        
        ploader.reg_event_handler('ros_moveto', self.rosMoveTo)
        ploader.reg_event_handler('client_tick', self.clientTick)
        ploader.reg_event_handler('action_tick', self.actionTick)
        ploader.reg_event_handler('cl_position_update', self.handlePosUpdate)
        
        ploader.provides('NewMovement', self.mvc)    
        # self.startMovementNode()
    
    
#     def startMovementNode(self):
#    
#        rospy.init_node('movement_listener')
#        print("movement listener node initialized")
#
#        rospy.Subscriber('movement_cmd', movement_msg, self.cac.setTarget)

    def rosMoveTo(self, name, data):

        self.mvc.setTarget(data)


    def clientTick(self, name, data):
        
        self.net.push_packet('PLAY>Player Position', self.clinfo.position.get_dict())
    
    
    def handlePosUpdate(self, name, data):
        
        self.net.push_packet('PLAY>Player Position and Look', data.get_dict())
 
    def getAngle(self, x, z):

        # from positive x axis (east)
        # where z is south
        dx = x - self.clinfo.position.x
        dz = z - self.clinfo.position.z
        
        if dx == 0:
            angle = math.copysign(180, dz)
        else:
            angle = math.degrees(math.atan(dz/dx))
            if dx < 0:
                angle += 180
        
        return angle


    def actionTick(self, name, data):
        
        self.doMovement()


    def doMovement(self):
        
        # as long as we have not reached our target, update position and calculate new frame
        # also push the frame to 'actions' queue to make available for the action sender
        
        """
        if (     self.cac.target.x != math.floor(self.clinfo.position.x)
                and self.cac.target.y != math.floor(self.clinfo.position.y)
                and self.cac.target.z != math.floor(self.clinfo.position.z)):
	"""
        if (self.mvc.target != None):
            direction = self.getAngle(self.mvc.target.x, self.mvc.target.z)
            print ("current pos: " + str(self.clinfo.position))
            self.phys.move(direction, self.mvc.speed, self.mvc.jump)
