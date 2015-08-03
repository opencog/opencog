#!/usr/bin/env python

"""
Created by Bradley Sheneman
action server. receives requests for simple actions and returns success or failure
another node should be responsible for building plans, and send the deconstructed plan
to this server one action at a time

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
#from minecraft_bot.srv import visible_blocks_srv, get_block_multi_srv
#from minecraft_bot.msg import map_block_msg, vec3_msg

from minecraft_bot.msg import position_msg, movement_msg
from minecraft_bot.srv import look_srv, rel_move_srv
import visibility as vis
import mcphysics as phy

#from spock.utils import pl_announce, BoundBuffer
#from spock.mcmap import smpmap, mapdata
#from spock.mcp import mcdata

class ClientMover():

    def __init__(self):

        self.x = None
        self.y = None
        self.z = None
        self.pitch = None
        self.yaw = None
        self.on_ground = True
        
        rospy.Subscriber('client_position_data', position_msg, self.handlePosUpdate)
        self.pub_move = rospy.Publisher('movement_data', movement_msg, queue_size = 10)


    def handleLogin(self, data):

        pass


    def handleJoin(self, data):

        pass


    def handleSpawn(self, data):

        pass


    def handleHealth(self, data):

        pass


    def handlePosUpdate(self, data):
        
        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.pitch = data.pitch
        self.yaw = data.yaw
        print "current pos: x %f, y %f, z%f, pitch %f, yaw %f "%(self.x, self.y, self.z, self.pitch, self.yaw)


    # 95% interval of actual desired value
    def inRange(self, first, second):
        
        print "first %f, second %f\n"%(first, second)

        if (first >= second - abs(second)*0.05) and (first <= second + abs(second)*0.05):
            print "in range"
            return True
        else:
            print "not in range"
            return False


    def posToDict(self):

        return {
                'x':self.x,
                'y':self.y,
                'z':self.z,
                'pitch':self.pitch,
                'yaw':self.yaw,
                'on_ground':self.on_ground
                }
    
    def handleLook(self, pitch, yaw):
        
        # as long as we have not reached our target, update position and calculate new frame
        
        #yaw = req.yaw
        #pitch = req.pitch

        timer = 0.
        while timer < 3:
            c_pitch = client_pos.pitch
            c_yaw = client_pos.yaw
            
            print "current pose: pitch %f, yaw %f\n"%(c_pitch, c_yaw)
            print "requested dir: pitch %f, yaw %f\n"%(pitch, yaw)
            
            if self.inRange(c_yaw, yaw) and self.inRange(c_pitch, pitch):
                return True
           
            print "desired: pitch %f, yaw %f\n"%(pitch, yaw)

            frames = phy.getLookFrames(self.posToDict(), pitch, yaw)
            
            for frame in frames:
                msg = movement_msg()
                msg.x = frame['x']
                msg.y = frame['y']
                msg.z = frame['z']
                msg.pitch = frame['pitch']
                msg.yaw = frame['yaw']

                self.pub_move.publish(msg)
            
            timer += 0.5
            rospy.sleep(0.5)

        # if we have not reached correct position in 3 seconds
        return False


      


    def handleRelativeLook(self, pitch, yaw):
        
        # as long as we have not reached our target, update position and calculate new frame
        
        #yaw = req.yaw
        #pitch = req.pitch
        desired_pitch = client_pos.pitch - pitch
        desired_yaw = client_pos.yaw + yaw
 
        timer = 0.
        while timer < 3:
            c_pitch = client_pos.pitch
            c_yaw = client_pos.yaw
            
            print "current pose: pitch %f, yaw %f\n"%(c_pitch, c_yaw)
            print "requested dir: pitch %f, yaw %f\n"%(pitch, yaw)
            if self.inRange(c_yaw, desired_yaw) and self.inRange(c_pitch, desired_pitch):
                return True

           
            print "desired: pitch %f, yaw %f\n"%(desired_pitch, desired_yaw)

            if desired_pitch < -90:
                desired_pitch = -90
            elif desired_pitch > 90:
                desired_pitch = 90

            if desired_yaw > 180:
                while desired_yaw > 180:
                    desired_yaw -= 360
            elif desired_yaw < -180:
                while desired_yaw < -180:
                    desired_yaw += 360
            
            frames = phy.getLookFrames(self.posToDict(), desired_pitch, desired_yaw)
            
            for frame in frames:
                msg = movement_msg()
                msg.x = frame['x']
                msg.y = frame['y']
                msg.z = frame['z']
                msg.pitch = frame['pitch']
                msg.yaw = frame['yaw']

                self.pub_move.publish(msg)
            
            timer += 0.5
            rospy.sleep(0.5)

        # if we have not reached correct position in 3 seconds
        return False

    # jump is either True or False
    def handleRelativeMove(self, yaw, dist, jump):
        
        abs_yaw = client_pos.yaw
        abs_x = client_pos.x
        abs_y = client_pos.y
        abs_z = client_pos.z

        desired_yaw = abs_yaw + yaw

        if desired_yaw > 180:
            while desired_yaw > 180:
                desired_yaw -= 360
        elif desired_yaw < -180:
            while desired_yaw < -180:
                desired_yaw += 360
        
        # calculate change in x, y, z desired using pitch of zero (no movement in y dir)
        # this will work the same for jumps, but dist will be small
        dx, dy, dz = vis.calcRayStep(0, desired_yaw, dist)
       
        desired_x = abs_x + dx
        desired_y = abs_x + dy
        desired_z = abs_x + dz

        msg = movement_msg()
        msg.x = desired_x
        msg.y = desired_y
        msg.z = desired_z
        msg.pitch = 0
        msg.yaw = desired_yaw
        msg.jump = jump
        msg.speed = 1

        print msg
        pub_move.publish(msg)

def handleAbsoluteLook(req):

    result = client_pos.handleLook(req.pitch, req.yaw)
    return result


def handleAbsoluteMove(req):

    result = client_pos.handleMove(req.x, req.y, req.z, req.jump)
    return result



def handleRelativeLook(req):

    result = client_pos.handleRelativeLook(req.pitch, req.yaw)
    return result


def handleRelativeMove(req):

    result = client_pos.handleRelativeMove(req.yaw, req.dist, req.jump)
    return result


client_pos = ClientMover()

def actionServer():


    #clientpos = ClientMover()


    #vis.initBlockMats()
    
    #client = ClientMover()

    rospy.init_node('action_server')
    
    # look handles changes in head position
    # move handles changes in body location and jump commands
    rel_look_srvc = rospy.Service('set_relative_look', look_srv, handleRelativeLook)
    rel_move_srvc = rospy.Service('set_relative_move', rel_move_srv, handleRelativeMove)
    
    abs_look_srvc = rospy.Service('set_look', look_srv, handleAbsoluteLook)
    abs_move_srvc = rospy.Service('set_move', rel_move_srv, handleAbsoluteMove)


    print("action server initialized")
    
    rospy.spin()
if __name__ == "__main__":

    #print "current position"
    #print (clientpos.x, clientpos.y, clientpos.z)

    actionServer()
