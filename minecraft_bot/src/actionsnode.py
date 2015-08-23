#!/usr/bin/env python

"""
Created by Bradley Sheneman
action server. receives requests for simple actions and returns success or failure
another node should be responsible for building plans, and send the deconstructed plan
to this server one action at a time

"""

from math import pi, acos, asin, sqrt

import roslib; roslib.load_manifest('minecraft_bot')
import rospy

from minecraft_bot.msg import position_msg, movement_msg
from minecraft_bot.srv import look_srv, rel_move_srv, abs_move_srv

import mc_vis_utils as vis
import mc_physics_utils as phy


class ClientMover():

    def __init__(self):

        self.x = None
        self.y = None
        self.z = None
        self.pitch = None
        self.yaw = None
        self.on_ground = True
        
        rospy.Subscriber('client_position_data', position_msg, self.handle_pos_update)

        self.pub_move = rospy.Publisher('movement_data', movement_msg, queue_size = 10)
        self.pub_pos = rospy.Publisher('camera_position_data', position_msg, queue_size = 100)


    def camera_tick(self):

        msg = position_msg()
        pos = self.pos_to_dict()
        
        msg.x = pos['x']
        msg.y = pos['y']
        msg.z = pos['z']
        msg.pitch = pos['pitch']
        msg.yaw = pos['yaw']
        
        self.pub_pos.publish(msg)


    def handle_pos_update(self, data):
        
        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.pitch = data.pitch
        self.yaw = data.yaw


    def in_range(self, first, second):
        
        if (first >= second - abs(second)*0.05) and (first <= second + abs(second)*0.05):
            return True
        else:
            return False


    def pos_to_dict(self):

        return {
                'x':self.x,
                'y':self.y,
                'z':self.z,
                'pitch':self.pitch,
                'yaw':self.yaw,
                'on_ground':self.on_ground
                }


    def get_desired_yaw(self, x_0, z_0, x_1, z_1):
        
        l = x_1 - x_0
        w = z_1 - z_0
        c = sqrt( l*l + w*w )
        alpha1 = -asin(l/c)/pi * 180
        alpha2 =  acos(w/c)/pi * 180
        if alpha2 > 90:
            return 180 - alpha1
        else:
            return alpha1


    def handle_look(self, pitch, yaw):
        
        timer = 0.
        while timer < 3:
            if self.inRange(self.yaw, yaw) and self.inRange(self.pitch, pitch):
                return True
            
            frames = phy.get_look_frames(self.pos_to_dict(), pitch, yaw)

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


    def handle_relative_look(self, pitch, yaw):
        
        pos = self.pos_to_dict()
        desired_pitch = pos['pitch'] - pitch
        desired_yaw = pos['yaw'] + yaw
 
        timer = 0.
        while timer < 3:
            if self.inRange(self.yaw, desired_yaw) and self.inRange(self.pitch, desired_pitch):
                return True

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
            
            frames = phy.get_look_frames(pos, desired_pitch, desired_yaw)
            
            for frame in frames:
                msg = movement_msg()
                msg.x = frame['x']
                msg.y = frame['y']
                msg.z = frame['z']
                msg.pitch = frame['pitch']
                msg.yaw = frame['yaw']

                print msg
                self.pub_move.publish(msg)
            
            timer += 0.5
            rospy.sleep(0.5)

        # if we have not reached correct position in 3 seconds
        return False

    def handle_move(self, x, z, jump):

        speed = 1
        pos = self.pos_to_dict()
        direction = self.get_desired_yaw(pos['x'], pos['z'], x, z)
        dist = sqrt( (x - pos['x'])**2 + (z - pos['z'])**2 )
        frames = phy.get_movement_frames(pos, direction, dist, speed, jump)
        
        for frame in frames:
            msg = movement_msg()
            msg.x = frame['x']
            msg.y = frame['y']
            msg.z = frame['z']
            msg.pitch = frame['pitch']
            msg.yaw = frame['yaw']
            msg.jump = jump
            print "abs_move_msg", msg
            self.pub_move.publish(msg)
        return True


    def handle_relative_move(self, direction, dist, jump):
        
        speed = 1
        pos = self.pos_to_dict()       
        desired_yaw = direction

        if desired_yaw > 180:
            while desired_yaw > 180:
                desired_yaw -= 360
        elif desired_yaw < -180:
            while desired_yaw < -180:
                desired_yaw += 360
        
        frames = phy.get_movement_frames(pos, direction, dist, speed, jump)
        
        #print "dx,dy,dz", dx, dy, dz
        for frame in frames:
            msg = movement_msg()
            msg.x = frame['x']
            msg.y = frame['y']
            msg.z = frame['z']
            msg.pitch = frame['pitch']
            msg.yaw = frame['yaw']
            msg.jump = jump
            print "rel_move_msg", msg
            self.pub_move.publish(msg)
        return True


def handle_absolute_look(req):

    print 'handleabslook'
    result = client_pos.handle_look(req.pitch, req.yaw)
    return result


def handle_absolute_move(req):
    print 'handleabsmove'
    result = client_pos.handle_move(req.x, req.z, req.jump)
    print 'absmove result'
    return result



def handle_relative_look(req):

    result = client_pos.handle_relative_look(req.pitch, req.yaw)
    return result


def handle_relative_move(req):
    result = client_pos.handle_relative_move(req.yaw, req.dist, req.jump)
    return result


def action_server():

    rospy.init_node('action_server')
    
    # make sure we have received at least one position update
    while client_pos.x == None:
        rospy.sleep(1.)

    rel_look_srvc = rospy.Service('set_relative_look', look_srv, handle_relative_look)
    rel_move_srvc = rospy.Service('set_relative_move', rel_move_srv, handle_relative_move)
    
    abs_look_srvc = rospy.Service('set_look', look_srv, handle_absolute_look)
    abs_move_srvc = rospy.Service('set_move', abs_move_srv, handle_absolute_move)


    print("action server initialized")
    
    # continue to send visible blocks until shutdown
    while not rospy.is_shutdown():
        client_pos.camera_tick()
        rospy.sleep(0.1)



client_pos = ClientMover()

if __name__ == "__main__":
    
    action_server()
