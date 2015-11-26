"""
created by Bradley Sheneman

small utility plugin to provide timestamp information
and ability to set message attributes from dictionary (with timestamp)

"""

import rospy

from spockbot.plugins.base import pl_announce
#from spockbot import mcdata
from spockbot.mcp import proto

import logging
logger = logging.getLogger('spock')

class MessengerCore:
    
    def __init__(self):
        
        self.age = 0
        self.time_of_day = 0

    def setMessage(self, msg, data):
    
        for key in data:
            if hasattr(msg, key):
                if isinstance(data[key], dict):
                    pass
                elif isinstance(data[key], list):
                    pass
                else:
                    setattr(msg, key, data[key])
            
        if hasattr(msg, 'MCtimestamp'):
            msg.MCtimestamp = self.age
                
        if hasattr(msg, 'ROStimestamp'):
            rostime = rospy.Time.now()
            msg.ROStimestamp = rostime.secs*10e9 + rostime.nsecs


    def updateTime(self, data):
        
        self.age = data['world_age']
        self.time_of_day = data['time_of_day']


    def reset(self):

        self.__init__()



@pl_announce('Messenger')
class MessengerPlugin:

    def __init__(self, ploader, settings):
    
        self.core = MessengerCore()
        ploader.provides('Messenger', self.core)

        ploader.reg_event_handler((proto.PLAY_STATE, proto.SERVER_TO_CLIENT, 0x03), self.handleTimeUpdate)
        ploader.reg_event_handler('disconnect', self.handleDisconnect)
        

    def handleTimeUpdate(self, name, packet):

        self.core.updateTime(packet.data)


    def handleDisconnect(self, name, data):

        self.core.reset()



