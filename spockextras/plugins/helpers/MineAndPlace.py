"""
created by Bradley Sheneman

simple plugin to provide block mine and place capability

"""
from spock.mcp import mcdata
from spock.utils import pl_announce

import logging
logger = logging.getLogger('spock')


class MineAndPlaceCore:

    def __init__(self):

        self.target = None



@pl_announce('MineAndPlace')
class MineAndPlacePlugin:
    
    def __init__(self, ploader, settings):
        
        self.net = ploader.requires('Net')
        
        ploader.reg_event_handler('ros_placeblock', self.handle_place)
        ploader.reg_event_handler('ros_mineblock', self.handle_break)
        
        self.mpc = MineAndPlaceCore()
        ploader.provides('MineAndPlace', self.mpc)
        
        #print("mine and place plugin was loaded...")
        #self.startMineAndPlaceNode()
    
   
    def handle_place(self, event, data):

        # just send a data block with more things, obtained from ROS message in main node
        block_data = {
                'location':     {'x': int(data.loc_x),'y': int(data.loc_y),'z': int(data.loc_z)},
                'direction':    int(data.dir),
                'held_item':    {'id': int(data.id)},
                'cur_pos_x':    int(data.pos_x),
                'cur_pos_y':    int(data.pos_y),
                'cur_pos_z':    int(data.pos_z)}
 
        #block_data = {
        #        'location':     {'x': int(args[0]),'y': int(args[1]),'z': int(args[2])},
        #        'direction':    1,
        #        'held_item':    {'id': -1},
        #        'cur_pos_x':    8,
        #        'cur_pos_y':    16,
        #        'cur_pos_z':    8}
        
        self.net.push_packet('PLAY>Player Block Placement', block_data)


    def handle_break(self, event, data):
        
        #args = data['args']
        print("received command ros_mineblock")
        block_data = {
                'location':     {'x': int(data.x),'y': int(data.y),'z': int(data.z)},
                'status':       int(data.status),
                'face':         int(data.face)}
        
        self.net.push_packet('PLAY>Player Digging', block_data)
        
        # update status to 2 (finished) and push final packet
        block_data['status'] = 2
        
        self.net.push_packet('PLAY>Player Digging', block_data)
