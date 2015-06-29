"""

    Plugin to test movement and pathfinding ability.
    Causes bot to detect and run away from the player. Will eventually be used for hostile mobs
    For now it is purely reactive, and responds to player within a predefined radius.
    Will always run in the opposite direction.

"""

"""

    common hostile mobs:

    entity      dec     hex

    creeper     50      32
    zombie      54      36
    skeleton    51      33
    witch       66      42
    spider      52      34

"""
import math


"""
# on a small map these can be used to detect hostile mobs. Will need some sort of priority list
# of mobs that the bot should currently be focused on, since all spawned mobs are currently detected

hostiles = {50: 'creeper',
            51: 'skeleton',
            52: 'spider',
            54: 'zombie',
            66: 'witch'}
"""

from spock.utils import pl_announce, Info

class RunAwayCore:
    
    tick_rate = 0.2
    
    def __init__(self):
        self.other_players = {}

@pl_announce('RunAway')
class RunAwayPlugin:

    def __init__(self, ploader, settings):
        
        self.rac = RunAwayCore()
        self.entities = ploader.requires('Entities')
        self.movement = ploader.requires('Movement')
        self.timers = ploader.requires('Timers')
        clinfo = ploader.requires('ClientInfo')
        self.pos = clinfo.position
        
        ploader.reg_event_handler('PLAY<Spawn Player', self.handle_spawn_player)
        self.timers.reg_event_timer(self.rac.tick_rate, self.handle_check_player_nearby)
        
        
    def handle_spawn_player(self, name, data):
        
        for playerID in self.entities.players:
            print ("I see %2f with ID: %2f") %(
                self.entities.players[playerID].uuid, playerID)
            
            self.rac.other_players[playerID] = self.entities.players[playerID]


    def handle_check_player_nearby(self):
    
        print("checking for other players")
        for playerID in self.rac.other_players:
            dx = self.pos.x - self.rac.other_players[playerID].x
            dz = self.pos.z - self.rac.other_players[playerID].z
            dist = math.sqrt(dx**2 + dz**2)
            
            # will probably want to come up with a way to vary this
            if dist < 6:
                # move in the opposite direction
                newx = self.pos.x + dx
                newz = self.pos.z + dz
                
                
                print ("my pos: %2f, %2f, %2f") %(
                    self.pos.x,
                    self.pos.y,
                    self.pos.z)
                                                  
                print ("player pos: %2f, %2f, %2f") %(
                    self.rac.other_players[playerID].x,
                    self.rac.other_players[playerID].y,
                    self.rac.other_players[playerID].z)
                                                      
                print ("new pos: %2f, %2f, %2f") %(
                    newx,
                    self.pos.y,
                    newz)
                                                  
                
                
                self.movement.move_to(newx, self.pos.y, newz)

