"""
An entity tracker
"""
import roslib; roslib.load_manifest('minecraft_bot')

# import all entity related message files
from minecraft_bot.msg import entity_msg, entity_exp_meta, entity_global_meta, entity_mob_meta
from minecraft_bot.msg import entity_movement_meta, entity_object_meta, entity_painting_meta, entity_player_meta 



from spockbot.plugins.base import pl_announce
from spockbot.mcdata.utils import Info

import logging
logger = logging.getLogger('spock')

class MCEntity(Info):
        x = 0
        y = 0
        z = 0
	eid = 0
	status = 0
        type = -1
        mob_meta = None
        object_meta = None
        painting_meta = None
        player_meta = None
        exp_meta = None
        globa_meta = None
	nbt = None

class ClientPlayerEntity(Info):
	metadata = None

class MovementEntity(Info):
	yaw = 0
	pitch = 0
	on_ground = True

class PlayerEntity(Info):
	uuid = 0
	current_item = 0
	metadata = None

class ObjectEntity(Info):
	obj_type = 0
	obj_data = 0
	velocity_x = 0
	velocity_y = 0
	velocity_z = 0

class MobEntity(Info):
	mob_type = 0
	head_pitch = 0
	head_yaw = 0
	velocity_x = 0
	velocity_y = 0
	velocity_z = 0
	metadata = None

class PaintingEntity(Info):
	title = ""
	direction = 0

class ExpEntity(Info):
	count = 0

class GlobalEntity(Info):
	global_type = 0

class SendEntityDataCore:
	def __init__(self):
		self.client_player = ClientPlayerEntity()
		#self.entities = {}
		#self.players = {}
		#self.mobs = {}
		#self.objects = {}
		#self.paintings = {}
		#self.exp_orbs = {}
		#self.global_entities = {}

@pl_announce('SendEntityData')
class SendEntityDataPlugin:
    
    def __init__(self, ploader, settings):
        
        self.event = ploader.requires('Event')
        handles = (
                ('PLAY<Join Game', self.handle_join_game),
                ('PLAY<Spawn Player', self.handle_spawn_player),
                ('PLAY<Spawn Object', self.handle_spawn_object),
                ('PLAY<Spawn Mob', self.handle_spawn_mob),
                ('PLAY<Spawn Painting', self.handle_spawn_painting),
                ('PLAY<Spawn Experience Orb', self.handle_spawn_experience_orb),
                ('PLAY<Destroy Entities', self.handle_destroy_entities),
                ('PLAY<Entity Equipment', self.handle_unhandled),
                ('PLAY<Entity Velocity', self.handle_set_dict),
                ('PLAY<Entity Relative Move', self.handle_relative_move),
                ('PLAY<Entity Look', self.handle_set_dict),
                ('PLAY<Entity Look and Relative Move', self.handle_relative_move),
                ('PLAY<Entity Teleport', self.handle_set_dict),
                ('PLAY<Entity Head Look', self.handle_set_dict),
                ('PLAY<Entity Status', self.handle_set_dict),
                ('PLAY<Entity Metadata', self.handle_set_dict),
                ('PLAY<Entity Effect', self.handle_unhandled),
                ('PLAY<Remove Entity Effect', self.handle_unhandled),
                ('PLAY<Entity Properties', self.handle_unhandled),
                ('PLAY<Spawn Global Entity', self.handle_spawn_global_entity),
                ('PLAY<Update Entity NBT', self.handle_set_dict),
                )
        
        for event, handler in handles:
            ploader.reg_event_handler(event, handler)
        
        self.core = SendEntityDataCore()
        ploader.provides('SendEntityData', self.core)

    
    #TODO: Implement all these things
    def handle_unhandled(self, event, packet):
        
        pass
    

    def handle_join_game(self, event, packet):
        
        #print packet
        entity = MCEntity()
        meta = PlayerEntity()
        move = MovementEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
        move.set_dict(packet.data)
                
        meta.movement = move
        entity.player = meta
        entity.type = 3

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)


    def handle_spawn_player(self, event, packet):
        
        #print packet
        entity = MCEntity()
        meta = PlayerEntity()
        move = MovementEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
        move.set_dict(packet.data)
                
        meta.movement = move
        entity.player = meta
        entity.type = 3

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)

        
    def handle_spawn_object(self, event, packet):
        
        entity = MCEntity()
        meta = ObjectEntity()
        move = MovementEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
        move.set_dict(packet.data)
                
        meta.movement = move
        entity.object = meta
        entity.type = 1

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)


    def handle_spawn_mob(self, event, packet):
        
        entity = MCEntity()
        meta = MobEntity()
        move = MovementEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
        move.set_dict(packet.data)
                
        meta.movement = move
        entity.mob = meta
        entity.type = 0

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)


    def handle_spawn_painting(self, event, packet):
        
        entity = MCEntity()
        meta = PaintingEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
                
        entity.painting = meta
        entity.type = 2

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)


    def handle_spawn_experience_orb(self, event, packet):
        
        entity = MCEntity()
        meta = ExpEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
                
        entity.exp = meta
        entity.type = 4

        #print(entity)
        self.event.emit('ros_entity_data', entity)


    def handle_spawn_global_entity(self, event, packet):
        
        entity = MCEntity()
        meta = GlobalEntity()

        entity.set_dict(packet.data)
        meta.set_dict(packet.data)
                
        entity.globalentity = meta
        entity.type = 5

        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)


    def handle_relative_move(self, event, packet):
         
        entity = MCEntity()
        entity.set_dict(packet.data)

        #type unknown
        entity.type = -1

        entity.x = packet.data['dx']
        entity.y = packet.data['dy']
        entity.z = packet.data['dz']
        
        #print(entity)
        self.event.emit('ros_entity_data', entity.__dict__)



    def handle_destroy_entities(self, event, packet):
        
        pass


    def handle_set_dict(self, event, packet):
        
        pass
        #if packet.data['eid'] in self.ec.entities:
            #self.ec.entities[packet.data['eid']].set_dict(packet.data)
