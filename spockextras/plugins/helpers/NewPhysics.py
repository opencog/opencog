"""
created by Bradley Sheneman

modified version of SpockBot physics.py
changed the walk, sprint, jump functions to a single move function for code clarity

more importantly, it allows for single movements that are composites of
different motions (e.g. jump while walking northeast)

does not directly impact any data. calculates a new vector
which the client can use at its discretion

"""


# documentation for SpockBot physics plugin
"""
PhysicsPlugin is planned to provide vectors and tracking necessary to implement
SMP-compliant client-side physics for entities. Primarirly this will be used to
keep update client position for gravity/knockback/water-flow etc. But it should
also eventually provide functions to track other entities affected by SMP
physics

Minecraft client/player physics is unfortunately very poorly documented. Most of
these values are based of experimental results and the contributions of a
handful of people (Thank you 0pteron!) to the Minecraft wiki talk page on
Entities and Transportation. Ideally someone will decompile the client with MCP
and document the totally correct values and behaviors.
"""


# Gravitational constants defined in blocks/(client tick)^2
PLAYER_ENTITY_GAV = 0.08
THROWN_ENTITY_GAV = 0.03
RIDING_ENTITY_GAV = 0.04
BLOCK_ENTITY_GAV  = 0.04
ARROW_ENTITY_GAV  = 0.05

# Air drag constants defined in 1/tick
PLAYER_ENTITY_DRG = 0.02
THROWN_ENTITY_DRG = 0.01
RIDING_ENTITY_DRG = 0.05
BLOCK_ENTITY_DRG  = 0.02
ARROW_ENTITY_DRG  = 0.01

# Player ground acceleration isn't actually linear, but we're going to pretend
# that it is. Max ground velocity for a walking client is 0.215blocks/tick, it
# takes a dozen or so ticks to get close to max velocity. Sprint is 0.28, just
# apply more acceleration to reach a higher max ground velocity
PLAYER_WLK_ACC    = 0.15
PLAYER_SPR_ACC    = 0.20
PLAYER_GND_DRG    = 0.41

# we can add more if there are more speeds that a player can move
motions = { 1: PLAYER_WLK_ACC,
            2: PLAYER_SPR_ACC }

# Seems about right, not based on anything
PLAYER_JMP_ACC    = 0.45

import math
from spock.utils import pl_announce
from spock.mcmap import mapdata
from spock.utils import BoundingBox, Vec3

import logging
logger = logging.getLogger('spock')

class NewPhysicsCore:
    def __init__(self, vec, pos):
    
        self.vec = vec
        self.pos = pos
    
    def move(self, direction, motion, jump):

        acc = motions[motion]

        # as before, we assume angles are in degrees
        angle = math.radians(direction)
        z = math.sin(angle)*acc
        x = math.cos(angle)*acc
        y = 0.0

        if jump:
            if self.pos.on_ground:
                self.pos.on_ground = False
                y = PLAYER_JMP_ACC
        
        self.vec.add_vector(x=x, y=y, z=z)



@pl_announce('NewPhysics')
class NewPhysicsPlugin:
	def __init__(self, ploader, settings):
		self.vec = Vec3(0.0, 0.0, 0.0)
		self.playerbb = BoundingBox(0.8, 1.8)
		self.world = ploader.requires('World')
		self.event = ploader.requires('Event')
		clinfo = ploader.requires('ClientInfo')
		self.pos = clinfo.position
		ploader.reg_event_handler('physics_tick', self.tick)
		self.pycore = NewPhysicsCore(self.vec, self.pos)
		ploader.provides('NewPhysics', self.pycore)
        
	def tick(self, _, __):
		self.check_collision()
		self.apply_horizontal_drag()
		self.apply_vector()

	
        def check_collision(self):
		cb = Vec3(math.floor(self.pos.x), math.floor(self.pos.y), math.floor(self.pos.z))
		if self.block_collision(cb, y=2): #we check +2 because above my head
			self.vec.y = 0
		if self.block_collision(cb, y=-1): #we check below feet
			self.pos.on_ground = True
			self.vec.y = 0
			self.pos.y = cb.y
		else:
			self.pos.on_ground = False
			self.vec.add_vector(y = -PLAYER_ENTITY_GAV)
			self.apply_vertical_drag()
		#feet or head collide with x
		if self.block_collision(cb, x=1) or self.block_collision(cb, x=-1) or self.block_collision(cb, y=1, x=1) or self.block_collision(cb, y=1, x=-1):
			self.vec.x = 0
			#replace with real info in event
			self.event.emit("phy_collision", "x")
		#feet or head collide with z
		if self.block_collision(cb, z=1) or self.block_collision(cb, z=-1) or self.block_collision(cb, y=1, z=1) or self.block_collision(cb, y=1, z=-1):
			self.vec.z = 0
			#replace with real info in event
			self.event.emit("phy_collision", "z")

	def block_collision(self, cb, x = 0, y = 0, z = 0):
		block_id, meta = self.world.get_block(cb.x+x, cb.y+y, cb.z+z)
		block = mapdata.get_block(block_id, meta)
		if block == None:
			return False
		#possibly we want to use the centers of blocks as the starting points for bounding boxes instead of 0,0,0
		#this might make thinks easier when we get to more complex shapes that are in the center of a block aka fences but more complicated for the player
		#uncenter the player position and bump it up a little down to prevent colliding in the floor
		pos1 = Vec3(self.pos.x-self.playerbb.w/2, self.pos.y-0.2, self.pos.z-self.playerbb.d/2)
		bb1 = self.playerbb
		bb2 = block.bounding_box
		if bb2 != None:
			pos2 = Vec3(cb.x+x+bb2.x, cb.y+y+bb2.y, cb.z+z+bb2.z)
			if ((pos1.x + bb1.w) >= (pos2.x) and (pos1.x) <= (pos2.x + bb2.w)) and \
				((pos1.y + bb1.h) >= (pos2.y) and (pos1.y) <= (pos2.y + bb2.h)) and \
				((pos1.z + bb1.d) >= (pos2.z) and (pos1.z) <= (pos2.z + bb2.d)):
				return True
		return False


	def apply_vertical_drag(self):
		self.vec.y = self.vec.y - self.vec.y*PLAYER_ENTITY_DRG

	def apply_horizontal_drag(self):
		self.vec.x -= self.vec.x * PLAYER_GND_DRG
		self.vec.z -= self.vec.z * PLAYER_GND_DRG
        

	def apply_vector(self):
		p = self.pos
		p.x = p.x + self.vec.x
		p.y = p.y + self.vec.y
		p.z = p.z + self.vec.z
        
