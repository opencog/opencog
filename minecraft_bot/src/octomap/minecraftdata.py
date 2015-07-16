#!/usr/bin/env python

"""
created by Bradley Sheneman

script to gather material metadata from Spock, combine similar types (e.g. all types of wood) into a
single material type, and map to a common list of names

"""
import roslib; roslib.load_manifest('minecraft_bot')
import rospy

from spock.mcmap import mapdata

# types: used for classifying Minecraft materials in OpenCog. Mainly determines crafting recipes

# WOOD: all LOG and PLANK items, as well as items made using another WOOD type. CHARCOAL included. flammable
# ROCK: including or created from any ROCK type, where base type is any hard substance generated naturally
# WORLD: generated in the world. not necessarily 'natural'. e.g. includes end portals and strongholds
# CLAY
# METAL: refined from ore
# ORE: any block containing ore
# SEDIMENT: sand, red sand, gravel. only blocks affected by gravity...
# SOIL: matter in which seeds/cuttings can be planted. special types spontaneously
#   produces a covering (grass, mycelium, podzol) under certain conditions
# PLANT: produced when SEED is placed in SOIL. does not require a special tool to harvest
# WOOL: gathered by shearing sheep, or dying any other wool. used to make beds or carpet
# WEB: often found in abandoned mineshafts, harvested with SHEARS
# VINE: appear naturally on trees in JUNGLE and SWAMP biomes. grow downward. harvested with SHEARS
# GLASS: fragile items that are often not harvestable. any tools will break equally well
# FRUIT: (botanical definition) melons, pumpkins, cocoa pods, and jack-o-lanterns.
#   harvested with axes, can be 'crafted' into SEED
# LEAF: spontaneously despawn when not near LOG item. naturally appear when trees grow
# MACHINE: block that can produce an external effect, typically its recipe includes REDSTONE_DUST

# renamed WOOD to LOG, for better accuracy. we will assume all wood type materials can have any wood subtype

world = [
    'DECORATIVE_STONE',
    'BEDROCK',
    'LOG',
    'CLAY',
    'ORE',
    'COARSE_DIRT',
    'CRACKED_STONE_BRICK',
    'DIRT',
    'DRAGON_EGG',
    'END_STONE',
    'GLOWSTONE',
    'GRASS',
    'GRAVEL',
    'ICE',
    'LAVA',
    'LEAVES',
    'LILY_PAD',
    'MONSTER_EGG',
    'MONSTER_SPAWNER',
    'MOSS_STONE',
    'MOSSY_STONE_BRICK',
    'MYCELIUM',
    'NETHER_BRICK',
    'NETHER_QUARTZ',
    'NETHERRACK',
    'OBSIDIAN',
    'SANDSTONE',
    'POLISHED_DECORATIVE_STONE',
    'SAND',
    'SNOW',
    'SOUL_SAND',
    'STONE',
    'VINE',
    'WATER',
    'WEB'
    'RAILS'
    ]

"""
woods = [
    'WOODEN_PLANKS',
    'LOG',
    'NOTE_BLOCK',
    'BED',
    'PISTON',
    'STICKY_PISTON',
    'BOOKSHELF',
    'TORCH',
    'WOODEN_STAIRS',
    'CHEST',
    'CRAFTING_TABLE',
    'SIGN_POST',
    'WOODEN_DOOR',
    'LADDER',
    'WALL_SIGN',
    'LEVER',
    'WOODEN_PRESSURE_PLATE',
    'REDSTONE_TORCH_ACTIVE',
    'REDSTONE_TORCH_INACTIVE',
    'POWERED_RAIL',
    'JUKEBOX',
    'FENCE',
    'LOCKED_CHEST',
    'TRAPDOOR',
    'FENCE_GATE',
    'WOODEN_DOUBLE_SLAB',
    'WOODEN_SLAB',
    'TRIPWIRE_HOOK',
    'WOODEN_BUTTON',
    'TRAPPED_CHEST',
    'DAYLIGHT_SENSOR',
    'FREE_STANDING_BANER',
    'WALL_MOUNTED_BANNER',
    'INVERTED_DAYLIGHT_SENSOR',
    'RAIL'
    'EMERALD_ORE'
    ]

# note: redstone ore includes glowing redstone ore
# this includes stone, cobble, or anything made from them
stones = [
    'STONE',
    'COBBLESTONE',
    'GOLD_ORE',
    'IRON_ORE',
    'COAL_ORE',
    'LAPIS_LAZULI_ORE',
    'DISPENSER',
    'DETECTOR_RAIL',
    'PISTON',
    'STICKY_PISTON',
    'DOUBLE_STONE_SLAB',
    'STONE_SLAB',
    'MOSS_STONE',
    'MONSTER_SPAWNER',
    'DIAMOND_ORE',
    'FURNACE',
    'FURNACE_ACTIVE',
    'COBBLESTONE_STAIRS',
    'LEVER',
    'STONE_PRESSURE_PLATE',
    'REDSTONE_ORE',
    'STONE_BUTTON',
    'STONE_BRICKS',
    'STONE_BRICK_STAIRS',
    'TRIPWIRE_HOOK',
    'COBBLESTONE_WALL',
    'REDSTONE_COMPARATOR_INACTIVE',
    'REDSTONE_COMPARATOR_ACTIVE',
    'DROPPER',
    'REDSTONE_REPEATER_ACTIVE',
    'REDSTONE_REPEATER_INACTIVE',
    'SANDSTONE_STAIRS',
    'EMERALD_ORE',
    'ENDER_CHEST',
    ]
"""

# any rock or mineral material including stone. non-stone is just the complement of the intersection
# of these two sets
rocks = [
    'STONE',
    'COBBLESTONE',
    'BEDROCK',
    'GOLD_ORE',
    'IRON_ORE',
    'COAL_ORE',
    'LAPIS_LAZULI_ORE',
    'DISPENSER',
    'SANDSTONE',
    'DETECTOR_RAIL',
    'PISTON',
    'STICKY_PISTON',
    'DOUBLE_STONE_SLAB',
    'STONE_SLAB',
    'BRICKS',
    'MOSS_STONE',
    'OBSIDIAN',
    'MONSTER_SPAWNER',
    'DIAMOND_ORE',
    'FURNACE',
    'FURNACE_ACTIVE',
    'COBBLESTONE_STAIRS',
    'LEVER',
    'STONE_PRESSURE_PLATE',
    'REDSTONE_ORE',
    'STONE_BUTTON',
    'NETHERRACK',
    'GLOWSTONE',
    'STONE_BRICKS',
    'STONE_BRICK_STAIRS',
    'TRIPWIRE_HOOK',
    'COBBLESTONE_WALL',
    'REDSTONE_COMPARATOR_INACTIVE',
    'REDSTONE_COMPARATOR_ACTIVE',
    'DROPPER',
    'REDSTONE_REPEATER_ACTIVE',
    'REDSTONE_REPEATER_INACTIVE',
    'NETHER_BRICK',
    'NETHER_BRICK_FENCE',
    'NETHER_BRICK_STAIRS',
    'ENCHANTMENT_TABLE',
    'BREWING_STAND',
    'END_STONE',

    ]


# tools: determines best tool for harvesting. some blocks can still be harvested with other tools
# HATCHET (most WOOD and GOURD items)
# PICKAXE (most STONE items)
# SHOVEL
# SHEARS
# NONE: any tool functions equally well (including bare hands)

# level: (harvest level) lowest quality tool that can harvest it. any tool at/above this level will work
# 0. NONE (bare hands)
# 1. WOOD
# 2. STONE
# 3. IRON/GOLD
# 4. DIAMOND

# for the subtypes: any type can be replaced with any other type in recipes

wood_subtypes = {'OAK', 'SPRUCE', 'BIRCH', 'JUNGLE', 'ACACIA', 'DARK_OAK'}

# do not function as normal stone. purely decorative
decorative_rock_subtypes = {'GRANITE', 'DIORITE', 'ANDESITE'}

# turns out these are slightly different. e.g. can produce moss stone with cobble + vines
#cobble_subtypes = {'STONE', 'MOSSY_STONE'}

#soil_subtypes = {'GRASS', 'DIRT', 'COARSE_DIRT', 'PODZOL'}

# as far as I know, these are identical in behavior
sand_subtypes = {'RED', 'WHITE'}

ore_subtypes = {'IRON', 'COAL', 'DIAMOND', 'LAPIS_LAZULI', 'EMERALD', 'REDSTONE', 'GOLD', 'NETHER_QUARTZ'}
# used for staining wool, clay, or glass

color_subtypes = ['WHITE', 'ORANGE', 'MAGENTA', 'LIGHT_BLUE', 'YELLOW', 'LIME', 'PINK', 'GRAY', 'LIGHT_GRAY', 'CYAN', 'PURPLE', 'BLUE', 'BROWN', 'GREEN', 'RED', 'BLACK']

mushroom_subtypes = {'BROWN', 'RED'}

flower_subtypes = {'ROSE', 'FLOWER'}

binary_subtypes = {'ACTIVE', 'INACTIVE'}


mats = {}
idmap = {}

idmap[(0,0)] = 'AIR'

idmap[(1,0)] = 'STONE'
idmap[(1,1)] = 'DECOR_STONE GRANITE RAW'
idmap[(1,2)] = 'DECOR_STONE GRANITE POLISHED'
idmap[(1,3)] = 'DECOR_STONE DIORITE RAW'
idmap[(1,4)] = 'DECOR_STONE DIORITE POLISHED'
idmap[(1,5)] = 'DECOR_STONE ANDESITE RAW'
idmap[(1,6)] = 'DECOR_STONE ANDESITE POLISHED'

idmap[(2,0)] = 'GRASS'

idmap[(3,0)] = 'DIRT'
idmap[(3,1)] = 'COARSE_DIRT'
idmap[(3,2)] = 'PODZOL'

idmap[(4,0)] = 'COBBLESTONE'

idmap[(5,0)] = 'PLANKS OAK'
idmap[(5,1)] = 'PLANKS SPRUCE'
idmap[(5,2)] = 'PLANKS BIRCH'
idmap[(5,3)] = 'PLANKS JUNGLE'
idmap[(5,4)] = 'PLANKS ACACIA'
idmap[(5,5)] = 'PLANKS DARK_OAK'


idmap[(6,0)] = 'SAPLING OAK'
idmap[(6,1)] = 'SAPLING SPRUCE'
idmap[(6,2)] = 'SAPLING BIRCH'
idmap[(6,3)] = 'SAPLING JUNGLE'
idmap[(6,4)] = 'SAPLING ACACIA'
idmap[(6,5)] = 'SAPLING DARK_OAK'

idmap[(7,0)] = 'BEDROCK'

idmap[(8,0)] = 'WATER FLOWING'
idmap[(9,0)] = 'WATER SOURCE'

idmap[(10,0)] = 'LAVA FLOWING'
idmap[(11,0)] = 'LAVA SOURCE'

idmap[(12,0)] = 'SAND WHITE'
idmap[(12,1)] = 'SAND RED'

idmap[(13,0)] = 'GRAVEL'

idmap[(14,0)] = 'ORE GOLD'

idmap[(15,0)] = 'ORE IRON'

idmap[(16,0)] = 'ORE COAL'

idmap[(17,0)] = 'LOG OAK'
idmap[(17,1)] = 'LOG SPRUCE'
idmap[(17,2)] = 'LOG BIRCH'
idmap[(17,3)] = 'LOG JUNGLE'

idmap[(18,0)] = 'LEAVES OAK'
idmap[(18,1)] = 'LEAVES SPRUCE'
idmap[(18,2)] = 'LEAVES BIRCH'
idmap[(18,3)] = 'LEAVES JUNGLE'

idmap[(19,0)] = 'SPONGE DRY'
idmap[(19,1)] = 'SPONGE WET'

idmap[(20,0)] = 'GLASS'

idmap[(21,0)] = 'ORE LAPIS_LAZULI'

idmap[(22,0)] = 'REFINED LAPIS_LAZULI'

idmap[(23,0)] = 'DISPENSER'

idmap[(24,0)] = 'SANDSTONE RAW'
idmap[(24,1)] = 'SANDSTONE CHISELED'
idmap[(24,2)] = 'SANDSTONE SMOOTH'

idmap[(25,0)] = 'NOTE_BLOCK'

idmap[(26,0)] = 'BED'

idmap[(27,0)] = 'RAIL POWERED'

idmap[(28,0)] = 'RAIL DETECTOR'

idmap[(29,0)] = 'PISTON STICKY'

idmap[(30,0)] = 'COBWEB'

idmap[(31,0)] = 'SHRUB DEAD'
idmap[(31,1)] = 'SHRUB GRASS'
idmap[(31,2)] = 'SHRUB FERN'

idmap[(32,0)] = 'SHRUB DEAD'

idmap[(33,0)] = 'PISTON STANDARD'

idmap[(34,0)] = 'PISTON_EXTENSION'

for i,color in enumerate(color_subtypes):
    idmap[(35,i)] = 'WOOL ' + color


idmap[(37,0)] = 'FLOWER DANDELION'

idmap[(38,0)] = 'FLOWER POPPY'
idmap[(38,0)] = 'FLOWER BLUE_ORCHID'
idmap[(38,0)] = 'FLOWER ALLIUM'
idmap[(38,0)] = 'FLOWER AZURE_BLUET'
idmap[(38,0)] = 'FLOWER RED_TULIP'
idmap[(38,0)] = 'FLOWER ORANGE_TULIP'
idmap[(38,0)] = 'FLOWER WHITE_TULIP'
idmap[(38,0)] = 'FLOWER PINK_TULIP'
idmap[(38,0)] = 'FLOWER OXEYE_DAISY'


idmap[(39,0)] = 'MUSHROOM BROWN'

idmap[(40,0)] = 'MUSHROOM RED'

idmap[(41,0)] = 'REFINED GOLD'

idmap[(42,0)] = 'REFINED IRON'

idmap[(43,0)] = 'DOUBLE_SLAB STONE'
idmap[(43,1)] = 'DOUBLE_SLAB SANDSTONE WHITE'
idmap[(43,2)] = 'DOUBLE_SLAB WOOD OAK'
idmap[(43,3)] = 'DOUBLE_SLAB COBBLESTONE'
idmap[(43,4)] = 'DOUBLE_SLAB BRICKS CLAY'
idmap[(43,5)] = 'DOUBLE_SLAB BRICKS STONE'
idmap[(43,6)] = 'DOUBLE_SLAB BRICKS NETHERRACK'
idmap[(43,7)] = 'DOUBLE_SLAB QUARTZ'

idmap[(44,0)] = 'SLAB STONE'
idmap[(44,1)] = 'SLAB SANDSTONE WHITE'
idmap[(44,2)] = 'SLAB WOOD OAK'
idmap[(44,3)] = 'SLAB COBBLESTONE'
idmap[(44,4)] = 'SLAB BRICKS CLAY'
idmap[(44,5)] = 'SLAB BRICKS STONE'
idmap[(44,6)] = 'SLAB BRICKS NETHERRACK'
idmap[(44,7)] = 'SLAB QUARTZ'

idmap[(45,0)] = 'BRICKS CLAY'

idmap[(46,0)] = 'TNT'

idmap[(47,0)] = 'BOOKSHELF'

idmap[(48,0)] = 'MOSS_COBBLESTONE'

idmap[(49,0)] = 'OBSIDIAN'

idmap[(50,0)] = 'TORCH'

idmap[(51,0)] = 'FIRE'

idmap[(52,0)] = 'MONSTER_SPAWNER'

idmap[(53,0)] = 'STAIRS WOOD OAK'

idmap[(54,0)] = 'CHEST'

idmap[(55,0)] = 'REDSTONE_DUST'

idmap[(56,0)] = 'ORE DIAMOND'

idmap[(57,0)] = 'REFINED DIAMOND'

idmap[(58,0)] = 'CRAFTING_TABLE'

idmap[(59,0)] = 'WHEAT'

idmap[(60,0)] = 'FARMLAND'

idmap[(61,0)] = 'FURNACE INACTIVE'

idmap[(62,0)] = 'FURNACE ACTIVE'

idmap[(63,0)] = 'SIGN STANDING'

idmap[(64,0)] = 'DOOR WOOD OAK'

idmap[(65,0)] = 'LADDER'

idmap[(66,0)] = 'RAIL STANDARD'

idmap[(67,0)] = 'STAIRS COBBLESTONE'

idmap[(68,0)] = 'SIGN WALL_MOUNTED'

idmap[(69,0)] = 'LEVER'

idmap[(70,0)] = 'PRESSURE_PLATE STONE'

idmap[(71,0)] = 'DOOR IRON'

idmap[(72,0)] = 'PRESSURE_PLATE WOOD'

idmap[(73,0)] = 'ORE REDSTONE INACTIVE'

idmap[(74,0)] = 'ORE REDSTONE ACTIVE'

idmap[(75,0)] = 'REDSTONE_TORCH INACTIVE'

idmap[(76,0)] = 'REDSTONE_TORCH ACTIVE'

idmap[(77,0)] = 'BUTTON STONE'

idmap[(78,0)] = 'SNOW_COVER'

idmap[(79,0)] = 'ICE'

idmap[(80,0)] = 'SNOW'

idmap[(81,0)] = 'CACTUS'

idmap[(82,0)] = 'CLAY UNSTAINED'

idmap[(83,0)] = 'SUGARCANE'

idmap[(84,0)] = 'JUKEBOX'

idmap[(85,0)] = 'FENCE WOOD OAK'

idmap[(86,0)] = 'PUMPKIN'

idmap[(87,0)] = 'NETHERRACK'

idmap[(88,0)] = 'SOUL_SAND'

idmap[(89,0)] = 'GLOWSTONE'

idmap[(90,0)] = 'NETHER_PORTAL'

idmap[(91,0)] = 'JACK_O_LANTERN'

idmap[(92,0)] = 'CAKE'

idmap[(93,0)] = 'REDSTONE_REPEATER INACTIVE'

idmap[(94,0)] = 'REDSTONE_REPEATER ACTIVE'

for i,color in enumerate(color_subtypes):
    idmap[(95,i)] = 'GLASS ' + color


idmap[(96,0)] = 'TRAPDOOR WOOD'

idmap[(97,2)] = 'MONSTER_EGG BRICKS STONE'
idmap[(97,3)] = 'MONSTER_EGG BRICKS MOSSY_STONE'
idmap[(97,4)] = 'MONSTER_EGG BRICKS CRACKED_STONE'
idmap[(97,5)] = 'MONSTER_EGG BRICKS CHISELED_STONE'

idmap[(98,0)] = 'BRICKS STONE'
idmap[(98,1)] = 'BRICKS MOSSY_STONE'
idmap[(98,2)] = 'BRICKS CRACKED_STONE'
idmap[(98,3)] = 'BRICKS CHISELED_STONE'

idmap[(101,0)] = 'BARS IRON'

idmap[(104,0)] = 'PUMPKIN_STEM'

idmap[(106,0)] = 'VINES'

idmap[(108,0)] = 'STAIRS BRICKS CLAY'

idmap[(109,0)] = 'STAIRS BRICKS STONE'

idmap[(110,0)] = 'MYCELIUM'

idmap[(111,0)] = 'LILY_PAD'

idmap[(112,0)] = 'BRICKS NETHERRACK'

idmap[(113,0)] = 'FENCE BRICKS NETHERRACK'

idmap[(114,0)] = 'STAIRS BRICKS NETHERRACK'

idmap[(117,0)] = 'BREWING_STAND'

idmap[(123,0)] = 'REDSTONE_LAMP INACTIVE'

idmap[(124,0)] = 'REDSTONE_LAMP ACTIVE'

idmap[(125,2)] = 'DOUBLE_SLAB WOOD BIRCH'
idmap[(125,4)] = 'DOUBLE_SLAB WOOD ACACIA'

idmap[(126,2)] = 'DOUBLE_SLAB WOOD BIRCH'
idmap[(126,4)] = 'DOUBLE_SLAB WOOD ACACIA'

idmap[(128,0)] = 'STAIRS SANDSTONE WHITE'

idmap[(131,0)] = 'TRIPWIRE_HOOK'

idmap[(132,0)] = 'TRIPWIRE'

idmap[(134,0)] = 'STAIRS WOOD SPRUCE'

idmap[(135,0)] = 'STAIRS WOOD BIRCH'

idmap[(136,0)] = 'STAIRS WOOD JUNGLE'

idmap[(145,0)] = 'ANVIL'

idmap[(147,0)] = 'PRESSURE_PLATE WEIGHTED LIGHT'

idmap[(148,0)] = 'PRESSURE_PLATE WEIGHTED HEAVY'

idmap[(149,0)] = 'REDSTONE_COMPARATOR INACTIVE'

idmap[(150,0)] = 'REDSTONE_COMPARATOR ACTIVE'

idmap[(151,0)] = 'DAYLIGHT_SENSOR STANDARD'

idmap[(155,1)] = 'CHISELED_QUARTZ'

idmap[(155,2)] = 'PILLAR_QUARTZ'

idmap[(156,0)] = 'STAIRS QUARTZ'

idmap[(157,0)] = 'RAIL ACTIVATOR'

for i,color in enumerate(color_subtypes):
    idmap[(159,i)] = 'CLAY ' + color

for i,color in enumerate(color_subtypes):
    idmap[(160,i)] = 'GLASS_PANE ' + color


idmap[(161,0)] = 'LEAVES ACACIA'

idmap[(162,0)] = 'LOG ACACIA'

idmap[(163,0)] = 'STAIRS WOOD ACACIA'

idmap[(164,0)] = 'STAIRS WOOD DARK_OAK'

idmap[(165,0)] = 'SLIME'

idmap[(166,0)] = 'BARRIER'

idmap[(167,0)] = 'TRAPDOOR IRON'

idmap[(168,0)] = 'PRISMARINE'
idmap[(168,1)] = 'BRICKS PRISMARINE'
idmap[(169,2)] = 'DARK_PRISMARINE'

idmap[(171,0)] = 'CARPET WHITE'
idmap[(171,3)] = 'CARPET LIGHT_BLUE'
idmap[(171,5)] = 'CARPET LIME'
idmap[(171,6)] = 'CARPET PINK'
idmap[(171,8)] = 'CARPET LIGHT_GRAY'

idmap[(174,0)] = 'PACKED_ICE'

idmap[(175,1)] = 'FLOWER LILAC'

idmap[(176,0)] = 'BANNER STANDING'

idmap[(177,0)] = 'BANNER WALL_MOUNTED'

idmap[(178,0)] = 'DAYLIGHT_SENSOR INVERTED'

idmap[(179,1)] = 'SANDSTONE RED CHISELED'

idmap[(180,0)] = 'STAIRS SANDSTONE RED'

idmap[(184,0)] = 'FENCE_GATE WOOD BIRCH'

idmap[(187,0)] = 'FENCE GATE WOOD ACACIA'

idmap[(189,0)] = 'FENCE WOOD BIRCH'

idmap[(192,0)] = 'FENCE WOOD ACACIA'

idmap[(194,0)] = 'DOOR WOOD BIRCH'

idmap[(196,0)] = 'DOOR WOOD ACACIA'



for bid in range(198):
    block = mapdata.get_block(bid)

    namelist = block.display_name.upper().split()
    name = namelist[0].strip()
    if len(namelist) > 1:
        for partial in namelist[1:]:
            name = name + '_' + partial.strip()
    
    #name = get_correct_name(name)

    #if block.hardness == None:
    #    hardness = -1
    #else: hardness = block.hardness


    #if block.material == 'melon' or block.material == 'plant':
    #        material = block.material.upper()

    #elif type(block.material) == int:
    #    mid = block.material    
    #    
    #    if mid == 0:
    #        material = 'ROCK'
    #    elif mid == 1:
    #        material = 'SOIL'
    #    elif mid == 2:
    #        material = 'WOOD'
    #    elif mid == 3:
    #        material = 'WEB'
    #    elif mid == 4:
    #        material = 'WOOL'
    #    elif mid == 5:
    #        material = 'VINE'
    #    elif mid == 6:
    #        material = 'LEAF'

        #print block.material
    #else:
    #    material = "NONE"

    #mats[name] = {}
    #mats[bid]['name'] = name
    #if name not in generic:
    #    generic[name] = {}
    
    #if 'stacksize' not in mats[name]:
    #    mats[name]['stacksize'] = block.stack_size
    #elif mats[name]['stacksize'] != block.stack_size:
    #    print "stack sizes not equal"
    #mats[bid]['diggable'] = block.diggable
    #mats[bid]['tools'] = block.harvest_tools
    #mats[bid]['hardness'] = hardness
    #mats[bid]['type'] = material

# now for fixes, more types, and data reorganization...

matfile = open('mc_materials.csv', 'wb')
idfile = open('mc_ids_names.csv', 'wb')
namefile = open('mc_names.csv', 'wb')

for key in sorted(idmap):
    idfile.write("(%d, %d) : %s\n"%(key[0],key[1], idmap[key]))

for name in sorted(idmap.values()):
    namefile.write(name + '\n')

matfile.write("{:>32s},{:>8s},{:>12s},{:>12s}\n"
        .format('name', 'type','hardness', 'stacksize'))
"""
for item in mats:
    #print 'name: ' + mats[item]['name']
    #print 'type: ' + mats[item]['type']
    #print 'hardness: ' + str(mats[item]['hardness'])
    #print 'stacksize: ' + str(mats[item]['stacksize'])
    #print 'diggable?: ' + str(mats[item]['diggable'])
    #print '\n'
    matfile.write("{:>32s},{:>8s},{:>12.2f},{:>12d}\n"
            .format(
                mats[item]['name'],
                mats[item]['type'],
                mats[item]['hardness'],
                mats[item]['stacksize']))
"""   
   

matfile.close()
idfile.close()
namefile.close()


