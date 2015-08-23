#!/usr/bin/env python

"""
created by Bradley Sheneman

script to gather material metadata from Spock, combine similar types (e.g. all types of wood) into a
single material type, and map to a common list of names

"""
import roslib; roslib.load_manifest('minecraft_bot')
import rospy

from spock.mcmap import mapdata



names_map = {
        'WOODEN_PLANKS': 'PLANKS',
        'WATER': 'WATER_FLOWING',
        'STATIONARY_WATER': 'WATER',
        'LAVA': 'LAVA_FLOWING',
        'STATIONARY_LAVA': 'LAVA',
        'GOLD_ORE': 'ORE',
        'IRON_ORE': 'ORE',
        'COAL_ORE': 'ORE',
        'LAPIS_LAZULI_ORE': 'ORE',
        'LAPIS_LAZULI_BLOCK': 'REFINED',
        'DEAD_BUSH': 'SHRUB',
        'BLOCK_MOVED_BY_PISTON': 'MOVED_BY_PISTON',
        'ROSE': 'FLOWER',
        'BROWN_MUSHROOM': 'MUSHROOM',
        'RED_MUSHROOM': 'MUSHROOM',
        'BLOCK_OF_GOLD': 'REFINED',
        'BLOCK_OF_IRON': 'REFINED',
        'MOSS_STONE': 'MOSS_COBBLESTONE',
        'WOODEN_STAIRS': 'STAIRS',
        'DIAMOND_ORE': 'ORE',
        'BLOCK_OF_DIAMOND': 'REFINED',
        'BURNING_FURNACE': 'FURNACE',
        'WOODEN_DOOR': 'DOOR',
        'REDSTONE_ORE': 'ORE',
        'GLOWING_REDSTONE_ORE': 'ORE',
        'REDSTONE_TORCH_(INACTIVE)': 'REDSTONE_TORCH',
        'REDSTONE_TORCH_(ACTIVE)': 'REDSTONE_TORCH',
        'SNOW': 'SNOW_COVER',
        'SNOW_BLOCK': 'SNOW',
        'SUGAR_CANE': 'SUGARCANE',
        'STONE_SLAB': 'SLAB',
        'DOUBLE_STONE_SLAB': 'DOUBLE_SLAB',
        'JACK_\'O\'_LANTERN': 'JACK_O_LANTERN',
        'REDSTONE_REPEATER_(INACTIVE)': 'REDSTONE_REPEATER',
        'REDSTONE_REPEATER_(ACTIVE)': 'REDSTONE_REPEATER',
        'STONE_BRICK': 'BRICKS',
        'REDSTONE_LAMP_(INACTIVE)': 'REDSTONE_LAMP',
        'REDSTONE_LAMP_(ACTIVE)': 'REDSTONE_LAMP',
        'WOODEN_DOUBLE_SLAB': 'DOUBLE_SLAB',
        'WOODEN_SLAB': 'SLAB',
        'EMERALD_ORE': 'ORE',
        'BLOCK_OF_EMERALD': 'REFINED',
        'SPRUCE_WOOD_STAIRS': 'STAIRS',
        'BIRCH_WOOD_STAIRS': 'STAIRS',
        'JUNGLE_WOOD_STAIRS': 'STAIRS',
        'WOODEN_BUTTON': 'BUTTON',
        'WEIGHTED_PRESSURE_PLATE_(HEAVY)': 'PRESSURE_PLATE',
        'WEIGHTED_PRESSURE_PLATE_(LIGHT)': 'PRESSURE_PLATE',
        'REDSTONE_COMPARATOR_(INACTIVE)': 'REDSTONE_COMPARATOR',
        'REDSTONE_COMPARATOR_(ACTIVE)': 'REDSTONE_COMPARATOR',
        'BLOCK_OF_REDSTONE': 'REFINED',
        'NETHER_QUARTZ_ORE': 'ORE',
        'BLOCK_OF_QUARTZ': 'REFINED',
        'QUARTZ_STAIRS': 'STAIRS',
        'STAINED_CLAY': 'CLAY',
        'STAINED_GLASS_PANE': 'GLASS_PANE',
        'ACACIA_LEAVES': 'LEAVES',
        'ACACIA_WOOD': 'LOGS',
        'ACACIA_STAIRS': 'STAIRS',
        'DARK_OAK_STAIRS': 'STAIRS',
        'IRON_TRAPDOOR': 'TRAPDOOR',
        'COAL': 'REFINED',
        'FREE_STANDING_BANNER': 'BANNER',
        'WALL_MOUNTED_BANNER': 'BANNER',
        'INVERTED_DAYLIGHT_SENSOR': 'DAYLIGHT_SENSOR',
        'RED_SANDSTONE': 'SANDSTONE',
        'RED_SANDSTONE_STAIRS': 'STAIRS',
        'RED_SANDSTONE_DOUBLE_SLAB': 'DOUBLE_SLAB',
        'RED_SANDSTONE_SLAB': 'SLAB',
        'SANDSTONE_STAIRS': 'STAIRS',
        'SPRUCE_FENCE_GATE': 'FENCE_GATE',
        'BIRCH_FENCE_GATE': 'FENCE_GATE',
        'JUNGLE_FENCE_GATE': 'FENCE_GATE',
        'DARK_OAK_FENCE_GATE': 'FENCE_GATE',
        'ACACIA_FENCE_GATE': 'FENCE_GATE',
        'SPRUCE_FENCE': 'FENCE',
        'BIRCH_FENCE': 'FENCE',
        'JUNGLE_FENCE': 'FENCE',
        'DARK_OAK_FENCE': 'FENCE',
        'ACACIA_FENCE': 'FENCE',
        'SPRUCE_DOOR': 'DOOR',
        'BIRCH_DOOR': 'DOOR',
        'JUNGLE_DOOR': 'DOOR',
        'ACACIA_DOOR': 'DOOR',
        'DARK_OAK_DOOR': 'DOOR',
        'MOSS_COBBLESTONE': 'COBBLESTONE',
        'COBBLESTONE_STAIRS': 'STAIRS',
        'STONE_PRESSURE_PLATE': 'PRESSURE_PLATE',
        'IRON_DOOR': 'DOOR',
        'WOODEN_PRESSURE_PLATE': 'PRESSURE_PLATE',
        'BRICK_STAIRS': 'STAIRS',
        'STONE_BRICK_STAIRS': 'STAIRS',
        'NETHER_BRICK_FENCE': 'FENCE',
        'NETHER_BRICK_STAIRS': 'STAIRS',
        'COBBLESTONE_WALL': 'FENCE',
        'NETHER_BRICK': 'BRICKS',
        'STONE_BUTTON': 'BUTTON',
        'SUNFLOWER': 'FLOWER'
        }


def get_correct_name(name):

    if name in names_map:
        return names_map[name]
    else:
        return name


def get_generic_names():
    
    generic = []

    for bid in range(198):
        block = mapdata.get_block(bid)

        namelist = block.display_name.upper().split()
        name = namelist[0].strip()
        if len(namelist) > 1:
            for partial in namelist[1:]:
                name = name + '_' + partial.strip()
        
        name = get_correct_name(name)
        
        if name not in generic:
            generic.append(name)

namefile = open('mc_generic_names.csv', 'wb')


for name in sorted(generic):
    namefile.write("%s\n"%name)


