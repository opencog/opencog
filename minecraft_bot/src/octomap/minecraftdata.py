#!/usr/bin/env python

"""
created by Bradley Sheneman

script to gather material metadata from Spock, combine similar types (e.g. all types of wood) into a
single material type, and map to a common list of names

"""
import roslib; roslib.load_manifest('minecraft_bot')
import rospy

from spock.mcmap import mapdata


wood_subtypes = {'OAK', 'SPRUCE', 'BIRCH', 'JUNGLE', 'ACACIA', 'DARK_OAK'}

decorative_rock_subtypes = {'GRANITE', 'DIORITE', 'ANDESITE'}

cobble_subtypes = {'STONE', 'MOSSY_STONE'}

soil_subtypes = {'GRASS', 'DIRT', 'COARSE_DIRT', 'PODZOL'}

wool_subtypes = {'WHITE', 'ORANGE', 'MAGENTA', 'LIGHT_BLUE', 'YELLOW', 'LIME', 'PINK', 'GRAY', 'LIGHT_GRAY', 'CYAN', 'PURPLE', 'BLUE', 'BROWN', 'GREEN', 'RED', 'BLACK'}

sand_subtypes = {'WHITE', 'RED'}




outfile = open('mc_materials.csv', 'wb')

outfile.write("{:>6s},{:>32s},{:>12s},{:>12s},{:>8s}\n"
        .format('id', 'name', 'hardness', 'stacksize', 'type'))

mats = {}

for bid in range(198):
    block = mapdata.get_block(bid)

    namelist = block.display_name.upper().split()
    name = namelist[0].strip()
    if len(namelist) > 1:
        for partial in namelist[1:]:
            name = name + '_' + partial.strip()


    if block.hardness:
        hardness = block.hardness
    else: hardness = -1


    if block.material == 'melon' or block.material == 'plant':
            material = block.material.upper()

    elif type(block.material) == int:
        mid = block.material    
        
        if mid == 0:
            material = 'ROCK'
        elif mid == 1:
            material = 'SOIL'
        elif mid == 2:
            material = 'WOOD'
        elif mid == 3:
            material = 'WEB'
        elif mid == 4:
            material = 'WOOL'
        elif mid == 5:
            material = 'VINE'
        elif mid == 6:
            material = 'LEAF'

        #print block.material
    else:
        material = "NONE"

    mats[bid] = {}
    mats[bid]['name'] = name
    mats[bid]['stacksize'] = block.stack_size
    mats[bid]['diggable'] = block.diggable
    mats[bid]['tools'] = block.harvest_tools
    mats[bid]['hardness'] = hardness
    mats[bid]['type'] = material

for item in mats:
    print 'name: ' + mats[item]['name']
    print 'type: ' + mats[item]['type']
    print 'hardness: ' + str(mats[item]['hardness'])
    print 'stacksize: ' + str(mats[item]['stacksize'])
    print 'diggable?: ' + str(mats[item]['diggable'])
    print '\n'
    
   

outfile.close()

"""
    outfile.write("{:>6d},{:>32s},{:>12.2f},{:>12d},{:>8s}\n"
            .format(
                bid,
                block.display_name,
                hardness,
                block.stack_size,
                material))
"""
