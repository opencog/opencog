#!/usr/bin/env python

"""
created by Bradley Sheneman

script to gather material metadata from Spock, combine similar types (e.g. all types of wood) into a
single material type, and map to a common list of names

"""
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
import mcidmap_blocks, mcidmap_items

from spock.mcmap import mapdata



mats = {}

for bid in range(198):
    block = mapdata.get_block(bid)

    namelist = block.display_name.upper().split()
    name = namelist[0].strip()
    if len(namelist) > 1:
        for partial in namelist[1:]:
            name = name + '_' + partial.strip()
    
    #name = get_correct_name(name)

def get_block_name(namestring):
    namelist = list(reversed(namestring.split()))
    
    name = namelist[0]
    if len(namelist) > 1:
        for item in namelist[1:]:
            name = name + '_' + item

    return name


matfile = open('mc_materials.csv', 'wb')
idfile = open('mc_ids_names.csv', 'wb')
namefile = open('mc_names.csv', 'wb')
conceptfile = open('mc_concepts.csv', 'wb')

blocksmap = mcidmap_blocks.idmap
itemsmap = mcidmap_items.idmap

concepts = set()

for value in blocksmap.values():
    namelist = value.split()
    
    for concept in namelist:
        concepts.add(concept.strip())

for concept in sorted(concepts):
    conceptfile.write(concept + '\n')


for namestring in sorted(blocksmap.values()):
    namefile.write(get_block_name(namestring) + '\n')

for key in sorted(blocksmap):
    idfile.write("%d, %d, %s\n"%(key[0],key[1], get_block_name(blocksmap[key])))


matfile.write("{:>32s},{:>8s},{:>12s},{:>12s}\n"
        .format('name', 'type','hardness', 'stacksize'))
"""
for item in mats:
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
conceptfile.close()

