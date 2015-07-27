#! /usr/bin/env python

"""
created by Bradley Sheneman
idmap that extracts useable data from the data field of a block or item
currently only for blocks

"""

stone_types = {
        0:'STONE',
        1:'GRANITE',
        2:'POLISHED_GRANITE',
        3:'DIORITE',
        4:'POLISHED_DIORITE',
        5:'ANDESITE',
        6:'POLISHED_ANDESITE'
        }
stone_slab_types = {
        0:'STONE',
        1:'WHITE_SANDSTONE',
        2:'WOODSTONE',
        3:'COBBLESTONE',
        4:'CLAY_BRICK',
        5:'STONE_BRICK',
        6:'NETHERRACK_BRICK',
        7:'NETHER_QUARTZ'
        }
wood_slab_types = {
        0:'OAK'
        }
dirt_types = {
        0:'DIRT',
        1:'COARSE_DIRT',
        2:'PODZOL'
        }
tree_types = {
        0:'OAK',
        1:'SPRUCE',
        2:'BIRCH',
        3:'JUNGLE',
        4:'ACACIA',
        5:'DARK_OAK'
        }
tree_types1 = {
        0:'OAK',
        1:'SPRUCE',
        2:'BIRCH',
        3:'JUNGLE'
        }
tree_types2 = {
        0:'ACACIA',
        1:'DARK_OAK',
        }
sandstone_shapes = {
        0:'',
        1:'CHISELED',
        2:'SMOOTH'
        }
sand_colors = {
        0:'WHITE',
        1:'RED'
        }

dye_colors = {
        0:'WHITE',
        1:'ORANGE',
        2:'MAGENTA',
        3:'LIGHT_BLUE',
        4:'YELLOW',
        5:'LIME',
        6:'PINK',
        7:'GRAY',
        8:'LIGHT_GRAY',
        9:'CYAN',
        10:'PURPLE',
        11:'BLUE',
        12:'BROWN',
        13:'GREEN',
        14:'RED',
        15:'BLACK'
        }


subtypes_map = {
        1:      {
                'name':'',
                'types': stone_types
                },

        3:      {
                'name':'',
                'types':dirt_types
                },

        5:      {
                'name':'PLANKS',
                'types': tree_types
                },
        
        6:      {
                'name':'SAPLING',
                'types': tree_types
                },
        
       
        12:     {
                'name':'SAND',
                'types': sand_colors
                },
        
        17:     {
                'name':'LOG',
                'types': tree_types1
                },
        
        18:     {
                'name':'LEAVES',
                'types': tree_types1
                },
        
        19:     {
                'name':'SPONGE',
                'types':
                    {
                        0:'',
                        1:'WET'
                    }
                },
        
        24:     {
                'name':'WHITE_SANDSTONE',
                'types': sandstone_shapes
                },
        
        31:     {
                'name':'',
                'types':
                    {
                        0:'SHRUB',
                        1:'TALL_GRASS',
                        2:'FERN'
                    }
                },
        
        35:     {
                'name':'WOOL',
                'types': dye_colors
                },
        
        38:     {
                'name':'',
                'types':
                    {
                        0:'DANDELION',
                        1:'POPPY',
                        2:'BLUE_ORCHID',
                        3:'ALLIUM',
                        4:'AZURE_BLUET',
                        5:'RED_TULIP',
                        6:'ORANGE_TULIP',
                        7:'WHITE_TULIP',
                        8:'PINK_TULIP',
                        9:'OXEYE_DAISY'
                    }
                },
        
        43:     {
                'name':'DOUBLE_SLAB',
                'types': stone_slab_types
                },
        
        44:     {
                'name':'SLAB',
                'types': stone_slab_types
                },
        
        95:     {
                'name':'GLASS',
                'types': dye_colors
                },
        
        97:     {
                'name':'MONSTER_EGG',
                'types':
                    {
                    0:'STONE',
                    1:'COBBLESTONE',
                    2:'STONE_BRICK',
                    3:'MOSSY_STONE_BRICK',
                    4:'CRACKED_STONE_BRICK',
                    5:'CHISELED_STONE_BRICK',
                    }
                },
        
        98:     {
                'name':'BRICKS',
                'types':
                    {
                    0:'STONE',
                    1:'MOSSY_STONE',
                    2:'CRACKED_STONE',
                    3:'CHISELED_STONE',
 
                    }
                },
        
        125:    {
                'name':'SLAB',
                'types': tree_types
                },
        
        126:    {
                'name':'SLAB',
                'types': tree_types
                },
        
        139:    {
                'name':'COBBLESTONE',
                'types': 
                    {
                        0:'',
                        1:'MOSSY'
                    }
                },
        
        155:    {
                'name':'NETHER_QUARTZ',
                'types':
                    {
                        0:'',
                        1:'CHISELED',
                        2:'PILLAR'
                    }
                },
        
        159:    {
                'name':'HARDENED_CLAY',
                'types': dye_colors
                },
        
        160:    {
                'name':'GLASS_PANE',
                'types': dye_colors
                },
        
        161:    {
                'name':'LEAVES',
                'types': tree_types2
                },
        
        162:    {
                'name':'LOG',
                'types': tree_types2
                },
        
        168:    {
                'name':'',
                'types':
                    {
                        0:'PRISMARINE',
                        1:'PRISMARINE_BRICKS',
                        2:'DARK_PRISMARINE'
                    }
                },
        
        171:    {
                'name':'CARPET',
                'types': dye_colors
                },
        
        175:    {
                'name':'',
                'types':
                    {
                        0:'SUNFLOWER',
                        2:'LILAC',
                        3:'TALL_GRASS',
                        4:'FERN',
                        5:'ROSE_BUSH',
                        6:'PEONY'
                    }
                },
        
        179:    {
                'name':'RED_SANDSTONE',
                'types': sandstone_shapes
                },
        }

states_map = {



        }

def check_generics():
    for key in sorted(subtypes_map):
        name = subtypes_map[key]['name']
        types = subtypes_map[key]['types']
        
        for typekey in sorted(types):
            print key, typekey
            full_name = (types[typekey] + '_' + name).strip('_')
            print full_name
       

def get_data(data):
    
    blockid = data >> 4
    meta = data&0x0F
    data_values = datamap[blockid]
    meta_values = metamap[blockid]

    for key in data_values:
        int_val = data_values[key](meta)
        str_val = meta_values[key][int_val]

        print key + " data: " + str(int_val) + ' value: ' + str(str_val)


# meta always integer(equivalent to four bits), and mask always four bits
def get_bits(metadata, mask):

    #possibilties for mask:
    masks = {
                0b1000: lambda meta: (meta >> 3)&0x01,
                0b0100: lambda meta: (meta >> 2)&0x01,
                0b0010: lambda meta: (meta >> 1)&0x01,
                0b0001: lambda meta: (meta     )&0x01,
                0b1100: lambda meta: (meta >> 2)&0x03,
                0b0110: lambda meta: (meta >> 1)&0x03,
                0b0011: lambda meta: (meta     )&0x03,
                0b1110: lambda meta: (meta >> 1)&0x07,
                0b0111: lambda meta: (meta     )&0x07
            }

    return masks[mask](metadata)


datamap = {}
metamap = {}

#saplings
datamap[6]= {
    'type': lambda meta: get_bits(meta, 0b0111),
    'growth': lambda meta: get_bits(meta, 0b1000)
    }
metamap[6] = {
        'type': tree_types,
        'growth': {0:'NOT_GROWING',1:'GROWING'} 
        }

#logs
datamap[17]= {
    'type': lambda meta: get_bits(meta, 0b0011),
    'facing': lambda meta: get_bits(meta, 0b1100)
    }
datamap[162]= {
    'type': lambda meta: get_bits(meta, 0b0011),
    'facing': lambda meta: get_bits(meta, 0b1100)
    }
       


if __name__ == "__main__":

    #get_data(104)
    
    check_generics()
    #print "meta 1001 mask 0001: %d\n"%(get_bits(9,0b0001))
    #print "meta 1001 mask 0010: %d\n"%(get_bits(9,0b0010))
    #print "meta 1001 mask 0011: %d\n"%(get_bits(9,0b0011))
    #print "meta 1001 mask 0110: %d\n"%(get_bits(9,0b0110))
    #print "meta 1001 mask 1000: %d\n"%(get_bits(9,0b1000))
    #print "meta 1001 mask 1100: %d\n"%(get_bits(9,0b1100))
