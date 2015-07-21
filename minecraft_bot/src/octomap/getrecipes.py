#!/usr/bin/env python


"""

small script to parse the minecraft recipes java file to get all crafting recipes.

"""

recipes = []

def getShapedRecipe(line):

    recipe = {}

    ni1 = line.find('(', 37, -1) + 1
    ni2 = line.find(')', 37, -1)
    
    namelist = line[ni1:ni2].split(',')
    #print namelist

    ns = namelist[0].find('.') + 1
    recipe['name'] = namelist[0][ns:]
    if len(namelist) > 1:
        recipe['num'] = int(namelist[1])
    else:
        recipe['num'] = 1
    
    if namelist[0].find('Items'):
        ntype = 'item'
    elif namelist[0].find('Blocks'):
        ntype = 'block'
    else:
        ntype = 'none'
    
    print recipe['name']
    
    recipe['type'] = ntype

    ri1 = line.find('{') + 1
    ri2 = line.find('}')

    shapelist = [item.strip(" \'\"") for item in line[ri1:ri2].split(',')]
    
    layers = []
    for item in shapelist:
        found = item.find('Character.valueOf')
        if found < 0:
            layers.append(item.strip(" \'\""))
        else:
            break
    
    remaining = shapelist[len(layers):]
    
    characters = [item[item.find('\'')+1] for item in remaining[0::2]]
    objects = [item for item in remaining[1::2]]

    print characters
    print objects

def getShapelessRecipe(line):
    pass


infile = open('mc_recipes.txt', 'rb')

for line in infile:
    #print line[0:40]    
    #shaped = False
    linestring = line.strip()

    if linestring.startswith("this.registerShapedRecipe"):
        #name, recipe = getShapedRecipe(linestring)
        getShapedRecipe(linestring)
        shaped = True

    elif linestring.startswith("this.registerShapelessRecipe"):
        #name, recipe = getShapelessRecipe(linestring)
        shaped = False

    #recipes.append((name,recipe))


#for item in recipes:
#    print item
