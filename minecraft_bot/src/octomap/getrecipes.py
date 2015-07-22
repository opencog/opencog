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

    ns = namelist[0].find('Blocks.')
    if ns < 0:
        ns = namelist[0].find('Items.')

    if ns >= 0:
        recipe['name'] = namelist[0][ns:]
    
    if len(namelist) > 1:
        recipe['num'] = int(namelist[1])
    else:
        recipe['num'] = 1
    
    #if namelist[0].find('Items'):
    #    ntype = 'item'
    #elif namelist[0].find('Blocks'):
    #    ntype = 'block'
    #else:
    #    ntype = 'none'
    
    #print recipe['name']
    
    #recipe['type'] = ntype

    ri1 = line.find('{') + 1
    ri2 = line.find('}')

    shapelist = [item.strip() for item in line[ri1:ri2].split(",")]
    #print shapelist

    layers = []
    for item in shapelist:
        found = item.find('Character.valueOf')
        if found < 0:
            layers.append(item.strip('\"'))
        else:
            break

    recipe['shape'] = layers
    #print layers

    remaining = " ".join(shapelist[len(layers):])
    #print remaining
    
    #print len(remaining)
    print remaining

   
    characters = [remaining[i+1]
            for i in range(len(remaining))
            if remaining[i] == "\'" and remaining[i+2] == "\'"]
    #print characters

    objects = []
    substring = remaining
    while len(substring) > 0:
        founditem = substring.find("Items.")
        foundblock = substring.find("Blocks.")
        
        # if nothing found, break
        if founditem < 0 and foundblock < 0:
            break

        #if only one exists, pick it
        if founditem < 0:
            foundobject = foundblock
        elif foundblock < 0:
            foundobject = founditem

        # otherwise both exist, so pick the smaller
        else:
            foundobject = min(foundblock, founditem)
        
        substring = substring[foundobject:]
        foundspace = substring.find(" ")
        
        if foundspace >= 0:
            item = substring[0:foundspace+1]
            substring = substring[foundspace+1:]
            objects.append(item.strip())
        else:
            item = substring[0:]
            objects.append(item.strip())
            break
    
    #print objects
    recipe['mats'] = {}
    for char,obj in zip(characters, objects):
        recipe['mats'][char] = obj

    print recipe
    return recipe


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
