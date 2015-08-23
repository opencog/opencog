#!/usr/bin/env python


"""
created by Bradley Sheneman
script to parse the minecraft recipes java file to get all crafting recipes.

"""

def getRecipeName(line):

    ni1 = line.find('(', 37, -1) + 1
    ni2 = line.find(')', 37, -1)
    
    namelist = line[ni1:ni2].split(',')
    #print namelist

    ns = namelist[0].find('Blocks.')
    if ns < 0:
        ns = namelist[0].find('Items.')

    if ns >= 0:
        name = namelist[0][ns:]
    
    if len(namelist) > 1:
        num = int(namelist[1])
    else:
        num = 1

    return name, num



def getObjects(objectstring):
    
    objects = []
    substring = objectstring
    
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
    
    return objects



def getShapedRecipe(line):

    recipe = {}
    recipe['shape'] = True

    name, num = getRecipeName(line)
    
    recipe['name'] = name
    recipe['num'] = num

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

    remaining = " ".join(shapelist[len(layers):])
   
    characters = [remaining[i+1]
            for i in range(len(remaining))
            if remaining[i] == "\'" and remaining[i+2] == "\'"]

    objects = getObjects(remaining)
    
    recipe['mats'] = {}
    for char,obj in zip(characters, objects):
        recipe['mats'][char] = obj

    print recipe
    return recipe


def getShapelessRecipe(line):
    
    recipe = {}
    recipe['shape'] = False

    name, num = getRecipeName(line)
    
    recipe['name'] = name
    recipe['num'] = num
    
    ri1 = line.find('{') + 1
    ri2 = line.find('}')
    
    remaining = line[ri1:ri2]
    #print remaining
    
    objects = getObjects(remaining)

    recipe['mats'] = objects

    print recipe
    return recipe




def getAllRecipes():

    infile = open('mc_recipes.txt', 'rb')

    recipes = []
    
    for line in infile:
        linestring = line.strip()

        if linestring.startswith("this.registerShapedRecipe"):
            recipe = getShapedRecipe(linestring)
            recipes.append(recipe)
        elif linestring.startswith("this.registerShapelessRecipe"):
            recipe = getShapelessRecipe(linestring)
            recipes.append(recipe)

    #for recipe in recipes:
    #    if recipe['shaped']:
    #        outfile.write("RECIPE: %s\n"%recipe['name'])
    #        outfile.write("SHAPE: %s\n"%(','.join(recipe['shape'])))
    
    infile.close()



if __name__ == "__main__":
    getAllRecipes()

