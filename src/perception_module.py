#! /usr/bin/env python

from opencog.atomspace import AtomSpace,Handle,types,get_refreshed_types
from opencog.spacetime import SpaceServer,TimeServer
#update spacetime types imported
types=get_refreshed_types()

#helper function for adding predicate
def addPredicate(atomspace,predicatestr,atomlist):
    return self._atomspace.add_link(types.EvaluationLink,[
        self._atomspace.add_node(types.PredicateNode,predicatestr),
        self._atomspace.add_link(types.ListLink,atomlist)])

class PerceptionManager:

    def __init__(self, atomspace, spaceserver,timeserver):
        self._atomspace = atomspace
        self._spaceserver = spaceserver
        self._timeserver = timeserver

        #TODO:For now the border of map is defined by Brad's flat world size,from(x,z)=(-100,-100) to (0,0)
        #But actually it should receive these borders argument from ROS to initialize the SpaceMap
        self._spaceserver.addMap(123456,"MCWorld",
                                 -100,-100,-100,
                                 200,200,200,-100)

    def buildBlockNodes(self,block,maphandle):

        #hack to make static object No. variable in class method
        if not hasattr(self.buildBlockNodes.__func__,"objNo"):
            self.buildBlockNodes.__func__.objNo=0            
        #Note: in 3DSpaceMap using structure node to represent block.
        objnode=self._atomspace.add_node(types.StructureNode,"obj%s"%(self.buildBlockNodes.__func__.objNo))
        self.buildBlockNodes.__func__.objNo+=1
        updatedevallinks=[]

        locationlink=self._atomspace.add_link(types.AtLocationLink,[
            objnode,
            maphandle,
            self._atomspace.add_link(types.ListLink,[
                self._atomspace.add_node(types.NumberNode,str(block.x)),
                self._atomspace.add_node(types.NumberNode,str(block.y)),
                self._atomspace.add_node(types.NumberNode,str(block.z))])])
        updatedevallinks.append(locationlink)

        # materiallink=addPredicate(self._atomspace,predicate,atomlist)
        materiallink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"material"),
            self._atomspace.add_link(types.ListLink,[
                objnode,
                self._atomspace.add_node(types.ConceptNode,str(block.btype))])])
        updatedevallinks.append(materiallink)
        return objnode,updatedevallinks

    def buildEntityNode(self,entity,maphandle):
        entitynode=self._atomspace.add_node(types.ObjectNode,str(entity.eid))
        updatedevallinks=[]

        locationlink=self._atomspace.add_link(types.AtLocationLink,[
            entitynode,
            maphandle,
            self._atomspace.add_link(types.ListLink,[
                self._atomspace.add_node(types.NumberNode,str(entity.x)),
                self._atomspace.add_node(types.NumberNode,str(entity.y)),
                self._atomspace.add_node(types.NumberNode,str(entity.z))])])
        updatedevallinks.append(locationlink)

        typelink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"type"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.ConceptNode,str(entity.mob_type))])])
        updatedevallinks.append(typelink)

        looklink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"look"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.head_yaw)),
                self._atomspace.add_node(types.NumberNode,str(entity.head_pitch))])])
        updatedevallinks.append(looklink)

        sizelink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"size"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.length)),
                self._atomspace.add_node(types.NumberNode,str(entity.width)),
                self._atomspace.add_node(types.NumberNode,str(entity.height))])])
        updatedevallinks.append(sizelink)

        looklink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"look"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.head_yaw)),
                self._atomspace.add_node(types.NumberNode,str(entity.head_pitch))])])
        updatedevallinks.append(looklink)
        

        velocitylink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"velocity"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_x)),
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_y)),
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_z))])])
        updatedevallinks.append(velocitylink)

        return entitynode,updatedevallinks

    def updateEntityNode(self,entitynode,entity,maphandle):
        
        updatedevallinks=[]
        
        locationlink=self._atomspace.add_link(types.AtLocationLink,[
            entitynode,
            maphandle,
            self._atomspace.add_link(types.ListLink,[
                self._atomspace.add_node(types.NumberNode,str(entity.x)),
                self._atomspace.add_node(types.NumberNode,str(entity.y)),
                self._atomspace.add_node(types.NumberNode,str(entity.z))])])
        updatedevallinks.append(locationlink)

        looklink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"look"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.head_yaw)),
                self._atomspace.add_node(types.NumberNode,str(entity.head_pitch))])])
        updatedevallinks.append(looklink)

        velocitylink=self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"velocity"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_x)),
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_y)),
                self._atomspace.add_node(types.NumberNode,str(entity.velocity_z))])])
        updatedevallinks.append(velocitylink)
        
        return updatedevallinks
        

    def processANewWorld(self,data):
        #TODO:For now the border of map is defined by Brad's flat world size,from(x,z)=(-100,-100) to (0,0)
        #But actually it should receive these borders argument from ROS to initialize the SpaceMap
        self._spaceserver.addMap(data.timestamp,"MCWorld",
                                 -100,-100,-100,
                                 100,100,100,-100)

    def processMapMessage(self,data):

    #TODO:the unit measurement of timestamp in game is per sec
    #Need a more accurate timestamp
    #->ROS node will send two time:one is time in game world and one is time in ROS timer
        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
        except IndexError:
            return 

        curscenemap=self._spaceserver.getMap(maphandle)
        for block in data.blocks:
            blockhandle=curscenemap.getUnitBlockHandleFromLocation((block.x,block.y,block.z))
            #Check if it's undefined, note that we don't use the is_undefined()
            #see the bug info in spacetime.pyx
            if blockhandle.value() != -1:
                blockinfo=curscenemap.getBlockAtLocation((block.x,block.y,block.z))
                if blockinfo.getMaterial()==str(block.btype):
                    continue
                else:
                    self._spaceserver.removeMapInfo(blockhandle,maphandle,data.timestamp)
                    #TODO: add a predicate to mark block is disappeared->This predicate should be added later in PredicateUpdater

            blocknode,updatedatoms=self.buildBlockNodes(block,maphandle)
            self._spaceserver.addMapInfo(blocknode.h,maphandle,False,data.timestamp,
                                         block.x,block.y,block.z,
                                         1,1,1,0,
                                         'block',blocknode.name,str(block.btype))
            self._timeserver.addTimeInfo(blocknode.h,data.timestamp)
            for link in updatedatoms:
                self._timeserver.addTimeInfo(link.h,data.timestamp)

    def processMobMessage(self,data):
        
        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
        except IndexError:
            return

        curscenemap=self._spaceserver.getMap(maphandle)

        for mob in data.mobs:
        
            try:
                mobnode=(self._atomspace.get_atoms_by_name(types.ObjectNode,str(mob.eid))[0])
                allupdatedatoms=self.updateEntityNode(mobnode,mob,maphandle)
            except IndexError:
                mobnode,allupdatedatoms=self.buildEntityNode(mob,maphandle)
                self._timeserver.addTimeInfo(mobnode.h,data.timestamp)            
            self._spaceserver.addMapInfo(mobnode.h,maphandle,False,data.timestamp,
                                       mob.x,mob.y,mob.z,
                                       mob.length,mob.width,mob.height,mob.head_yaw,
                                       'mob',mobnode.name,str(mob.mob_type))
            for atom in allupdatedatoms:
                self._timeserver.addTimeInfo(atom.h,data.timestamp)


    """
    def processPlayerPosMessage(self,data):
    
        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
        except IndexError:
            return
            
        curscenemap=self._spaceserver.getMap(maphandle)
        
        try:
            playernode=self._atomspace.get_atoms_by_name(types.ObjectNode,"myself")[0]
            allupdatedatoms=self.updatePlayerNode(playernode,data,maphandle)
            self._timeserver.addTimeInfo(atlocationlink.h,data.timestamp)

        except IndexError:
            playernode,allupdatedatoms=self.buildPlayerNode(data,maphandle)
            self._timeserver.addTimeInfo(playernode.h,data.timestamp)            
        self._spaceserver.addMapInfo(playernode.h,maphandle,True,data.timestamp,
                          data.x,data.y,data.z,
                          data.length,data.width,data.height,data.yaw,
                          'player',"myself","player")
        for atom in allupdatedatoms:
            self._timeserver.addTimeInfo(atom.h,data.timestamp)
    """

#    def remove(self):
        # make sure this is called by the time script exits
#        finalize_opencog()
#        del self._atomspace
