#! /usr/bin/env python

from opencog.atomspace import AtomSpace,Handle,TruthValue,types,get_refreshed_types
from opencog.spacetime import SpaceServer,TimeServer
#update spacetime types imported
types=get_refreshed_types()
from atomspace_util import addPredicate,addLocation

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

        locationlink=addLocation(self._atomspace,objnode,maphandle,
                                 [block.x,block.y,block.z])
        updatedevallinks.append(locationlink)

        typenode=self._atomspace.add_node(types.ConceptNode,str(block.blockid))
        materiallink=addPredicate(self._atomspace,"material",
                                  [objnode,typenode])
        updatedevallinks.append(materiallink)
        return objnode,updatedevallinks

    def buildEntityNode(self,entity,maphandle):
        entitynode=self._atomspace.add_node(types.ObjectNode,str(entity.eid))
        updatedevallinks=[]

        locationlink=addLocation(self._atomspace,entitynode,maphandle,
                                 [entity.x,entity.y,entity.z])
        updatedevallinks.append(locationlink)

        typenode=self._atomspace.add_node(types.ConceptNode,str(entity.mob_type))
        typelink=addPredicate(self._atomspace,"entitytype",
                              [entitynode,typenode])
        updatedevallinks.append(typelink)

        yawnode=self._atomspace.add_node(types.NumberNode,str(entity.head_yaw))
        pitchnode=self._atomspace.add_node(types.NumberNode,str(entity.head_pitch))
        looklink=addPredicate(self._atomspace,"look",
                              [entitynode,yawnode,pitchnode])
        updatedevallinks.append(looklink)

        lengthnode=self._atomspace.add_node(types.NumberNode,str(entity.length))
        widthnode=self._atomspace.add_node(types.NumberNode,str(entity.width))
        heightnode=self._atomspace.add_node(types.NumberNode,str(entity.height))
        sizelink=addPredicate(self._atomspace,"size",
                              [entitynode,
                               lengthnode,widthnode,heightnode])               
        updatedevallinks.append(sizelink)        

        v_xnode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_x))
        v_ynode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_y))
        v_znode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_z))
        velocitylink=addPredicate(self._atomspace,"velocity",
                                  [entitynode,
                                   v_xnode,v_ynode,v_znode])
        updatedevallinks.append(velocitylink)

        return entitynode,updatedevallinks

    def updateEntityNode(self,entitynode,entity,maphandle):
        
        updatedevallinks=[]

        locationlink=addLocation(self._atomspace,entitynode,maphandle,
                                 [entity.x,entity.y,entity.z])
        updatedevallinks.append(locationlink)

        yawnode=self._atomspace.add_node(types.NumberNode,str(entity.head_yaw))
        pitchnode=self._atomspace.add_node(types.NumberNode,str(entity.head_pitch))
        looklink=addPredicate(self._atomspace,"look",
                              [entitynode,yawnode,pitchnode])
        updatedevallinks.append(looklink)

        v_xnode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_x))
        v_ynode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_y))
        v_znode=self._atomspace.add_node(types.NumberNode,str(entity.velocity_z))
        velocitylink=addPredicate(self._atomspace,"velocity",
                                  [entitynode,
                                   v_xnode,v_ynode,v_znode])
        updatedevallinks.append(velocitylink)

                
        return updatedevallinks
        

    def processANewWorld(self,data):
        #TODO:For now the border of map is defined by Brad's flat world size,from(x,z)=(-100,-100) to (0,0)
        #But actually it should receive these borders argument from ROS to initialize the SpaceMap
        self._spaceserver.addMap(data.timestamp,"MCWorld",
                                 -100,-100,-100,
                                 100,100,100,-100)

    def processBlockMessage(self,block):
            #TODO:the unit measurement of timestamp in game is per sec
    #Need a more accurate timestamp
    #->ROS node will send two time:one is time in game world and one is time in ROS timer
        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,block.scene)[0]).h
        except IndexError:
            return 
        
        curscenemap=self._spaceserver.getMap(maphandle)        
        blockhandle=curscenemap.getUnitBlockHandleFromLocation((block.x,block.y,block.z))
        #Check if it's undefined, note that we don't use the is_undefined()
        #see the bug info in spacetime.pyx
        if blockhandle.value() != -1:
            blockinfo=curscenemap.getBlockAtLocation((block.x,block.y,block.z))
            if blockinfo.getMaterial()==str(block.blockid):
                return
            else:
                print blockinfo.getMaterial(),str(block.blockid)
                print (block.x,block.y,block.z)
                self._spaceserver.removeMapInfo(blockhandle,maphandle,block.ROStimestamp)
                #TODO: add a predicate to mark block is disappeared->This predicate should be added later in PredicateUpdater
        if block.blockid==0:
            return
        blocknode,updatedatoms=self.buildBlockNodes(block,maphandle)
        self._spaceserver.addMapInfo(blocknode.h,maphandle,False,block.ROStimestamp,
                                     block.x,block.y,block.z,
                                     1,1,1,0,
                                     'block',blocknode.name,str(block.blockid))
        self._timeserver.addTimeInfo(blocknode.h,block.ROStimestamp,TruthValue(1.0,1.0e35),"ROStimestamp")
        self._timeserver.addTimeInfo(blocknode.h,block.MCtimestamp,TruthValue(1.0,1.0e35),"MCtimestamp")
        for link in updatedatoms:
            self._timeserver.addTimeInfo(link.h,block.ROStimestamp,TruthValue(1.0,1.0e35),"ROStimestamp")
            self._timeserver.addTimeInfo(link.h,block.MCtimestamp,TruthValue(1.0,1.0e35),"MCtimestamp")

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
                if blockinfo.getMaterial()==str(block.blockid):
                    continue
                else:
                    self._spaceserver.removeMapInfo(blockhandle,maphandle,data.timestamp)
                    #TODO: add a predicate to mark block is disappeared->This predicate should be added later in PredicateUpdater

            blocknode,updatedatoms=self.buildBlockNodes(block,maphandle)
            self._spaceserver.addMapInfo(blocknode.h,maphandle,False,data.timestamp,
                                         block.x,block.y,block.z,
                                         1,1,1,0,
                                         'block',blocknode.name,str(block.blockid))
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
