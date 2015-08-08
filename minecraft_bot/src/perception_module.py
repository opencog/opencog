#! /usr/bin/env python

from opencog.atomspace import AtomSpace,Handle,TruthValue,types,get_refreshed_types
from opencog.spacetime import SpaceServer,TimeServer
#update spacetime types imported
types = get_refreshed_types()
from atomspace_util import addPredicate,addLocation

default_map_timestamp = 0
default_map_name = "MCmap"
default_map_resolution = 1
default_map_agent_height = 1
default_map_floor_height = -255

class PerceptionManager:

    def __init__(self, atomspace, spaceserver,timeserver):
        self._atomspace = atomspace
        self._spaceserver = spaceserver
        self._timeserver = timeserver

        #TODO: we should receive map initialization info from ROS node.
        self._spaceserver.add_map(default_map_timestamp, default_map_name, default_map_resolution, default_map_agent_height, default_map_floor_height)

    def build_block_nodes(self,block,maphandle):

        # hack to make static object No. variable in class method
        if not hasattr(self.build_block_nodes.__func__,"objNo"):
            self.build_block_nodes.__func__.objNo=0
        # Note: in 3DSpaceMap using structure node to represent block,
        # entity node to represent entity

        objnode=self._atomspace.add_node(types.StructureNode,"obj%s"%(self.build_block_nodes.__func__.objNo))
        self.build_block_nodes.__func__.objNo+=1
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

    def processBlockMessage(self,block):

        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,default_map_name)[0]).h
        except IndexError:
            return

        curscenemap=self._spaceserver.get_map(maphandle)
        old_block_handle=curscenemap.get_block(block.x,block.y,block.z)
        updatedatoms = []
        if block.blockid != 0:
            blocknode,updatedatoms = self.build_block_nodes(block,maphandle)
        else:
            blocknode = Handle(-1)
        self._spaceserver.add_map_info(blocknode.h,maphandle,False, False,
                                       block.ROStimestamp,
                                       block.x,block.y,block.z)
        # ROS node will send two time:one is time in game world and one is time in ROS timer
        self._timeserver.add_time_info(blocknode.h, block.ROStimestamp, "ROS")
        self._timeserver.add_time_info(blocknode.h, block.MCtimestamp, "MC")
        for link in updatedatoms:
            self._timeserver.add_time_info(link.h, block.ROStimestamp, "ROS")
            self._timeserver.add_time_info(link.h, block.MCtimestamp, "MC")

    def processMapMessage(self,data):

        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,default_map_name)[0]).h
        except IndexError:
            return

        curscenemap=self._spaceserver.get_map(maphandle)

        for block in data:
            old_block_handle=curscenemap.get_block((block.x,block.y,block.z))
            updatedatoms = []
            if block.blockid != 0:
                blocknode,updatedatoms = self.build_block_nodes(block,maphandle)
            else:
                blocknode = Handle(-1)
            self._spaceserver.add_map_info(blocknode.h,maphandle,False, False,
                                           block.ROStimestamp,
                                           block.x,block.y,block.z)
            # ROS node will send two time:one is time in game world and one is time in ROS timer
            self._timeserver.add_time_info(blocknode.h,block.ROStimestamp, "ROS")
            self._timeserver.add_time_info(blocknode.h,block.MCtimestamp, "MC")
            for link in updatedatoms:
                self._timeserver.add_time_info(link.h,block.ROStimestamp, "ROS")
                self._timeserver.add_time_info(link.h,block.MCtimestamp, "MC")

    def processMobMessage(self,data):

        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,default_map_name)[0]).h
        except IndexError:
            return

        curscenemap=self._spaceserver.get_map(maphandle)

        for mob in data.mobs:

            try:
                mobnode=(self._atomspace.get_atoms_by_name(types.ObjectNode,str(mob.eid))[0])
                allupdatedatoms=self.updateEntityNode(mobnode,mob,maphandle)
            except IndexError:
                mobnode,allupdatedatoms=self.buildEntityNode(mob,maphandle)
                self._timeserver.add_time_info(mobnode.h,data.timestamp)
            self._spaceserver.add_map_info(mobnode.h,maphandle,False,data.timestamp,
                                       mob.x,mob.y,mob.z,
                                       mob.length,mob.width,mob.height,mob.head_yaw,
                                       'mob',mobnode.name,str(mob.mob_type))
            for atom in allupdatedatoms:
                self._timeserver.add_time_info(atom.h,data.timestamp)


    """
    def processPlayerPosMessage(self,data):

        try:
            maphandle=(self._atomspace.get_atoms_by_name(types.SpaceMapNode,default_map_name)[0]).h
        except IndexError:
            return

        curscenemap=self._spaceserver.get_map(maphandle)

        try:
            playernode=self._atomspace.get_atoms_by_name(types.ObjectNode,"myself")[0]
            allupdatedatoms=self.updatePlayerNode(playernode,data,maphandle)
            self._timeserver.add_time_info(atlocationlink.h,data.timestamp)

        except IndexError:
            playernode,allupdatedatoms=self.buildPlayerNode(data,maphandle)
            self._timeserver.add_time_info(playernode.h,data.timestamp)
        self._spaceserver.add_map_info(playernode.h,maphandle,True,data.timestamp,
                          data.x,data.y,data.z,
                          data.length,data.width,data.height,data.yaw,
                          'player',"myself","player")
        for atom in allupdatedatoms:
            self._timeserver.add_time_info(atom.h,data.timestamp)
    """

#    def remove(self):
        # make sure this is called by the time script exits
#        finalize_opencog()
#        del self._atomspace
