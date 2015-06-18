#! /usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import visible_blocks_msg,mobs_msg,playerpos_msg

#Note:must import spacetime before atomspace, 
#or the types class won't include spacetime atom types
from opencog.spacetime import SpaceServer,TimeServer
from opencog.atomspace import AtomSpace,Handle,types
from opencog.utilities import initialize_opencog,finalize_opencog



class PerceptionManager:

    def __init__(self, atomspace):
        self._atomspace = atomspace


    def buildBlockNodes(self,block,maphandle):

        #hack to make static object No. variable in class method
        if not hasattr(self.buildBlockNodes.__func__,"objNo"):
            self.buildBlockNodes.__func__.objNo=0            
        #Note: in 3DSpaceMap using structure node to represent block.
        objnode=self._atomspace.add_node(types.StructureNode,"obj%s".format(self.buildBlockNodes.__func__.objNo))
        self.buildBlockNodes.__func__.objNo+=1

        self._atomspace.add_link(types.AtLocationLink,[
            objnode,
            maphandle,
            self._atomspace.add_link(types.ListLink,[
                self._atomspace.add_node(types.NumberNode,str(block.x)),
                self._atomspace.add_node(types.NumberNode,str(block.y)),
                self._atomspace.add_node(types.NumberNode,str(block.z))])])
        
        self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"material"),
            self._atomspace.add_link(types.ListLink,[
                objnode,
                self._atomspace.add_node(types.ConceptNode,str(block.btype))])])
        return objnode

    def buildEntityNode(self,entity,maphandle):
        entitynode=self._atomspace.add_node(types.ObjectNode,str(entity.eid))
        self._atomspace.add_link(types.AtLocationLink,[
            entitynode,
            maphandle,
            self._atomspace.add_link(types.ListLink,[
                self._atomspace.add_node(types.NumberNode,str(entity.x)),
                self._atomspace.add_node(types.NumberNode,str(entity.y)),
                self._atomspace.add_node(types.NumberNode,str(entity.z))])])
        self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"entitytype"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.ConceptNode,str(entity.entity_type))])])
        self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"yaw"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.yaw))])])
        self._atomspace.add_link(types.EvaluationLink,[
            self._atomspace.add_node(types.PredicateNode,"size"),
            self._atomspace.add_link(types.ListLink,[
                entitynode,
                self._atomspace.add_node(types.NumberNode,str(entity.length)),
                self._atomspace.add_node(types.NumberNode,str(entity.width)),
                self._atomspace.add_node(types.NumberNode,str(entity.height))])])
                                
        return entitynode

    def updateEntityNode(self,entitynode,entity,maphandle):
        pass

    def remove(self):
        # make sure this is called by the time script exits
        finalize_opencog()
        del self._atomspace

a = AtomSpace()
pm = PerceptionManager(a)
ss = SpaceServer(a)
ts = TimeServer(a,ss)
    

def main():

    global a,pm,ss,ts
    ss.setTimeServer(ts)
    initialize_opencog(a)

    rospy.init_node('OpenCog_Perception')
    rospy.Subscriber('visible_blocks_data',visible_blocks_msg,processMapMessage)    
    rospy.Subscriber('mobs_data',mobs_msg,processMobMessage)
    rospy.Subscriber('playerpos_data',playerpos_msg,processPlayerPosMessage)
    #TODO    rospy.Subscriber('entity_data',entity_msg,processEntityMessage)

    while not rospy.is_shutdown():
        rospy.sleep(1.)

    pm.remove()

def processMapMessage(data):

    #TODO:the unit measurement of timestamp in game is per sec
    #Need a more accurate timestamp
    print 'callback, timestamp %s'%(data.timestamp)

    global a,pm,ss,ts

    try:
        maphandle=(a.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
    except IndexError:
        maphandle=(ss.addMap(data.timestamp,data.scene,-60,-30,-400,50,50,50,-370))

    curscenemap=ss.getMap(maphandle)

    for block in data.blocks:
        blockhandle=curscenemap.getUnitBlockHandleFromLocation((block.x,block.y,block.z))
        #Check if it's undefined, note that we don't use the is_undefined(),see
        #the bug info in spacetime.pyx
        if blockhandle.value() != -1:
            blockinfo=curscenemap.getBlockAtLocation((block.x,block.y,block.z))
            if blockinfo.getMaterial()==str(block.btype):
                ts.addTimeInfo(blockhandle,data.timestamp)
                continue
            else:
                ss.removeMapInfo(blockhandle,maphandle,data.timestamp)
        blocknode=pm.buildBlockNodes(block,maphandle)
        ss.addMapInfo(blocknode.h,maphandle,False,data.timestamp,
                              block.x,block.y,block.z,
                              1,1,1,0,
                              'block',blocknode.name,str(block.btype))
        ts.addTimeInfo(blocknode.h,data.timestamp)

    print pm._atomspace.size()

#TODO
def processMobMessage(data):

    global a,pm,ss,ts

    try:
        maphandle=(a.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
    except IndexError:
        maphandle=(ss.addMap(data.timestamp,data.scene,-60,-30,-400,50,50,50,-370))
    curscenemap=ss.getMap(maphandle)

    for mobdata in data.mobs:
        
        try:
            mobnode=(a.get_atoms_by_name(types.ObjectNode,str(mobdata.eid))[0])
            pm.updateEntityLocation(mobnode,mobdata,maphandle)
        except IndexError:
            mobnode=pm.buildEntityNode(mobdata,maphandle)
        SpaceServer.addMapInfo(mobnode.h,maphandle,False,data.timestamp,
                               mobdata.x,mobdata.y,mobdata.z,
                               mobdata.length,mobdata.width,mobdata.height,mobdata.yaw,
                               'mob',mobnode.name,mobdata.mob_type)
        ts.addTimeInfo(mobnode.h,data.timestamp)            
    print pm._atomspace.size()

def processPlayerPosMessage(data):

    global a,pm,ss,ts
    
    try:
        maphandle=(a.get_atoms_by_name(types.SpaceMapNode,data.scene)[0]).h
    except IndexError:
        maphandle=(ss.addMap(data.timestamp,data.scene,-60,-30,-400,50,50,50,-370))

    curscenemap=ss.getMap(maphandle)

    try:
        playernode=a.get_atoms_by_name(types.ObjectNode,"myself")[0]
    except IndexError:
        playernode=pm.buildEntityNode(data,maphandle)
        
    ss.addMapInfo(playernode.h,maphandle,True,data.timestamp,
                  data.x,data.y,data.z,
                  data.length,data.width,data.height,data.yaw,
                  'player',"myself","player")
    ts.addTimeInfo(playernode.h,data.timestamp)

if __name__ == "__main__":
    main()
