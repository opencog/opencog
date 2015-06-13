#! /usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import visible_blocks_msg

#Note:must import spacetime before atomspace, 
#or the types class won't include spacetime atom types
from opencog.spacetime import SpaceServer,TimeServer
from opencog.atomspace import AtomSpace,Handle,types
from opencog.utilities import initialize_opencog,finalize_opencog



class PerceptionManager:

    def __init__(self, atomspace):
        self._atomspace = atomspace


    def buildBlockNodes(self,block,timestamp,scene):

        #hack to make static object No. variable in class method
        if not hasattr(self.buildBlockNodes.__func__,"objNo"):
            self.buildBlockNodes.__func__.objNo=0            
        #Note: in 3DSpaceMap using structure node to represent block.
        objnode=self._atomspace.add_node(types.StructureNode,"obj%s".format(self.buildBlockNodes.__func__.objNo))
        self.buildBlockNodes.__func__.objNo+=1

        mapnodes=self._atomspace.get_atoms_by_name(types.SpaceMapNode,scene)
        if mapnodes == []:
            print "there is no spacemap named %s!"%(scene)
            return Handle(-1)
        else: mapnode=mapnodes[0]
        #TODO: using AtLocationLink to save location info
        self._atomspace.add_link(types.AtLocationLink,[
            objnode,
            mapnode,
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
    #TODO    rospy.Subscriber('entity_data',entity_msg,processEntityMessage)

    while not rospy.is_shutdown():
        rospy.sleep(1.)

    pm.remove()

def processMapMessage(data):

    #TODO:the unit measurement of timestamp in game is per sec
    #Need a more accurate timestamp
    print 'callback, timestamp %s'%(data.timestamp)

    global a,pm,ss,ts

    mapnodes=a.get_atoms_by_name(types.SpaceMapNode,data.scene)
    
    if mapnodes:
        maphandle=mapnodes[0].h
    else: 
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
        blocknode=pm.buildBlockNodes(block,data.timestamp,data.scene)
        ss.addMapInfo(blocknode.h,maphandle,False,data.timestamp,
                              block.x,block.y,block.z,
                              1,1,1,0,0,
                              'block',blocknode.name,str(block.btype))
        ts.addTimeInfo(blocknode.h,data.timestamp)

    print pm._atomspace.size()

#TODO
def processEntityMessage(data):
    pass

if __name__ == "__main__":
    main()
