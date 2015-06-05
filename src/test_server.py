import rospy

#Note:must import spacetime before atomspace, 
#or the types class won't include spacetime atom types
from opencog.spacetime import SpaceServer,TimeServer
from opencog.atomspace import AtomSpace,Handle,types



class PerceptionManager:

    def __init__(self, atomspace):
        self._atomspace = atomspace

    def buildBlockNodes(self,data):
        for block in data.visibleblocks:
            objnode=self._atomspace.add_node(types.StructureNode,"%s".format(time.time()))
            self._atomspace.add_link(types.EvaluationLink,[
                self._atomspace.add_node(types.PredicateNode,"MC_position"),
                self._atomspace.add_link(types.ListLink,[
                    objnode,
                    self._atomspace.add_node(types.NumberNode,str(block.x)),
                    self._atomspace.add_node(types.NumberNode,str(block.y)),
                    self._atomspace.add_node(types.NumberNode,str(block.z))])])

            self._atomspace.add_link(types.EvaluationLink,[
                self._atomspace.add_node(types.PredicateNode,"material"),
                self._atomspace.add_link(types.ListLink,[
                    objnode,
                    self._atomspace.add_node(types.ConceptNode,data.btype)])])

    def remove(self):
        # make sure this is called by the time script exits
        finalize_opencog()
        del self._atomspace


def main():
    a=AtomSpace()
    pm=PerceptionManager(a)
    ss = SpaceServer(self._atomspace)
    ts = TimeServer(_atomspace, _spaceserver)
    ss.setTimeServer(self._timeserver)
    initialize_opencog(self._atomspace)

    rospy.init_node('OpenCog Perception')
    rospy.Subscriber('visibility_data',visibility_msg,processMapMessage)    
    rospy.Subscriber('entity_data',entity_msg,processEntityMessage)

    while not rospy.is_shutdown():
        rospy.sleep(1.)

    pm.remove()

def processMapMessage(data,pm,ss,ts):
    maphandles=a.get_atoms_by_name(types.SpaceMapNode,data.scene)
    
    if maphandles is None:
        maphandle=ss.addMap(time.time(),data.scene,-255,-255,-255,255,255,255,0)
    else: maphandle=maphandles[0]

    curscenemap=ss.getMap(maphandle)
    
    for block in data.blocks:

        blockinfo=curscenemap.getBlockAtLocation(block.x,block.y,block.z)
        if blockinfo is not None:
            if blockinfo.getMaterial()==block.material:
                ts.addTimeInfo(blockinfo)
                pass
            else:
                ss.removeMapInfo(blockhandle,maphandle,timestamp)
                pm.forgetblock(blockhandle)
        else:
            blockhandle=pm.buildBlockNode(block)
            ss.addMapInfo(blockhandle,maphandle,False,time.time(),block.x,block.y,block.z,block.l,block.w,block.h,0,0,'block',blockhandle.name(),block.material)
        

def processEntityMessage(data):
    pass
