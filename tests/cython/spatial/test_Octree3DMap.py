from opencog.spatial import Octree3DMapManager    
from opencog.atomspace import AtomSpace,Handle,types

class TestMap:

    def setUp(self):
        self._atomspace=AtomSpace()
        self._testmap=Octree3DMapManager.initNewMap(self._atomspace,"testmap",1,0,1)
    def test_getMapName(self):
        assert self._testmap.getMapName()=="testmap"

    def test_addBlock(self):
        objnode=self._atomspace.add_node(types.ConceptNode,"object111")
        self._testmap.addSolidUnitBlock((2,3,4),objnode.h) 
        assert self._testmap.getBlockLocation(objnode.h)==(2,3,4)
        assert self._testmap.getBlock((2,3,4))==objnode.h
        #if obj is not in spacemap, getObjectLocation should return None
        notinmapobj=self._atomspace.add_node(types.ConceptNode,"object222")
        assert self._testmap.getBlockLocation(notinmapobj.h)==None
