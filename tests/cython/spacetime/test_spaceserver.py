from opencog.spacetime import SpaceServer,TimeServer
from opencog.atomspace import AtomSpace,Handle,types

class TestSpaceServer:

    def setUp(self):
        self._atomspace=AtomSpace()
        self._testserver=SpaceServer(self._atomspace)
        self._timeserver=TimeServer(self._atomspace,self._testserver)
        self._testserver.setTimeServer(self._timeserver)
        
    def tearDown(self):
        del self._testserver
        del self._atomspace

    def test_addMap(self):
        handle=self._testserver.addMap(123456,"testmap",1,0,1)
        assert self._atomspace.get_name(handle)=="testmap"
    
    def test_getMap(self):
        handle=self._testserver.addMap(123456,"testmap",1,0,1)
        mapinstance=self._testserver.getMap(handle)
        assert mapinstance.getMapName()=="testmap"

    
    def test_addAndRemoveMapInfo(self):
        maphandle=self._testserver.addMap(123456,"testmap",1,0,1)
        objnode=self._atomspace.add_node(types.StructureNode,"object111")
        assert self._testserver.addMapInfo(objnode.h,maphandle,False,False,123456,4,5,6)==True
        mapinstance=self._testserver.getMap(maphandle)        
        assert mapinstance.getBlockLocation(objnode.h)==(4,5,6)
        self._testserver.removeMapInfo(objnode.h,maphandle,234567)
        assert mapinstance.getBlockLocation(objnode.h)==None    
