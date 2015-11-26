from opencog.spacetime import SpaceServer, TimeServer
from opencog.atomspace import AtomSpace, Handle, types

class TestSpaceServer:

    def setUp(self):
        self._atomspace = AtomSpace()
        self.space_server = SpaceServer(self._atomspace)
        self.time_server = TimeServer(self._atomspace, self.space_server)
        self.space_server.set_time_server(self.time_server)
        
    def tearDown(self):
        del self.space_server
        del self._atomspace

    def test_addMap(self):
        handle = self.space_server.add_map(123456, "testmap", 1, 0, 1)
        assert self._atomspace.get_name(handle) == "testmap"
    
    def test_getMap(self):
        handle = self.space_server.add_map(123456, "testmap", 1, 0, 1)
        mapinstance = self.space_server.get_map(handle)
        assert mapinstance.get_map_name() == "testmap"

    
    def test_addAndRemoveMapInfo(self):
        map_handle = self.space_server.add_map(123456,"testmap",1,0,1)
        obj_node = self._atomspace.add_node(types.StructureNode,"object111")
        assert self.space_server.add_map_info(obj_node.h, map_handle,
                                             False, False, 123456, 4, 5, 6) == True
        map_instance = self.space_server.get_map(map_handle) 
        assert map_instance.get_block_location(obj_node.h) == (4, 5, 6)
        self.space_server.remove_map_info(obj_node.h, map_handle, 234567)
        assert map_instance.get_block_location(obj_node.h) == None    
