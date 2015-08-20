from opencog.spacetime import SpaceServer, TimeServer
from opencog.atomspace import AtomSpace, Handle, types

class TestTimeServer:

    def setUp(self):
        self._atomspace = AtomSpace()
        self._spaceserver = SpaceServer(self._atomspace)
        self._timeserver = TimeServer(self._atomspace, self._spaceserver)
        self._spaceserver.set_time_server(self._timeserver)
        
    def tearDown(self):
        del self._spaceserver
        del self._atomspace

    def test_addTimeInfo(self):
        objnode = self._atomspace.add_node(types.StructureNode, "object111")
        at_time_link = self._timeserver.add_time_info(objnode.h, 123456)
        assert objnode in self._atomspace.get_outgoing(at_time_link)
