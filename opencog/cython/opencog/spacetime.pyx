from cython.operator cimport dereference as deref

from opencog.atomspace cimport Atom, AtomSpace, TruthValue, void_from_candle
from opencog.spatial import OctomapOcTree as SpaceMap
from opencog.spatial import EntityRecorder

cdef class TimeServer:
    cdef cTimeServer* c_time_server

    def __cinit__(self,AtomSpace _a, SpaceServer _ss):
        self.c_time_server = new cTimeServer(deref((<AtomSpace>_a).atomspace), _ss.c_space_server)

    def __dealloc__(self):
        if self.c_time_server:
            del self.c_time_server

    def add_time_info(self, Atom atom, timestamp,
                      time_domain = None, TruthValue tv = TruthValue(1.0,1.0e35)):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef cHandle c_at_time_link = self.c_time_server.addTimeInfo(deref((<Atom>atom).handle),
                                                                     timestamp,
                                                                     c_time_domain,
                                                                     deref(tv._tvptr()))
        return Atom(void_from_candle(c_at_time_link), atom.atomspace)


cdef class SpaceServer:
    cdef cSpaceServer *c_space_server
    cdef AtomSpace atomspace

    def __cinit__(self, AtomSpace _a):
        self.atomspace = _a
        self.c_space_server = new cSpaceServer(deref((<AtomSpace>_a).atomspace))

    def __dealloc__(self):
        if self.c_space_server:
            self.c_space_server.clear()
            del self.c_space_server

    def get_map(self, Atom atom):
        cdef cHandle c_handle = deref((<Atom>atom).handle)
        map_instance = SpaceMap(<long>(&(self.c_space_server.getMap(c_handle))), self.atomspace)
        return map_instance

    def get_entity_recorder(self, Atom atom):
        cdef cHandle c_handle = deref((<Atom>atom).handle)
        er_instance = EntityRecorder(<long>(&(self.c_space_server.getEntityRecorder(c_handle))), self.atomspace)
        return er_instance

    def add_map(self, timestamp, map_name, resolution, time_domain = None):
        cdef string c_map_name = map_name.encode('UTF-8')
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef cHandle ch = self.c_space_server.addOrGetSpaceMap(timestamp, c_map_name,
                                                               resolution, c_time_domain)
        cdef Atom handle = Atom(void_from_candle(ch), self.atomspace)
        return handle

    def add_map_info(self, Atom object_node, Atom space_map_node,
                     is_self_object, is_avatar_entity, timestamp,
                     obj_x, obj_y, obj_z, time_domain = None):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef bool ret = self.c_space_server.addSpaceInfo(deref((<Atom>object_node).handle),
                                                         deref((<Atom>space_map_node).handle),
                                                         is_self_object, is_avatar_entity,
                                                         timestamp, obj_x, obj_y, obj_z,
                                                         c_time_domain)
        return ret

    def remove_map_info(self,Atom object_node,Atom space_map_node,
                        timestamp, time_domain = None):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        self.c_space_server.removeSpaceInfo(deref((<Atom>object_node).handle),
                                            deref((<Atom>space_map_node).handle),
                                            timestamp, c_time_domain)

    def clear(self):
        self.c_space_server.clear()

    def set_time_server(self,TimeServer time_server):
        self.c_space_server.setTimeServer(time_server.c_time_server)

class SpaceTimeAndAtomSpace:
    """
    A static instance for global space/time server and atomspace for embodiment
    """
    atomspace_instance = AtomSpace()
    space_server_instance = SpaceServer(atomspace_instance)
    time_server_instance = TimeServer(atomspace_instance, space_server_instance)
    space_server_instance.set_time_server(time_server_instance)
    def get_space_server(self):
        return SpaceTimeAndAtomSpace.space_server_instance

    def get_time_server(self):
        return SpaceTimeAndAtomSpace.time_server_instance

    def get_atomspace(self):
        return SpaceTimeAndAtomSpace.atomspace_instance
