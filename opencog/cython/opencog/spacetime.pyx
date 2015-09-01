from cython.operator cimport dereference as deref

from opencog.atomspace cimport AtomSpace, Handle, TruthValue
from opencog.spatial import OctomapOcTree as SpaceMap

cdef class TimeServer:
    cdef cTimeServer* c_time_server

    def __cinit__(self,AtomSpace _a, SpaceServer _ss):
        self.c_time_server = new cTimeServer(deref((<AtomSpace>_a).atomspace), _ss.c_space_server)

    def __dealloc__(self):
        if self.c_time_server:
            del self.c_time_server

    def add_time_info(self,Handle h, timestamp,
                      time_domain = None, TruthValue tv = TruthValue(1.0,1.0e35)):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef cHandle c_at_time_link = self.c_time_server.addTimeInfo(deref((<Handle>h).h),
                                                                     timestamp,
                                                                     c_time_domain,
                                                                     deref(tv._tvptr()))
        return Handle(c_at_time_link.value())


cdef class SpaceServer:
    cdef cSpaceServer *c_space_server

    def __cinit__(self, AtomSpace _a):
        self.c_space_server = new cSpaceServer(deref((<AtomSpace>_a).atomspace))

    def __dealloc__(self):
        if self.c_space_server:
            self.c_space_server.clear()
            del self.c_space_server

    def get_map(self,Handle handle):
        cdef cHandle c_handle = deref((<Handle>handle).h)
        map_instance = SpaceMap(<long>(&(self.c_space_server.getMap(c_handle))))
        return map_instance

    def add_map(self, timestamp, map_name, resolution, agent_height, time_domain = None):
        cdef string c_map_name = map_name.encode('UTF-8')
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef cHandle ch = self.c_space_server.addOrGetSpaceMap(timestamp, c_map_name,
                                                               resolution, agent_height,
                                                               c_time_domain)
        handle = Handle(ch.value())
        return handle

    def add_map_info(self,Handle object_node, Handle space_map_node,
                     is_self_object, is_avatar_entity, timestamp,
                     obj_x, obj_y, obj_z, time_domain = None):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        cdef bool ret = self.c_space_server.addSpaceInfo(deref((<Handle>object_node).h),
                                                         deref((<Handle>space_map_node).h),
                                                         is_self_object, is_avatar_entity,
                                                         timestamp, obj_x, obj_y, obj_z,
                                                         c_time_domain)
        return ret

    def remove_map_info(self,Handle object_node,Handle space_map_node,
                        timestamp, time_domain = None):
        cdef string c_time_domain = DEFAULT_TIMEDOMAIN if time_domain is None else time_domain.encode('UTF-8')

        self.c_space_server.removeSpaceInfo(deref((<Handle>object_node).h),
                                            deref((<Handle>space_map_node).h),
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
