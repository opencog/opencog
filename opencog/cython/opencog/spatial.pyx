from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport Handle, AtomSpace

cdef class OctomapOcTreeNode:
    cdef cOctomapOcTreeNode* c_node

    def __cinit__(self):
        pass

    def __init__(self, long addr):
        self.c_node = <cOctomapOcTreeNode*> PyLong_AsVoidPtr(addr)

    def __dealloc__(self):
        #OctomapOcTree will handle this
        pass

    def get_log_odds(self):
        return self.c_node.getLogOdds()

cdef class OctomapOcTree:
    cdef cOctomapOcTree* c_octree_map

    def __cinit__(self):
        pass

    def __init__(self, long addr):
        self.c_octree_map = <cOctomapOcTree*> PyLong_AsVoidPtr(addr)

    def __dealloc__(self):
        #SpaceServer will handle this
        pass

    @classmethod
    def init_new_map(cls, map_name, resolution):
        cdef cOctomapOcTree* cspmap = new cOctomapOcTree(map_name, resolution)

        newmap = cls(PyLong_FromVoidPtr(cspmap))
        return newmap

    def get_occupancy_thres_log(self):
        return self.c_octree_map.getOccupancyThresLog()

    def set_occupancy_thres(self,prob):
        self.c_octree_map.setOccupancyThres(prob)

    def search(self, pos):
        cdef cOctomapOcTreeNode* c_node = self.c_octree_map.search(pos[0], pos[1], pos[2])
        return OctomapOcTreeNode(PyLong_FromVoidPtr(c_node))

    def get_map_name(self):
        cdef string c_map_name = self.c_octree_map.getMapName()
        return c_map_name.decode('UTF-8')

    def get_agent_height(self):
        return self.c_octree_map.getAgentHeight()

    def set_agent_height(self,height):
        self.c_octree_map.setAgentHeight(height)

    def get_total_unit_block_num(self):
        return self.c_octree_map.getTotalUnitBlockNum()

    def add_solid_unit_block(self, Handle handle, pos):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_octree_map.addSolidUnitBlock(deref((<Handle>handle).h), c_pos)

    def remove_solid_unit_block(self, Handle handle):
        self.c_octree_map.removeSolidUnitBlock(deref((<Handle>handle).h))

    def set_unit_block(self,Handle handle, pos, update_log_odds_occupancy):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_octree_map.setUnitBlock(deref((<Handle>handle).h), c_pos,
                                       update_log_odds_occupancy)

    def get_block(self, pos, log_odds_occupancy = None):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        if log_odds_occupancy is None:
            log_odds_occupancy = self.c_octree_map.getOccupancyThresLog()
        cdef cHandle cblock = self.c_octree_map.getBlock(c_pos, log_odds_occupancy)
        return Handle(cblock.value())

    def get_block_location(self, Handle handle, log_odds_occupancy = None):
        if log_odds_occupancy is None:
            log_odds_occupancy = self.c_octree_map.getOccupancyThresLog()
        cdef cHandle c_handle = deref((<Handle>handle).h)
        cdef cBlockVector c_pos = self.c_octree_map.getBlockLocation(c_handle,
                                                                     log_odds_occupancy)
        #TODO: in BlockVector it returns BlockVector::ZERO, which is (0,0,0)
        #if there's really one block in the zero point this will fail
        if (c_pos.x, c_pos.y, c_pos.z) == (0,0,0):
            return None
        return (c_pos.x, c_pos.y, c_pos.z)

cdef class EntityRecorder:
    cdef cEntityRecorder* c_entity_recorder

    def __cinit__(self):
        pass

    def __init__(self):
        self.c_entity_recorder = new cEntityRecorder()

    def __dealloc__(self):
        #SpaceServer will handle this
        pass

    def get_self_agent_entity(self):
        return Handle(self.c_octree_map.getSelfAgentEntity().value())

    def add_none_block_entity(self, Handle handle, pos, isSelfObject, isAvatarEntity, timestamp):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_entity_recorder.addNoneBlockEntity(deref((<Handle>handle).h), c_pos,
                                             isSelfObject, isAvatarEntity, timestamp)

    def remove_none_block_entity(self, Handle handle):
        self.c_entity_recorder.removeNoneBlockEntity(deref((<Handle>handle).h))

    def update_none_block_entity_location(self, Handle handle, pos, timestamp):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_entity_recorder.updateNoneBlockEntityLocation(deref((<Handle>handle).h), c_pos,
                                                        timestamp)

    def get_last_appeared_location(self, Handle handle):
        cdef cHandle c_handle = deref((<Handle>handle).h)
        cdef cBlockVector c_pos = self.c_entity_recorder.getLastAppearedLocation(c_handle)
        return (c_pos.x, c_pos.y, c_pos.z)

    def get_entity(self, pos):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        cdef cHandle c_handle = self.c_entity_recorder.getEntity(c_pos)
        return Handle(c_handle.value())

def check_standable(AtomSpace atomspace, OctomapOcTree space_map, pos, log_odds_occupancy = None):
    assert len(pos) == 3
    cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
    if log_odds_occupancy is None:
        log_odds_occupancy = space_map.c_octree_map.getOccupancyThresLog()
    return checkStandableWithProb(deref(atomspace.atomspace), deref(space_map.c_octree_map), c_pos, log_odds_occupancy)

def get_near_free_point(AtomSpace atomspace, OctomapOcTree octree_map, dest,
                        dist, first_direction, bool to_be_stand):
    cdef cAtomSpace* c_atomspace = atomspace.atomspace
    cdef cOctomapOcTree* c_octree_map = octree_map.c_octree_map
    assert len(dest) == 3
    cdef cBlockVector c_dest = cBlockVector(dest[0], dest[1], dest[2])
    assert len(first_direction) == 3
    cdef cBlockVector c_first_direction = cBlockVector(first_direction[0],
                                                       first_direction[1],
                                                       first_direction[2])

    cdef cBlockVector c_pos = getNearFreePointAtDistance(deref(c_atomspace),
                                                         deref(c_octree_map),
                                                         c_dest,
                                                         dist, c_first_direction,
                                                         to_be_stand)
    if (c_pos.x, c_pos.y, c_pos.z) == (0,0,0):
        return None
    return (c_pos.x, c_pos.y, c_pos.z)
