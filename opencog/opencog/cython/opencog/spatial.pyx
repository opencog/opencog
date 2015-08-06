from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport Handle, AtomSpace

cdef class Octree3DMapManager:
    cdef cOctree3DMapManager* c_octree_map

    def __cinit__(self):
        pass

    def __init__(self, long addr):
        self.c_octree_map = <cOctree3DMapManager*> PyLong_AsVoidPtr(addr)

    def __dealloc__(self):
        #spaceserver will handle this
        pass

    @classmethod
    def init_new_map(cls, AtomSpace atomspace, map_name,
                     resolution, floor_height, agent_height):
        cdef cOctree3DMapManager* cspmap = new cOctree3DMapManager(atomspace.atomspace,
                                                                   map_name, resolution,
                                                                   floor_height,
                                                                   agent_height)

        newmap = cls(PyLong_FromVoidPtr(cspmap))
        return newmap

    def get_floor_height(self):
        return self.c_octree_map.getFloorHeight()

    def get_map_name(self):
        cdef string c_map_name = self.c_octree_map.getMapName()
        return c_map_name.decode('UTF-8')

    def get_agent_height(self):
        return self.c_octree_map.getAgentHeight()

    def set_agent_height(self,height):
        self.c_octree_map.setAgentHeight(height)

    def get_total_depth_of_octree(self):
        return self.c_octree_map.getTotalDepthOfOctree()

    def get_total_unit_block_num(self):
        return self.c_octree_map.getTotalUnitBlockNum()

    def get_log_odds_occupied_threshold(self):
        return self.c_octree_map.getLogOddsOccupiedThreshold()

    def set_log_odds_occupied_threshold(self,log_odds_occupancy):
        self.c_octree_map.setLogOddsOccupiedThreshold(log_odds_occupancy)

    def get_known_space_min_coord(self):
        cdef cBlockVector c_pos = self.c_octree_map.getKnownSpaceMinCoord()
        return (c_pos.x, c_pos.y, c_pos.z)

    def get_known_space_max_coord(self):
        cdef cBlockVector c_pos = self.c_octree_map.getKnownSpaceMaxCoord()
        return (c_pos.x, c_pos.y, c_pos.z)

    def get_known_space_dim(self):
        cdef cBlockVector c_dim = self.c_octree_map.getKnownSpaceDim()
        return (c_dim.x, c_dim.y, c_dim.z)

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

    def check_standable(self, pos, log_odds_occupancy = None):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        if log_odds_occupancy is None:
            log_odds_occupancy = self.c_octree_map.getLogOddsOccupiedThreshold()
        return self.c_octree_map.checkStandable(c_pos, log_odds_occupancy)

    def get_block(self, pos, log_odds_occupancy = None):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        if log_odds_occupancy is None:
            log_odds_occupancy = self.c_octree_map.getLogOddsOccupiedThreshold()
        cdef cHandle cblock = self.c_octree_map.getBlock(c_pos, log_odds_occupancy)
        return Handle(cblock.value())

    def get_block_location(self, Handle handle, log_odds_occupancy = None):
        if log_odds_occupancy is None:
            log_odds_occupancy = self.c_octree_map.getLogOddsOccupiedThreshold()
        cdef cHandle c_handle = deref((<Handle>handle).h)
        cdef cBlockVector c_pos = self.c_octree_map.getBlockLocation(c_handle,
                                                                     log_odds_occupancy)
        #TODO: in BlockVector it returns BlockVector::ZERO, which is (0,0,0)
        #if there's really one block in the zero point this will fail
        if (c_pos.x, c_pos.y, c_pos.z) == (0,0,0):
            return None
        return (c_pos.x, c_pos.y, c_pos.z)

    def get_block_log_odds_occupancy(self, pos):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        return self.c_octree_map.getBlockLogOddsOccupancy(c_pos)

    def add_none_block_entity(self, Handle handle, pos, isSelfObject, isAvatarEntity, timestamp):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_octree_map.addNoneBlockEntity(deref((<Handle>handle).h), c_pos,
                                             isSelfObject, isAvatarEntity, timestamp)

    def remove_none_block_entity(self, Handle handle):
        self.c_octree_map.removeNoneBlockEntity(deref((<Handle>handle).h))

    def update_none_block_entity_location(self, Handle handle, pos, timestamp):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        self.c_octree_map.updateNoneBlockEntityLocation(deref((<Handle>handle).h), c_pos,
                                                        timestamp)

    def get_last_appeared_location(self, Handle handle):
        cdef cHandle c_handle = deref((<Handle>handle).h)
        cdef cBlockVector c_pos = self.c_octree_map.getLastAppearedLocation(c_handle)
        return (c_pos.x, c_pos.y, c_pos.z)

    def get_entity(self, pos):
        assert len(pos) == 3
        cdef cBlockVector c_pos = cBlockVector(pos[0], pos[1], pos[2])
        cdef cHandle c_handle = self.c_octree_map.getEntity(c_pos)
        return Handle(c_handle.value())
