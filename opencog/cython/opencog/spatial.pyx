from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport Handle, AtomSpace

cdef class Octree3DMapManager:
    
    cdef cOctree3DMapManager* coctree3dmap 

    def __cinit__(self):
        pass

    def __init__(self, long addr):
        self.coctree3dmap = <cOctree3DMapManager*> PyLong_AsVoidPtr(addr)

    def __dealloc__(self):
        #spaceserver will handle this
        pass

    @classmethod
    def initNewMap(self,AtomSpace atomspace,mapName, resolution, floorHeight, agentHeight):
        cdef cOctree3DMapManager* cspmap = new cOctree3DMapManager(atomspace.atomspace,mapName, resolution,floorHeight, agentHeight)

        newmap=self(PyLong_FromVoidPtr(cspmap))
        return newmap
    
    def getFloorHeight(self):
        return self.octree3dmap.getFloorHeight()

    def getMapName(self):
        cdef string cname=self.coctree3dmap.getMapName()
        return cname.decode('UTF-8')

    def getAgentHeight(self):
        return self.octree3dmap.getAgentHeight()

    def setAgentHeight(self,height):
        self.octree3dmap.setAgentHeight(height)

    def getTotalDepthOfOctree(self):
        return self.octree3dmap.getTotalDepthOfOctree()

    def getTotalUnitBlockNum(self):
        return self.octree3dmap.getTotalUnitBlockNum()

    def getLogOddsOccupiedThreshold(self):
        return self.octree3dmap.getLogOddsOccupiedThreshold()

    def setLogOddsOccupiedThreshold(self,logOddsOccupancy):
        self.octree3dmap.setLogOddsOccupiedThreshold(logOddsOccupancy)

    def getNextDistance(self):
        return self.octree3dmap.getNextDistance()

    def getKnownSpaceMinCoord(self):
        cdef cBlockVector cpos=self.coctree3dmap.getKnownSpaceMinCoord()
        return (cpos.x,cpos.y,cpos.z)

    def getKnownSpaceMaxCoord(self):
        cdef cBlockVector cpos=self.coctree3dmap.getKnownSpaceMaxCoord()
        return (cpos.x,cpos.y,cpos.z)

    def getKnownSpaceDim(self):
        cdef cBlockVector cdim=self.coctree3dmap.getKnownSpaceDim()
        return (cdim.x,cdim.y,cdim.z)
    
    def addSolidUnitBlock(self,Handle handle,pos):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        self.coctree3dmap.addSolidUnitBlock(deref((<Handle>handle).h),cpos)

    def removeSolidUnitBlock(self,Handle handle):
        self.coctree3dmap.removeSolidUnitBlock(deref((<Handle>handle).h))

    def setUnitBlock(self,Handle handle,pos,updateLogOddsOccupancy):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        self.coctree3dmap.setUnitBlock(deref((<Handle>handle).h),cpos,updateLogOddsOccupancy)

    def checkIsSolid(self,pos):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        return self.coctree3dmap.checkIsSolid(cpos)

    def checkIsSolid(self,pos,logOddsOccupancy):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        return self.coctree3dmap.checkIsSolid(cpos,logOddsOccupancy)

    def checkIsStandable(self,pos):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        return self.coctree3dmap.checkIsStandable(cpos)

    def checkIsStandable(self,pos,logOddsOccupancy):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        return self.coctree3dmap.checkIsStandable(cpos,logOddsOccupancy)

    def getBlock(self,pos):
        assert len(pos)==3
        cdef cBlockVector cvec=cBlockVector(pos[0],pos[1],pos[2])
        cdef cHandle cblock= self.coctree3dmap.getBlock(cvec)
        return Handle(cblock.value())

    def getBlock(self,pos,logOddsOccupancy):
        assert len(pos)==3
        cdef cBlockVector cvec=cBlockVector(pos[0],pos[1],pos[2])
        cdef cHandle cblock= self.coctree3dmap.getBlock(cvec,logOddsOccupancy)
        return Handle(cblock.value())
        
    def getBlockLocation(self,Handle handle):
        cdef cBlockVector cpos=self.coctree3dmap.getBlockLocation(deref((<Handle>handle).h))
        #TODO: in BlockVector it returns BlockVector::ZERO, which is (0,0,0)
        #if there's really one block in the zero point that will fail
        if (cpos.x,cpos.y,cpos.z)==(0,0,0):
            return None
        return (cpos.x,cpos.y,cpos.z)

    def getBlockLocation(self,Handle handle,logOddsOccupancy):
        cdef cBlockVector cpos=self.coctree3dmap.getBlockLocation(deref((<Handle>handle).h),logOddsOccupancy)
        #TODO: in BlockVector it returns BlockVector::ZERO, which is (0,0,0)
        #if there's really one block in the zero point that will fail
        if (cpos.x,cpos.y,cpos.z)==(0,0,0):
            return None
        return (cpos.x,cpos.y,cpos.z)

    def getBlockLogOddsOccupancy(self,pos):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        return self.coctree3dmap.getBlockLogOddsOccupancy(cpos)

    def addNoneBlockEntity(self,Handle handle, pos, isSelfObject, isAvatarEntity,timestamp):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])
        self.coctree3dmap.addNoneBlockEntity(deref((<Handle>handle).h),cpos,isSelfObject,isAvatarEntity,timestamp)

    def removeNoneBlockEntity(self,Handle handle):
        self.coctree3dmap.removeNoneBlockEntity(deref((<Handle>handle).h))

    def updateNoneBlockEntityLocation(self,Handle handle, pos, timestamp):
        assert len(pos)==3
        cdef cBlockVector cpos=cBlockVector(pos[0],pos[1],pos[2])        
        self.coctree3dmap.updateNoneBlockEntityLocation(deref((<Handle>handle).h),cpos,timestamp)
    
    def getLastAppearedLocation(self,Handle handle):
        cdef cBlockVector cpos=self.coctree3dmap.getLastAppearedLocation(deref((<Handle>handle).h))
        return (cpos.x,cpos.y,cpos.z)
    
    
