import unittest
import opencog.spacetime #for import spacetime types
from opencog.spatial import Octree3DMapManager    
from opencog.atomspace import AtomSpace,Handle,types

class TestMap(unittest.TestCase):

    def setUp(self):
        self._atomspace=AtomSpace()
        resolution=1
        floorHeight=-255
        agentHeight=1.6
        self._testmap=Octree3DMapManager.initNewMap(self._atomspace,"testmap",resolution,floorHeight,agentHeight)

    def tearDown(self):
        del self._testmap
        del self._atomspace

    def test_getMapName(self):
        self.assertEqual("testmap",self._testmap.getMapName())

    def test_GetBlock_NoBlockAdded_ReturnUndefinedHandle(self):
        testpos=(7,8,9)

        testHandle=self._testmap.getBlock(testpos)

        self.assertTrue(testHandle.is_undefined())

    def testBinaryAddandRemove_NormalUnitBlock_AllGetFunctionWork(self):
        testpos1=(7,8,9)
        testHandle1=Handle(100)

        self._testmap.addSolidUnitBlock(testHandle1,testpos1)
        testHandle2=self._testmap.getBlock(testpos1)
        self.assertEqual(testHandle1,testHandle2)
        self.assertTrue(self._testmap.checkIsSolid(testpos1))
        self.assertEqual(testpos1,self._testmap.getBlockLocation(testHandle1))

        self._testmap.removeSolidUnitBlock(testHandle1)
        testHandle3=self._testmap.getBlock(testpos1)
        self.assertTrue(testHandle3.is_undefined())
        self.assertFalse(self._testmap.checkIsSolid(testpos1))
        self.assertIsNone(self._testmap.getBlockLocation(testHandle1))

    def testAddSolidUnitBlock__PositionOverBorder__GetBlockFailed(self):
        border=32768
        testpos1=(border,8,9)
        testHandle1=Handle(100)

        self._testmap.addSolidUnitBlock(testHandle1,testpos1)
        testHandle2=self._testmap.getBlock(testpos1)

        self.assertTrue(testHandle2.is_undefined())

    def testSetBlock_AddBlockWithProbabilityControl_GetFunctionsWorkWithProb(self):
	
        testpos1=(7,8,9)
        testHandle1=Handle(100)
        logOddThreshold=self._testmap.getLogOddsOccupiedThreshold()

        self._testmap.setUnitBlock(testHandle1,testpos1,logOddThreshold)
    
        #test binary getBlock
        self.assertEqual(testHandle1,self._testmap.getBlock(testpos1))
        self.assertEqual(testHandle1,self._testmap.getBlock(testpos1,logOddThreshold))
        self.assertTrue(self._testmap.checkIsSolid(testpos1))
        # test prob checkIsSolid
        self.assertTrue(self._testmap.checkIsSolid(testpos1,logOddThreshold))
        self.assertEqual(testpos1,self._testmap.getBlockLocation(testHandle1))
        #test getBlockLocation
        self.assertEqual(testpos1,self._testmap.getBlockLocation(testHandle1,logOddThreshold))

        #change the occupancy so it's small enough to make getter find nothing
        self._testmap.setUnitBlock(testHandle1,testpos1,-0.1)

        self.assertTrue(self._testmap.getBlock(testpos1).is_undefined())
        self.assertTrue(self._testmap.getBlock(testpos1,logOddThreshold).is_undefined())
        self.assertFalse(self._testmap.checkIsSolid(testpos1))
        self.assertFalse(self._testmap.checkIsSolid(testpos1,logOddThreshold))
        self.assertIsNone(self._testmap.getBlockLocation(testHandle1))
        self.assertIsNone(self._testmap.getBlockLocation(testHandle1,logOddThreshold))
		
        #change the threshold, so the occupancy is large enough to find it
        self._testmap.setLogOddsOccupiedThreshold(-0.2)
        logOddThreshold=self._testmap.getLogOddsOccupiedThreshold()
        self.assertEqual(testHandle1,self._testmap.getBlock(testpos1))
        self.assertEqual(testHandle1,self._testmap.getBlock(testpos1,logOddThreshold))

        self.assertTrue(self._testmap.checkIsSolid(testpos1))
        self.assertTrue(self._testmap.checkIsSolid(testpos1,logOddThreshold))
        self.assertEqual(testpos1,self._testmap.getBlockLocation(testHandle1))
        self.assertEqual(testpos1,self._testmap.getBlockLocation(testHandle1,logOddThreshold))
	
    def testStandable_NormalBlock_Standable(self):
	
        #case1: single block
        testpos=(1,2,4)
        blockpos1=(1,2,3)
        self.assertFalse(self._testmap.checkStandable(testpos))
        testBlock1=self._atomspace.add_node(types.StructureNode,"block1").h
        materialNode1=self._atomspace.add_node(types.ConceptNode,"dirt").h
	materialPredNode=self._atomspace.add_node(types.PredicateNode,"material").h
        listLink=self._atomspace.add_link(types.ListLink,
                                                  [testBlock1,materialNode1]).h
        evalLink=self._atomspace.add_link(types.EvaluationLink,
                                                  [materialPredNode,listLink]).h
        self._testmap.addSolidUnitBlock(testBlock1,blockpos1)

        standable=self._testmap.checkStandable(testpos)

        self.assertTrue(standable)
	

	
    def testStandable_WaterBlock_Unstandable(self):
	
        #case2: single block which is water, cannot stand on water
        testpos=(1,2,4)
        blockpos1=(1,2,3)
        self.assertFalse(self._testmap.checkStandable(testpos))
        testBlock1=self._atomspace.add_node(types.StructureNode,"block1").h
        materialNode1=self._atomspace.add_node(types.ConceptNode,"water").h
	materialPredNode=self._atomspace.add_node(types.PredicateNode,"material").h
        listLink=self._atomspace.add_link(types.ListLink,
                                                  [testBlock1,materialNode1]).h
        evalLink=self._atomspace.add_link(types.EvaluationLink,
                                                  [materialPredNode,listLink]).h
        self._testmap.addSolidUnitBlock(testBlock1,blockpos1)

        standable=self._testmap.checkStandable(testpos)

        self.assertFalse(standable)
	
	
    def testStandable_TwoNearBlock_Unstandable(self):
	
        # case3: two block which z coord is close ( ditance < agentHeight)
        # so it's not standable
        testpos=(1,2,4)
        blockpos1=(1,2,3)
        self.assertFalse(self._testmap.checkStandable(testpos))
        testBlock1=self._atomspace.add_node(types.StructureNode,"block1").h
        materialNode1=self._atomspace.add_node(types.ConceptNode,"dirt").h
	materialPredNode=self._atomspace.add_node(types.PredicateNode,"material").h
        listLink1=self._atomspace.add_link(types.ListLink,
                                           [testBlock1,materialNode1]).h
        evalLink1=self._atomspace.add_link(types.EvaluationLink,
                                           [materialPredNode,listLink1]).h
        self._testmap.addSolidUnitBlock(testBlock1,blockpos1)
		
        blockpos2=(1,2,5)
        testBlock2=self._atomspace.add_node(types.StructureNode,"block2").h
        materialNode2=self._atomspace.add_node(types.ConceptNode,"stone").h
	materialPredNode=self._atomspace.add_node(types.PredicateNode,"material").h
        listLink2=self._atomspace.add_link(types.ListLink,
                                           [testBlock2,materialNode2]).h
        evalLink2=self._atomspace.add_link(types.EvaluationLink,
                                           [materialPredNode,listLink2]).h
        self._testmap.addSolidUnitBlock(testBlock2,blockpos2)
        self.assertFalse(self._testmap.getBlock(blockpos2).is_undefined())
        self.assertFalse(self._testmap.getBlock(blockpos1).is_undefined())
        standable=self._testmap.checkStandable(testpos)
    
        self.assertFalse(standable)
	


    def testAddandRemoveNoneBlockEntity_NormalEntity_AllGetFunctionsWork(self):
	
        testpos1=(17,28,39)
	timestamp1=12345
        testHandle1=self._atomspace.add_node(types.EntityNode,"entity").h
        entityIsSelf=False
        entityIsAvatar=False

        self._testmap.addNoneBlockEntity(testHandle1,testpos1,
					entityIsSelf, entityIsAvatar, 
					timestamp1)

        testHandle2=self._testmap.getEntity(testpos1)
        self.assertEqual(testHandle1,testHandle2)
        self.assertEqual(testpos1,self._testmap.getLastAppearedLocation(testHandle1))
        self._testmap.removeNoneBlockEntity(testHandle1)
		
        self.assertTrue(self._testmap.getEntity(testpos1).is_undefined())
        #preserve record
        self.assertEqual(testpos1,self._testmap.getLastAppearedLocation(testHandle1))

    def testUpdateEntityLocation_MultipleLocation_LastLocationIsPos2(self):

        testpos1=(17,28,39)
	timestamp1=12345
        testHandle1=self._atomspace.add_node(types.EntityNode,"entity").h
        entityIsSelf=False
        entityIsAvatar=False

        self._testmap.addNoneBlockEntity(testHandle1,testpos1,
					entityIsSelf, entityIsAvatar, 
					timestamp1)

        testpos2=(17,28,40)
        timestamp2=12346
        self._testmap.updateNoneBlockEntityLocation(testHandle1,testpos2,timestamp2)
        lastLocation=self._testmap.getLastAppearedLocation(testHandle1)

        self.assertEqual(testpos2,lastLocation)

    def testAddEntity_PressureTest(self):
	
        count=10000
        timestamp=12345
        while count!=0:
            print count
            count-=1
            testpos1=(count,count,count)
            testHandle1=self._atomspace.add_node(types.EntityNode,"entity"+str(count)).h
            entityIsSelf=False
            entityIsAvatar=False

            self._testmap.addNoneBlockEntity(testHandle1,testpos1,
                                             entityIsSelf, entityIsAvatar, timestamp)
            testHandle2=self._testmap.getEntity(testpos1)
            self.assertEqual(testHandle1,testHandle2)
            self.assertEqual(testpos1,self._testmap.getLastAppearedLocation(testHandle1))
            self._testmap.removeNoneBlockEntity(testHandle1)
			
            self.assertTrue(self._testmap.getEntity(testpos1).is_undefined())
            #preserve record
            self.assertEqual(testpos1,self._testmap.getLastAppearedLocation(testHandle1))	
		
	
