import unittest
import opencog.spacetime #for import spacetime types
from opencog.spatial import Octree3DMapManager
from opencog.atomspace import AtomSpace,Handle,types

class TestMap(unittest.TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        resolution = 1
        floor_height = -255
        agent_height = 1.6
        self.testmap = Octree3DMapManager.init_new_map(self.atomspace, "testmap", resolution,
                                                       floor_height, agent_height)

    def tearDown(self):
        del self.testmap
        del self.atomspace

    def test_GetMapName(self):
        self.assertEqual("testmap", self.testmap.get_map_name())

    def test_GetBlock_NoBlockAdded_ReturnUndefinedHandle(self):
        test_pos = (7, 8, 9)

        test_handle = self.testmap.get_block(test_pos)

        self.assertTrue(test_handle.is_undefined())

    def testBinaryAddandRemove_NormalUnitBlock_AllGetFunctionWork(self):
        test_pos1 = (7, 8, 9)
        test_handle1 = Handle(100)

        self.testmap.add_solid_unit_block(test_handle1, test_pos1)
        test_handle2 = self.testmap.get_block(test_pos1)
        self.assertEqual(test_handle1, test_handle2)
        self.assertTrue(self.testmap.check_is_solid(test_pos1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))

        self.testmap.remove_solid_unit_block(test_handle1)
        test_handle3 = self.testmap.get_block(test_pos1)
        self.assertTrue(test_handle3.is_undefined())
        self.assertFalse(self.testmap.check_is_solid(test_pos1))
        self.assertIsNone(self.testmap.get_block_location(test_handle1))

    def testAddSolidUnitBlock__PositionOverBorder__GetBlockFailed(self):
        border = 32768
        test_pos1 = (border, 8, 9)
        test_handle1 = Handle(100)

        self.testmap.add_solid_unit_block(test_handle1, test_pos1)
        test_handle2 = self.testmap.get_block(test_pos1)

        self.assertTrue(test_handle2.is_undefined())

    def testSetBlock_AddBlockWithProbabilityControl_GetFunctionsWorkWithProb(self):
        test_pos1 = (7, 8, 9)
        test_handle1 = Handle(100)
        log_odds_threshold = self.testmap.get_log_odds_occupied_threshold()

        self.testmap.set_unit_block(test_handle1, test_pos1, log_odds_threshold)

        #test binary get_block
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1, log_odds_threshold))
        self.assertTrue(self.testmap.check_is_solid(test_pos1))
        # test prob check_is_solid
        self.assertTrue(self.testmap.check_is_solid(test_pos1, log_odds_threshold))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))
        #test get_block_location
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1,
                                                                    log_odds_threshold))

        #change the occupancy so it's small enough to make getter find nothing
        self.testmap.set_unit_block(test_handle1, test_pos1, -0.1)

        self.assertTrue(self.testmap.get_block(test_pos1).is_undefined())
        self.assertTrue(self.testmap.get_block(test_pos1, log_odds_threshold).is_undefined())
        self.assertFalse(self.testmap.check_is_solid(test_pos1))
        self.assertFalse(self.testmap.check_is_solid(test_pos1, log_odds_threshold))
        self.assertIsNone(self.testmap.get_block_location(test_handle1))
        self.assertIsNone(self.testmap.get_block_location(test_handle1,log_odds_threshold))

        #change the threshold, so the occupancy is large enough to find it
        self.testmap.set_log_odds_occupied_threshold(-0.2)
        log_odds_threshold = self.testmap.get_log_odds_occupied_threshold()
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1, log_odds_threshold))

        self.assertTrue(self.testmap.check_is_solid(test_pos1))
        self.assertTrue(self.testmap.check_is_solid(test_pos1, log_odds_threshold))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1,
                                                                    log_odds_threshold))

    def testStandable_NormalBlock_Standable(self):
        #case1: single block
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.assertFalse(self.testmap.check_standable(test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1").h
        material_node = self.atomspace.add_node(types.ConceptNode, "dirt").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material").h
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node]).h
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                            [material_pred_node,list_link]).h
        self.testmap.add_solid_unit_block(test_block, block_pos)

        standable = self.testmap.check_standable(test_pos)

        self.assertTrue(standable)

    def testStandable_WaterBlock_Unstandable(self):
        #case2: single block which is water, cannot stand on water
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.assertFalse(self.testmap.check_standable(test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1").h
        material_node = self.atomspace.add_node(types.ConceptNode, "water").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material").h
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node]).h
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                          [material_pred_node,list_link]).h
        self.testmap.add_solid_unit_block(test_block, block_pos)

        standable = self.testmap.check_standable(test_pos)

        self.assertFalse(standable)


    def testStandable_TwoNearBlock_Unstandable(self):
        # case3: two block which z coord is close ( ditance < agentHeight)
        # so it's not standable
        test_pos = (1, 2, 4)
        block_pos1 = (1, 2, 3)
        self.assertFalse(self.testmap.check_standable(test_pos))
        test_block1 = self.atomspace.add_node(types.StructureNode, "block1").h
        material_node1 = self.atomspace.add_node(types.ConceptNode, "dirt").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode,"material").h
        list_link1 = self.atomspace.add_link(types.ListLink, [test_block1,material_node1]).h
        eval_link1 = self.atomspace.add_link(types.EvaluationLink,
                                             [material_pred_node,list_link1]).h
        self.testmap.add_solid_unit_block(test_block1, block_pos1)

        blockpos2 = (1, 2, 5)
        testBlock2 = self.atomspace.add_node(types.StructureNode, "block2").h
        material_node2 = self.atomspace.add_node(types.ConceptNode, "stone").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material").h
        list_link2 = self.atomspace.add_link(types.ListLink, [testBlock2, material_node2]).h
        eval_link2 = self.atomspace.add_link(types.EvaluationLink,
                                             [material_pred_node,list_link2]).h
        self.testmap.add_solid_unit_block(testBlock2, blockpos2)
        self.assertFalse(self.testmap.get_block(blockpos2).is_undefined())
        self.assertFalse(self.testmap.get_block(block_pos1).is_undefined())
        standable = self.testmap.check_standable(test_pos)

        self.assertFalse(standable)

    def testAddandRemoveNoneBlockEntity_NormalEntity_AllGetFunctionsWork(self):
        test_pos = (17, 28, 39)
        timestamp = 12345
        test_handle1 = self.atomspace.add_node(types.EntityNode, "entity").h
        entity_is_self = False
        entity_is_avatar = False

        self.testmap.add_none_block_entity(test_handle1,test_pos,
                                           entity_is_self, entity_is_avatar,
                                           timestamp)

        test_handle2 = self.testmap.get_entity(test_pos)
        self.assertEqual(test_handle1, test_handle2)
        self.assertEqual(test_pos, self.testmap.get_last_appeared_location(test_handle1))
        self.testmap.remove_none_block_entity(test_handle1)

        self.assertTrue(self.testmap.get_entity(test_pos).is_undefined())
        #preserve record
        self.assertEqual(test_pos, self.testmap.get_last_appeared_location(test_handle1))

    def testUpdateEntityLocation_MultipleLocation_LastLocationIsPos2(self):
        test_pos1 = (17, 28, 39)
        timestamp1 = 12345
        test_handle1 = self.atomspace.add_node(types.EntityNode, "entity").h
        entity_is_self = False
        entity_is_avatar = False

        self.testmap.add_none_block_entity(test_handle1, test_pos1,
                                           entity_is_self, entity_is_avatar,
                                           timestamp1)

        test_pos2 = (17, 28, 40)
        timestamp2 = 12346
        self.testmap.update_none_block_entity_location(test_handle1, test_pos2, timestamp2)
        last_location = self.testmap.get_last_appeared_location(test_handle1)

        self.assertEqual(test_pos2, last_location)

    def testAddEntity_PressureTest(self):
        count = 10000
        timestamp = 12345
        while count != 0:
            print count
            count -= 1
            test_pos = (count, count, count)
            test_handle1 = self.atomspace.add_node(types.EntityNode, "entity"+str(count)).h
            entity_is_self = False
            entity_is_avatar = False

            self.testmap.add_none_block_entity(test_handle1, test_pos,
                                               entity_is_self, entity_is_avatar, timestamp)
            test_handle2 = self.testmap.get_entity(test_pos)
            self.assertEqual(test_handle1, test_handle2)
            self.assertEqual(test_pos,self.testmap.get_last_appeared_location(test_handle1))
            self.testmap.remove_none_block_entity(test_handle1)

            self.assertTrue(self.testmap.get_entity(test_pos).is_undefined())
            #preserve record
            self.assertEqual(test_pos,self.testmap.get_last_appeared_location(test_handle1))
