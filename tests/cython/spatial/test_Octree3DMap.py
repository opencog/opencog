import unittest
import opencog.spacetime #for import spacetime types
from opencog.spatial import *
from opencog.atomspace import AtomSpace,Handle,types

class TestMap(unittest.TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        resolution = 1
        self.testmap = OctomapOcTree.init_new_map("testmap", resolution)

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
        test_pos1 = (7.0, 8.0, 9.0)
        test_handle1 = self.atomspace.add_node(types.Node,"bogus").h

        self.testmap.add_solid_unit_block(test_handle1, test_pos1)
        test_handle2 = self.testmap.get_block(test_pos1)
        self.assertEqual(test_handle1, test_handle2)
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))

        self.testmap.remove_solid_unit_block(test_handle1)
        test_handle3 = self.testmap.get_block(test_pos1)
        self.assertTrue(test_handle3.is_undefined())
        self.assertIsNone(self.testmap.get_block_location(test_handle1))

    def testAddSolidUnitBlock__PositionOverBorder__GetBlockFailed(self):
        border = 32768
        test_pos1 = (border, 8, 9)
        test_handle1 = self.atomspace.add_node(types.Node,"bogus").h

        self.testmap.add_solid_unit_block(test_handle1, test_pos1)
        test_handle2 = self.testmap.get_block(test_pos1)

        self.assertTrue(test_handle2.is_undefined())

    def testSetBlock_AddBlockWithProbabilityControl_GetFunctionsWorkWithProb(self):
        test_pos1 = (7.0, 8.0, 9.0)
        test_handle1 = self.atomspace.add_node(types.Node,"bogus").h
        log_odds_threshold = self.testmap.get_occupancy_thres_log()

        self.testmap.set_unit_block(test_handle1, test_pos1, log_odds_threshold)

        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1, log_odds_threshold))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1,
                                                                    log_odds_threshold))
        self.assertEqual(log_odds_threshold, self.testmap.search(test_pos1).get_log_odds())
        #change the occupancy so it's small enough to make getter find nothing
        self.testmap.set_unit_block(test_handle1, test_pos1, -0.1)

        self.assertTrue(self.testmap.get_block(test_pos1).is_undefined())
        self.assertTrue(self.testmap.get_block(test_pos1, log_odds_threshold).is_undefined())
        self.assertIsNone(self.testmap.get_block_location(test_handle1))
        self.assertIsNone(self.testmap.get_block_location(test_handle1,log_odds_threshold))

        #change the threshold, so the occupancy is large enough to find it
        self.testmap.set_occupancy_thres(-0.2)
        log_odds_threshold = self.testmap.get_occupancy_thres_log()
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_handle1, self.testmap.get_block(test_pos1, log_odds_threshold))

        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_handle1,
                                                                    log_odds_threshold))

    def testStandable_NormalBlock_Standable(self):
        #case1: single block
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.testmap.set_agent_height(1)
        self.assertFalse(check_standable(self.atomspace, self.testmap, test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1").h
        material_node = self.atomspace.add_node(types.ConceptNode, "dirt").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material").h
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node]).h
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                            [material_pred_node,list_link]).h
        self.testmap.add_solid_unit_block(test_block, block_pos)

        standable = check_standable(self.atomspace, self.testmap, test_pos)

        self.assertTrue(standable)

    def testStandable_WaterBlock_Unstandable(self):
        #case2: single block which is water, cannot stand on water
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.testmap.set_agent_height(1)
        self.assertFalse(check_standable(self.atomspace, self.testmap, test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1").h
        material_node = self.atomspace.add_node(types.ConceptNode, "water").h
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material").h
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node]).h
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                          [material_pred_node,list_link]).h
        self.testmap.add_solid_unit_block(test_block, block_pos)

        standable = check_standable(self.atomspace, self.testmap, test_pos)

        self.assertFalse(standable)


    def testStandable_TwoNearBlock_Unstandable(self):
        # case3: two block which z coord is close ( ditance < agentHeight)
        # so it's not standable
        test_pos = (1, 2, 4)
        block_pos1 = (1, 2, 3)
        # higher agent hieght
        self.testmap.set_agent_height(2)
        self.assertFalse(check_standable(self.atomspace, self.testmap, test_pos))
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
        standable = check_standable(self.atomspace, self.testmap, test_pos)

        self.assertFalse(standable)

