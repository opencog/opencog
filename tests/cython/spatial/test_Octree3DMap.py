import unittest
import opencog.spacetime #for import spacetime types
from opencog.spatial import *
from opencog.atomspace import AtomSpace,types

class TestMap(unittest.TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        resolution = 1
        self.testmap = OctomapOcTree.init_new_map(self.atomspace, "testmap", resolution)

    def tearDown(self):
        del self.testmap
        del self.atomspace

    def test_GetMapName(self):
        self.assertEqual("testmap", self.testmap.get_map_name())

    def test_GetBlock_NoBlockAdded_ReturnUndefinedHandle(self):
        test_pos = (7, 8, 9)
        test_atom = self.testmap.get_block(test_pos)
        self.assertIsNone(test_atom)

    def testBinaryAddandRemove_NormalUnitBlock_AllGetFunctionWork(self):
        test_pos1 = (7.0, 8.0, 9.0)
        test_atom1 = self.atomspace.add_node(types.Node,"bogus")

        self.testmap.add_solid_unit_block(test_atom1, test_pos1)
        test_atom2 = self.testmap.get_block(test_pos1)
        self.assertEqual(test_atom1, test_atom2)
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_atom1))

        self.testmap.remove_solid_unit_block(test_atom1)
        test_atom3 = self.testmap.get_block(test_pos1)
        self.assertIsNone(test_atom3)
        self.assertIsNone(self.testmap.get_block_location(test_atom1))

    def testAddSolidUnitBlock__PositionOverBorder__GetBlockFailed(self):
        border = 32768
        test_pos1 = (border, 8, 9)
        test_atom1 = self.atomspace.add_node(types.Node,"bogus")

        self.testmap.add_solid_unit_block(test_atom1, test_pos1)
        test_atom2 = self.testmap.get_block(test_pos1)

        self.assertIsNone(test_atom2)

    def testSetBlock_AddBlockWithProbabilityControl_GetFunctionsWorkWithProb(self):
        test_pos1 = (7.0, 8.0, 9.0)
        test_atom1 = self.atomspace.add_node(types.Node,"bogus")
        log_odds_threshold = self.testmap.get_occupancy_thres_log()

        self.testmap.set_unit_block(test_atom1, test_pos1, log_odds_threshold)

        self.assertEqual(test_atom1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_atom1, self.testmap.get_block(test_pos1, log_odds_threshold))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_atom1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_atom1,
                                                                    log_odds_threshold))
        self.assertEqual(log_odds_threshold, self.testmap.search(test_pos1).get_log_odds())
        #change the occupancy so it's small enough to make getter find nothing
        self.testmap.set_unit_block(test_atom1, test_pos1, -0.1)

        self.assertIsNone(self.testmap.get_block(test_pos1))
        self.assertIsNone(self.testmap.get_block(test_pos1, log_odds_threshold))
        self.assertIsNone(self.testmap.get_block_location(test_atom1))
        self.assertIsNone(self.testmap.get_block_location(test_atom1,log_odds_threshold))

        #change the threshold, so the occupancy is large enough to find it
        self.testmap.set_occupancy_thres(-0.2)
        log_odds_threshold = self.testmap.get_occupancy_thres_log()
        self.assertEqual(test_atom1, self.testmap.get_block(test_pos1))
        self.assertEqual(test_atom1, self.testmap.get_block(test_pos1, log_odds_threshold))

        self.assertEqual(test_pos1, self.testmap.get_block_location(test_atom1))
        self.assertEqual(test_pos1, self.testmap.get_block_location(test_atom1,
                                                                    log_odds_threshold))

    def testStandable_NormalBlock_Standable(self):
        #case1: single block
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.testmap.set_agent_height(1)
        self.assertFalse(check_standable(self.atomspace, self.testmap, test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1")
        material_node = self.atomspace.add_node(types.ConceptNode, "dirt")
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material")
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node])
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                            [material_pred_node,list_link])
        self.testmap.add_solid_unit_block(test_block, block_pos)

        standable = check_standable(self.atomspace, self.testmap, test_pos)

        self.assertTrue(standable)

    def testStandable_WaterBlock_Unstandable(self):
        #case2: single block which is water, cannot stand on water
        test_pos = (1, 2, 4)
        block_pos = (1, 2, 3)
        self.testmap.set_agent_height(1)
        self.assertFalse(check_standable(self.atomspace, self.testmap, test_pos))
        test_block = self.atomspace.add_node(types.StructureNode, "block1")
        material_node = self.atomspace.add_node(types.ConceptNode, "water")
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material")
        list_link = self.atomspace.add_link(types.ListLink, [test_block,material_node])
        eval_link = self.atomspace.add_link(types.EvaluationLink,
                                          [material_pred_node,list_link])
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
        test_block1 = self.atomspace.add_node(types.StructureNode, "block1")
        material_node1 = self.atomspace.add_node(types.ConceptNode, "dirt")
        material_pred_node = self.atomspace.add_node(types.PredicateNode,"material")
        list_link1 = self.atomspace.add_link(types.ListLink, [test_block1,material_node1])
        eval_link1 = self.atomspace.add_link(types.EvaluationLink,
                                             [material_pred_node,list_link1])
        self.testmap.add_solid_unit_block(test_block1, block_pos1)

        block_pos2 = (1, 2, 5)
        testBlock2 = self.atomspace.add_node(types.StructureNode, "block2")
        material_node2 = self.atomspace.add_node(types.ConceptNode, "stone")
        material_pred_node = self.atomspace.add_node(types.PredicateNode, "material")
        list_link2 = self.atomspace.add_link(types.ListLink, [testBlock2, material_node2])
        eval_link2 = self.atomspace.add_link(types.EvaluationLink,
                                             [material_pred_node,list_link2])
        self.testmap.add_solid_unit_block(testBlock2, block_pos2)
        self.assertTrue(self.testmap.get_block(block_pos2) != None)
        self.assertTrue(self.testmap.get_block(block_pos1) != None)
        standable = check_standable(self.atomspace, self.testmap, test_pos)

        self.assertFalse(standable)

