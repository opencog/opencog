import unittest
#import opencog.spacetime #for import spacetime types
from opencog.spatial import EntityManager
from opencog.atomspace import AtomSpace,Handle,types

class TestEntityManager(unittest.TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        self.test_entity_manager = EntityManager()

    def tearDown(self):
        del self.test_entity_manager
        del self.atomspace

    def testAddandRemoveNoneBlockEntity_NormalEntity_AllGetFunctionsWork(self):
        test_pos = (17, 28, 39)
        timestamp = 12345
        test_handle1 = self.atomspace.add_node(types.EntityNode, "entity").h
        entity_is_self = False
        entity_is_avatar = False

        self.test_entity_manager.add_none_block_entity(test_handle1,test_pos,
                                                       entity_is_self, entity_is_avatar,
                                                       timestamp)

        test_handle2 = self.test_entity_manager.get_entity(test_pos)
        self.assertEqual(test_handle1, test_handle2)
        self.assertEqual(test_pos, self.test_entity_manager.get_last_appeared_location(test_handle1))
        self.test_entity_manager.remove_none_block_entity(test_handle1)

        self.assertTrue(self.test_entity_manager.get_entity(test_pos).is_undefined())
        #preserve record
        self.assertEqual(test_pos, self.test_entity_manager.get_last_appeared_location(test_handle1))

    def testUpdateEntityLocation_MultipleLocation_LastLocationIsPos2(self):
        test_pos1 = (17, 28, 39)
        timestamp1 = 12345
        test_handle1 = self.atomspace.add_node(types.EntityNode, "entity").h
        entity_is_self = False
        entity_is_avatar = False

        self.test_entity_manager.add_none_block_entity(test_handle1, test_pos1,
                                                       entity_is_self, entity_is_avatar,
                                                       timestamp1)

        test_pos2 = (17, 28, 40)
        timestamp2 = 12346
        self.test_entity_manager.update_none_block_entity_location(test_handle1, test_pos2, timestamp2)
        last_location = self.test_entity_manager.get_last_appeared_location(test_handle1)

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

            self.test_entity_manager.add_none_block_entity(test_handle1, test_pos,
                                                           entity_is_self, entity_is_avatar, timestamp)
            test_handle2 = self.test_entity_manager.get_entity(test_pos)
            self.assertEqual(test_handle1, test_handle2)
            self.assertEqual(test_pos,self.test_entity_manager.get_last_appeared_location(test_handle1))
            self.test_entity_manager.remove_none_block_entity(test_handle1)

            self.assertTrue(self.test_entity_manager.get_entity(test_pos).is_undefined())
            #preserve record
            self.assertEqual(test_pos,self.test_entity_manager.get_last_appeared_location(test_handle1))
