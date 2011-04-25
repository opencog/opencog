from unittest import TestCase

from opencog import AtomSpace, TruthValue

class AtomSpaceTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()

    def tearDown(self):
        pass

    def test_add_node(self):
        h1 = self.space.add_node(1,"test")
        self.assertFalse(h1.is_undefined())
        # duplicates resolve to same handle
        h2 = self.space.add_node(1,"test")
        self.assertEquals(h1,h2)

        # fails when adding with a link type
        h1 = self.space.add_node(2,"test")
        self.assertEquals(h1,None)

        # test adding with a truthvalue
        h3 = self.space.add_node(1,"test_w_tv",TruthValue(0.5,100))

        self.assertEquals(self.space.size(),2)

        h1 = self.space.add_node(1,"test",prefixed=True)
        h2 = self.space.add_node(1,"test",prefixed=True)
        self.assertNotEqual(h1,h2)
        self.assertEquals(self.space.size(),4)

        h3 = self.space.add_node(1,"test",TruthValue(0.5,100),prefixed=True)
        self.assertNotEqual(h1,h3)
        self.assertEquals(self.space.size(),5)

    def test_add_link(self):
        h1 = self.space.add_node(1,"test1")
        h2 = self.space.add_node(1,"test2")
        l1 = self.space.add_link(2,[h1,h2])
        self.assertTrue(l1 is not None)
        l2 = self.space.add_link(2,[h1,h2])
        self.assertTrue(l2 is not None)
        self.assertTrue(l2 == l1)

        h3 = self.space.add_node(1,"test3")
        l3 = self.space.add_link(2,[h1,h3],TruthValue(0.5,100))
        self.assertTrue(l3 is not None)

        # fails when adding with a link type
        h1 = self.space.add_link(1,[h1,h3])
        self.assertEquals(h1,None)





